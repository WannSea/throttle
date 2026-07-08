#![no_std]
#![no_main]

use alloc_cortex_m::CortexMHeap;
use bsp::entry;
use can2040::{Can2040, CanFrame};
use core::fmt::Write;
use defmt::*;
use defmt_rtt as _;
use embedded_can::nb::Can;
use embedded_can::{ExtendedId, Frame, Id};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::i2c::I2c;
use heapless::String;
use panic_probe as _;
use usb_device::{
    class_prelude::UsbBusAllocator,
    device::{StringDescriptors, UsbDevice},
    prelude::*,
};
use usbd_serial::SerialPort;

use seeeduino_xiao_rp2040 as bsp;

const CONFIG_CANBUS_FREQUENCY: u32 = 250_000;
const CONFIG_RP2040_CANBUS_GPIO_RX: u32 = 26;
const CONFIG_RP2040_CANBUS_GPIO_TX: u32 = 27;
const ENABLE_USB_LOGGING: bool = false;

#[global_allocator]
pub static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

pub fn init_allocator() {
    // Please set the correct heap size.
    const HEAP_SIZE: usize = 0x8000;
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
    unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP.len()) }
}

use bsp::hal::{
    clocks::init_clocks_and_plls,
    fugit::RateExtU32,
    gpio::{FunctionI2C, Pin},
    pac,
    sio::Sio,
    usb::UsbBus,
    watchdog::Watchdog,
    Timer, I2C,
};

#[allow(dead_code)]
#[repr(u8)]
enum CanCommands {
    SetDuty = 0,
    SetCurrent = 1,
    SetCurrentBrake = 2,
    SetRpm = 3,
    SetPos = 4,
}

const VESC_ID: u32 = 22;
const CAN_ID_SET_CURRENT: Option<ExtendedId> =
    ExtendedId::new((CanCommands::SetCurrent as u32) << 8 | VESC_ID);
const CAN_ID_UNLOCK_THROTTLE: Option<ExtendedId> = ExtendedId::new(0x00000F55);
const CAN_UNLOCK_THROTTLE_DATA: [u8; 1] = [0xA5];

const DELAY_MS: u32 = 20;
const SMOOTH_SAMPLES: usize = 50;
const MAX_CURRENT_A: f32 = 300.0;

// AS5600 magnetic encoder on the 5-pin connector
const AS5600_I2C_ADDRESS: u8 = 0x36;
const AS5600_RAW_ANGLE_REGISTER: u8 = 0x0C;

// Encoder calibration:
// 1. Put the throttle in neutral/rest position and copy the logged raw angle here.
// 2. Move the throttle in the desired forward direction.
// 3. If the logged signed offset gets negative, set ENCODER_FORWARD_SIGN to -1.
const ENCODER_ZERO_RAW: u16 = 2237;
const ENCODER_FORWARD_SIGN: i32 = 1;
const ENCODER_DEADZONE_COUNTS: i32 = 50;
const ENCODER_FORWARD_MAX_COUNTS: i32 = 2382;
const ENCODER_REVERSE_MAX_COUNTS: i32 = 1315;

// 3-pin Hall connector on A3. Change this if the replacement Hall sensor
// reports the opposite level when the magnet/kill-cord is present.
const HALL_PRESENT_WHEN_LOW: bool = true;
const PRETEND_HALL_PIN_ON: bool = false;

#[entry]
fn main() -> ! {
    init_allocator();
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let clocks = init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_green = pins.led_green.into_push_pull_output();
    let mut led_red = pins.led_red.into_push_pull_output();

    // Configure two pins as being I²C, not GPIO
    let sda_pin: Pin<_, FunctionI2C, _> = pins.sda.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, _> = pins.scl.reconfigure();

    // Create the I²C driver, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let mut i2c = I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        200.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );
    let mut can_bus = can2040::initialize_cbus(
        &mut core,
        CONFIG_CANBUS_FREQUENCY,
        CONFIG_RP2040_CANBUS_GPIO_RX,
        CONFIG_RP2040_CANBUS_GPIO_TX,
    );

    let mut hall_pin = pins.a3.into_floating_input();

    let usb_bus = if ENABLE_USB_LOGGING {
        Some(UsbBusAllocator::new(UsbBus::new(
            pac.USBCTRL_REGS,
            pac.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut pac.RESETS,
        )))
    } else {
        None
    };
    let mut serial = usb_bus.as_ref().map(SerialPort::new);
    let mut usb_dev = usb_bus.as_ref().map(|usb_bus| {
        UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .strings(&[StringDescriptors::default()
                .manufacturer("WannSea")
                .product("Gashebel Setup Serial")
                .serial_number("throttle")])
            .unwrap()
            .device_class(2)
            .build()
    });

    let registers: [u8; 1] = [AS5600_RAW_ANGLE_REGISTER];
    let mut angle_buff: [u8; 2] = [0; 2];

    let mut engine_locked = true;
    let mut can_unlock_received = false;
    let mut smooth: [i32; SMOOTH_SAMPLES] = [0; SMOOTH_SAMPLES];
    let mut index = 0;

    loop {
        poll_usb_serial(&mut usb_dev, &mut serial);
        if poll_can_unlock(&mut can_bus) {
            can_unlock_received = true;
        }

        led_green.set_high().unwrap();
        // use rp-pico 0.9
        let current: i32 = match i2c.write_read(AS5600_I2C_ADDRESS, &registers, &mut angle_buff) {
            Ok(_) => {
                let angle = (((angle_buff[0] as u16) << 8) | angle_buff[1] as u16) & 0x0fff;
                let throttle = throttle_from_angle(angle);
                let offset = encoder_offset(angle);
                info!(
                    "encoder raw: {}, offset: {}, throttle: {}",
                    angle, offset, throttle
                );
                write_usb_encoder_log(&mut serial, angle, offset, throttle);
                current_from_throttle(throttle)
            }
            Err(_e) => {
                warn!("could not read from i2c");
                write_usb_line(&mut serial, "could not read from i2c\r\n");
                0
            }
        };

        let kill_cord_present = if PRETEND_HALL_PIN_ON {
            true
        } else {
            match hall_pin.is_low() {
                Ok(is_low) => is_low == HALL_PRESENT_WHEN_LOW,
                Err(_) => false,
            }
        };

        if !kill_cord_present {
            engine_locked = true;
            can_unlock_received = false;
        }

        if kill_cord_present && can_unlock_received && current == 0 {
            engine_locked = false;
        }

        let throttle = if engine_locked {
            led_red.set_low().unwrap();
            0
        } else {
            led_red.set_high().unwrap();
            current
        };

        info!("throttle: {}", throttle);

        if index >= smooth.len() {
            index = 0;
        }

        let transmit_throttle = if throttle == 0 {
            smooth = [0; SMOOTH_SAMPLES];
            0
        } else {
            smooth[index] = throttle;
            (smooth.iter().sum::<i32>() as f32 / smooth.len() as f32) as i32
        };

        let frame =
            CanFrame::new(CAN_ID_SET_CURRENT.unwrap(), &transmit_throttle.to_be_bytes()).unwrap();
        let _ = <Can2040 as Can>::transmit(&mut can_bus, &frame)
            .inspect_err(|_e| warn!("CAN TX error would block: dropping Frame"));

        delay_ms_maybe_usb_poll(&mut timer, &mut usb_dev, &mut serial, DELAY_MS / 2);
        led_green.set_low().unwrap();
        delay_ms_maybe_usb_poll(&mut timer, &mut usb_dev, &mut serial, DELAY_MS / 2);
        index += 1;
    }
}

fn poll_can_unlock(can_bus: &mut Can2040) -> bool {
    let mut unlock_received = false;

    while let Ok(frame) = <Can2040 as Can>::receive(can_bus) {
        if frame.id() == Id::Extended(CAN_ID_UNLOCK_THROTTLE.unwrap())
            && frame.data() == CAN_UNLOCK_THROTTLE_DATA
        {
            unlock_received = true;
        }
    }

    unlock_received
}

fn delay_ms_maybe_usb_poll(
    timer: &mut Timer,
    usb_dev: &mut Option<UsbDevice<UsbBus>>,
    serial: &mut Option<SerialPort<UsbBus>>,
    ms: u32,
) {
    if !ENABLE_USB_LOGGING {
        timer.delay_ms(ms);
        return;
    }

    for _ in 0..ms {
        poll_usb_serial(usb_dev, serial);
        timer.delay_ms(1);
    }
}

fn poll_usb_serial(
    usb_dev: &mut Option<UsbDevice<UsbBus>>,
    serial: &mut Option<SerialPort<UsbBus>>,
) {
    let (Some(usb_dev), Some(serial)) = (usb_dev.as_mut(), serial.as_mut()) else {
        return;
    };

    if usb_dev.poll(&mut [serial]) {
        let mut buf = [0u8; 64];
        let _ = serial.read(&mut buf);
    }
}

fn write_usb_encoder_log(
    serial: &mut Option<SerialPort<UsbBus>>,
    angle: u16,
    offset: i32,
    throttle: f32,
) {
    if !ENABLE_USB_LOGGING {
        return;
    }

    let mut line: String<96> = String::new();
    let throttle_milli = (throttle * 1000.0) as i32;
    let _ = writeln!(
        &mut line,
        "encoder raw: {}, offset: {}, throttle_milli: {}\r",
        angle, offset, throttle_milli
    );
    write_usb_line(serial, line.as_str());
}

fn write_usb_line(serial: &mut Option<SerialPort<UsbBus>>, line: &str) {
    if let Some(serial) = serial.as_mut() {
        let _ = serial.write(line.as_bytes());
    }
}

fn current_from_throttle(throttle: f32) -> i32 {
    (throttle.clamp(-1.0, 1.0) * MAX_CURRENT_A * 1000.0) as i32
}

fn throttle_from_angle(angle: u16) -> f32 {
    let offset = encoder_offset(angle);

    if offset.abs() <= ENCODER_DEADZONE_COUNTS {
        return 0.0;
    }

    let throttle = if offset > 0 {
        (offset - ENCODER_DEADZONE_COUNTS) as f32
            / (ENCODER_FORWARD_MAX_COUNTS - ENCODER_DEADZONE_COUNTS) as f32
    } else {
        (offset + ENCODER_DEADZONE_COUNTS) as f32
            / (ENCODER_REVERSE_MAX_COUNTS - ENCODER_DEADZONE_COUNTS) as f32
    };

    throttle.clamp(-1.0, 1.0)
}

fn encoder_offset(angle: u16) -> i32 {
    let raw_offset = angle as i32 - ENCODER_ZERO_RAW as i32;
    raw_offset * ENCODER_FORWARD_SIGN
}
