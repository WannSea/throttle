#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use embedded_hal::digital::{OutputPin,InputPin};
use alloc_cortex_m::CortexMHeap;
use can2040::{Can2040, CanFrame};
use embedded_can::nb::Can;
use embedded_can::{ExtendedId, Frame};
use panic_probe as _;

use seeeduino_xiao_rp2040 as bsp;

const CONFIG_CANBUS_FREQUENCY: u32 = 250_000;
const CONFIG_RP2040_CANBUS_GPIO_RX: u32 = 26;
const CONFIG_RP2040_CANBUS_GPIO_TX: u32 = 27;

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
    watchdog::Watchdog,
    Timer,
    I2C,
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

const VESC_ID :u32 = 22;
const CAN_ID_SET_DUTY: Option<ExtendedId> = ExtendedId::new((CanCommands::SetDuty as u32) << 8 | VESC_ID);

const DELAY_MS: u32 = 100;


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
    let sda_pin: Pin<_, FunctionI2C,_> = pins.sda.reconfigure();
    let scl_pin: Pin<_, FunctionI2C,_> = pins.scl.reconfigure();

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
    
    let mut stop_pin = pins.a3.into_floating_input();

    let address: u8 = 0x06;
    let registers: [u8; 2] = [0x03,0x04];
    let mut angle_buff: [u8; 2] = [0; 2];
    
    let mut engine_locked = true;

    loop {
        led_green.set_high().unwrap(); 
        // use rp-pico 0.9
        let duty: i32 = match i2c.write_read(address, &registers, &mut angle_buff) {
            Ok(_) => {
                let angle = ((angle_buff[0] as u16) << 6) | angle_buff[1] as u16;
                info!("buff: {}, angle: {}", angle_buff, angle);
                angle_map(angle)
            },
            Err(_e) =>{
                warn!("could not read from i2c");
                0
            }
        };

        let kill_cord_present = match stop_pin.is_low(){
            Ok(present) => present,
            Err(_) => false
        }; 

        if !kill_cord_present {
            engine_locked=true;
        }

        if kill_cord_present && duty==0 {
            engine_locked=false;
        }

        let throttle = if engine_locked {
            led_red.set_low().unwrap();
            0
        } else {
            led_red.set_high().unwrap();
            duty
        };




        info!("throttle: {}", throttle);
        
        let frame = CanFrame::new(CAN_ID_SET_DUTY.unwrap(),&throttle.to_be_bytes()).unwrap();
        let _ = <Can2040 as Can>::transmit(&mut can_bus, &frame).inspect_err(|_e| warn!("CAN TX error would block: dropping Frame"));

        timer.delay_ms(DELAY_MS/2);
        led_green.set_low().unwrap();
        timer.delay_ms(DELAY_MS/2);
    }
}

fn angle_map(angle: u16) -> i32 {
    const ANGLE_MIN: u16 = 3720;
    const ANGLE_DEADZONE_START: u16 = 9120;
    const ANGLE_DEADZONE_END: u16 = 9888;
    const ANGLE_MAX: u16 = 10250;
    const SIGN: f32 = -1.0;

    let mut throttle: f32 = 0.0;


    if angle < ANGLE_DEADZONE_START {
        throttle =  -((ANGLE_DEADZONE_START-angle) as f32)*SIGN / (ANGLE_DEADZONE_START-ANGLE_MIN) as f32
    }
    if angle > ANGLE_DEADZONE_END {
        throttle =  ((angle-ANGLE_DEADZONE_END) as f32)*SIGN / (ANGLE_MAX-ANGLE_DEADZONE_END) as f32
    }

    return (throttle.clamp(-1.0,1.0) * 100_000.0 ) as i32
}
