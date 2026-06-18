#![no_std]
#![no_main]

use alloc_cortex_m::CortexMHeap;
use bsp::entry;
use can2040::{Can2040, CanFrame};
use defmt::*;
use defmt_rtt as _;
use embedded_can::nb::Can;
use embedded_can::{ExtendedId, Frame};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::i2c::I2c;
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
    Timer, I2C,
};

const VESC_ID: u32 = 22;
const CAN_PACKET_SET_CURRENT: u32 = 1;
const CAN_PACKET_SET_RPM: u32 = 3;

const CAN_ID_SET_CURRENT: Option<ExtendedId> =
    ExtendedId::new((CAN_PACKET_SET_CURRENT << 8) | VESC_ID);
const CAN_ID_SET_RPM: Option<ExtendedId> = ExtendedId::new((CAN_PACKET_SET_RPM << 8) | VESC_ID);

const DELAY_MS: u32 = 100;
const CONTROL_DT_S: f32 = DELAY_MS as f32 / 1000.0;

// Acceleration tuning:
// MAX_MOTOR_CURRENT_A is the torque limit. Raise carefully; the boat can draw
// about 300 A, but start much lower during tests.
const MAX_MOTOR_CURRENT_A: f32 = 80.0;
const MAX_ERPM: i32 = 15_000;
const CURRENT_RAMP_A_PER_S: f32 = 150.0;
const RPM_RAMP_ERPM_PER_S: f32 = 25_000.0;
const RPM_MODE_MIN_THROTTLE: f32 = 0.08;

// AS5600 magnetic encoder on the 5-pin connector.
const AS5600_I2C_ADDRESS: u8 = 0x36;
const AS5600_RAW_ANGLE_REGISTER: u8 = 0x0C;

// Encoder calibration:
// 1. Put the throttle in neutral/rest position and copy the logged raw angle here.
// 2. Move the throttle in the desired forward direction.
// 3. If the logged signed offset gets negative, set ENCODER_FORWARD_SIGN to -1.
const ENCODER_ZERO_RAW: u16 = 0;
const ENCODER_FORWARD_SIGN: i32 = 1;
const ENCODER_DEADZONE_COUNTS: i32 = 35;
const ENCODER_FORWARD_MAX_COUNTS: i32 = 500;
const ENCODER_REVERSE_MAX_COUNTS: i32 = 500;

// 3-pin Hall connector on A3. Change this if the replacement Hall sensor
// reports the opposite level when the magnet/kill-cord is present.
const HALL_PRESENT_WHEN_LOW: bool = true;

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

    let registers: [u8; 1] = [AS5600_RAW_ANGLE_REGISTER];
    let mut angle_buff: [u8; 2] = [0; 2];

    let mut engine_locked = true;
    let mut controller = VescController::new();

    loop {
        led_green.set_high().unwrap();
        // use rp-pico 0.9
        let requested_throttle: f32 =
            match i2c.write_read(AS5600_I2C_ADDRESS, &registers, &mut angle_buff) {
                Ok(_) => {
                    let angle = (((angle_buff[0] as u16) << 8) | angle_buff[1] as u16) & 0x0fff;
                    let throttle = throttle_from_angle(angle);
                    info!(
                        "encoder raw: {}, offset: {}, throttle: {}",
                        angle,
                        encoder_offset(angle),
                        throttle
                    );
                    throttle
                }
                Err(_e) => {
                    warn!("could not read from i2c");
                    0.0
                }
            };

        let kill_cord_present = match hall_pin.is_low() {
            Ok(is_low) => is_low == HALL_PRESENT_WHEN_LOW,
            Err(_) => false,
        };

        if !kill_cord_present {
            engine_locked = true;
        }

        if kill_cord_present && requested_throttle == 0.0 {
            engine_locked = false;
        }

        let throttle = if engine_locked {
            led_red.set_low().unwrap();
            0.0
        } else {
            led_red.set_high().unwrap();
            requested_throttle
        };

        info!("throttle: {}", throttle);

        let command = if engine_locked {
            controller.stop_now()
        } else {
            controller.update(throttle)
        };
        let frame = match command {
            VescCommand::Current(current) => CanFrame::new(
                CAN_ID_SET_CURRENT.unwrap(),
                &((current * 1000.0) as i32).to_be_bytes(),
            )
            .unwrap(),
            VescCommand::Rpm(rpm) => {
                CanFrame::new(CAN_ID_SET_RPM.unwrap(), &(rpm as i32).to_be_bytes()).unwrap()
            }
        };

        let _ = <Can2040 as Can>::transmit(&mut can_bus, &frame)
            .inspect_err(|_e| warn!("CAN TX error would block: dropping Frame"));

        timer.delay_ms(DELAY_MS / 2);
        led_green.set_low().unwrap();
        timer.delay_ms(DELAY_MS / 2);
    }
}

enum VescCommand {
    Current(f32),
    Rpm(f32),
}

enum ControlMode {
    Current,
    Rpm,
}

struct VescController {
    mode: ControlMode,
    current_cmd_a: f32,
    rpm_cmd: f32,
}

impl VescController {
    fn new() -> Self {
        Self {
            mode: ControlMode::Current,
            current_cmd_a: 0.0,
            rpm_cmd: 0.0,
        }
    }

    fn stop_now(&mut self) -> VescCommand {
        self.mode = ControlMode::Current;
        self.current_cmd_a = 0.0;
        self.rpm_cmd = 0.0;
        VescCommand::Current(0.0)
    }

    fn update(&mut self, throttle: f32) -> VescCommand {
        let throttle = throttle.clamp(-1.0, 1.0);

        if throttle == 0.0 {
            self.mode = ControlMode::Current;
            self.current_cmd_a = ramp_towards(self.current_cmd_a, 0.0, CURRENT_RAMP_A_PER_S);
            self.rpm_cmd = ramp_towards(self.rpm_cmd, 0.0, RPM_RAMP_ERPM_PER_S);
            return VescCommand::Current(self.current_cmd_a);
        }

        let shaped_throttle = throttle * throttle.abs();
        let target_current = shaped_throttle * MAX_MOTOR_CURRENT_A;
        let target_rpm = shaped_throttle * MAX_ERPM as f32;

        self.mode = if throttle.abs() < RPM_MODE_MIN_THROTTLE {
            ControlMode::Current
        } else {
            ControlMode::Rpm
        };

        match self.mode {
            ControlMode::Current => {
                self.current_cmd_a =
                    ramp_towards(self.current_cmd_a, target_current, CURRENT_RAMP_A_PER_S);
                self.rpm_cmd = 0.0;
                VescCommand::Current(self.current_cmd_a)
            }
            ControlMode::Rpm => {
                self.rpm_cmd = ramp_towards(self.rpm_cmd, target_rpm, RPM_RAMP_ERPM_PER_S);
                self.current_cmd_a = target_current;
                VescCommand::Rpm(self.rpm_cmd)
            }
        }
    }
}

fn ramp_towards(current: f32, target: f32, rate_per_second: f32) -> f32 {
    let max_step = rate_per_second * CONTROL_DT_S;
    let diff = target - current;

    if diff > max_step {
        current + max_step
    } else if diff < -max_step {
        current - max_step
    } else {
        target
    }
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
    let raw_offset = ((angle as i32 - ENCODER_ZERO_RAW as i32 + 2048).rem_euclid(4096)) - 2048;
    raw_offset * ENCODER_FORWARD_SIGN
}
