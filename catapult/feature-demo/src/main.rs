#![no_std]
#![no_main]

// pick a panicking behavior
//use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m_rt::entry;
use microbit::{
    hal::{
        gpio::Level,
        prelude::*,
        pwm::{Channel, Pwm},
        twim, Timer,
    },
    Board,
};
use panic_rtt_target as _;
use rtt_target::rtt_init_print;
use superbit_board::{MotorDirection, MotorPosition, RgbColor, ServoPosition, SuperBit};

const COLOR_ORANGE: RgbColor = RgbColor {
    r: 255,
    g: 165,
    b: 0,
};
// const COLOR_YELLOW: RgbColor = RgbColor {
//     r: 255,
//     g: 255,
//     b: 0,
// };
const COLOR_LIME_GREEN: RgbColor = RgbColor {
    r: 127,
    g: 255,
    b: 0,
};
const COLOR_INDIGO: RgbColor = RgbColor {
    r: 75,
    g: 0,
    b: 130,
};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let board = Board::take().unwrap();
    let pwm = Pwm::new(board.PWM0);
    let pwm_digital_pin = board.pins.p0_12.degrade().into_push_pull_output(Level::Low);
    pwm.set_output_pin(Channel::C0, pwm_digital_pin);
    let external_i2c = twim::Twim::new(
        board.TWIM0,
        twim::Pins::from(board.i2c_external),
        twim::Frequency::K100,
    );
    let mut superbit = SuperBit::new(external_i2c, pwm);
    let mut event_timer = Timer::new(board.TIMER0);
    let mut display_pins = board.display_pins;
    display_pins.col1.set_low().unwrap();
    display_pins.col2.set_low().unwrap();
    display_pins.col3.set_low().unwrap();
    display_pins.col4.set_low().unwrap();
    display_pins.col5.set_low().unwrap();
    while board.buttons.button_a.is_low().unwrap() {
        if display_pins.row1.is_set_low().unwrap() {
            display_pins.row1.set_high().unwrap();
            display_pins.row2.set_high().unwrap();
            display_pins.row3.set_high().unwrap();
            display_pins.row4.set_high().unwrap();
            display_pins.row5.set_high().unwrap();
        } else {
            display_pins.row1.set_low().unwrap();
            display_pins.row2.set_low().unwrap();
            display_pins.row3.set_low().unwrap();
            display_pins.row4.set_low().unwrap();
            display_pins.row5.set_low().unwrap();
        }
        event_timer.delay_ms(1000u16);
    }
    event_timer.delay_ms(1000u16);
    let mut step = 1u8;
    loop {
        // your code goes here
        match step {
            1 => {
                display_pins.row5.set_low().unwrap();
                display_pins.row1.set_high().unwrap();
                superbit.set_all_neopixel_colors(COLOR_INDIGO).unwrap();
                superbit.neopixel_show().unwrap();
                superbit.move_270_servo(ServoPosition::S1, 100).unwrap();
                superbit
                    .drive_motor(MotorPosition::M1, 255, MotorDirection::Forward)
                    .unwrap();
                superbit
                    .drive_motor(MotorPosition::M3, 255, MotorDirection::Forward)
                    .unwrap();
            }
            2 => {
                display_pins.row1.set_low().unwrap();
                display_pins.row2.set_high().unwrap();
                superbit.set_all_neopixel_colors(COLOR_LIME_GREEN).unwrap();
                superbit.neopixel_show().unwrap();
                superbit
                    .drive_motor(MotorPosition::M1, 255, MotorDirection::Reverse)
                    .unwrap();
                superbit
                    .drive_motor(MotorPosition::M3, 255, MotorDirection::Reverse)
                    .unwrap();
            }
            3 => {
                display_pins.row2.set_low().unwrap();
                display_pins.row3.set_high().unwrap();
                superbit.set_all_neopixel_colors(COLOR_INDIGO).unwrap();
                superbit.neopixel_show().unwrap();
                superbit
                    .drive_motor(MotorPosition::M1, 255, MotorDirection::Forward)
                    .unwrap();
                superbit
                    .drive_motor(MotorPosition::M3, 255, MotorDirection::Reverse)
                    .unwrap();
            }
            4 => {
                display_pins.row3.set_low().unwrap();
                display_pins.row4.set_high().unwrap();
                superbit.set_all_neopixel_colors(COLOR_ORANGE).unwrap();
                superbit.neopixel_show().unwrap();
                superbit
                    .drive_motor(MotorPosition::M1, 255, MotorDirection::Reverse)
                    .unwrap();
                superbit
                    .drive_motor(MotorPosition::M3, 255, MotorDirection::Forward)
                    .unwrap();
            }
            5 => {
                display_pins.row4.set_low().unwrap();
                display_pins.row5.set_high().unwrap();
                superbit
                    .set_all_neopixel_colors(RgbColor { r: 255, g: 0, b: 0 })
                    .unwrap();
                superbit.neopixel_show().unwrap();
                event_timer.delay_ms(500u16);
                superbit.move_270_servo(ServoPosition::S1, 135).unwrap();
            }
            _ => {
                unreachable!()
            }
        }
        step += 1;
        if step == 6 {
            step = 1;
        }
        event_timer.delay_ms(2_000u16);
    }
}
