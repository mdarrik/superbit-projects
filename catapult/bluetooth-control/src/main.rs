#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

// pick a panicking behavior
//use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use core::{mem, str::FromStr};
use embassy_executor::executor::{Executor, Spawner};
use microbit::{
    hal::{
        gpio::Level,
        prelude::*,
        pwm::{Channel, Pwm},
        twim, Timer,
    },
    Board,
};
use nrf_softdevice::{
    ble::{
        gatt_server::{self, builder::ServiceBuilder, RegisterError},
        peripheral, Uuid,
    },
    raw, Softdevice,
};
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use static_cell::StaticCell;
use superbit_board::{MotorDirection, MotorPosition, RgbColor, ServoPosition, SuperBit};

const CATAPULT_REST_POSITION: u16 = 115;
const CATAPULT_READY_POSITION: u16 = 105;
const CATAPULT_FIRE_POSITION: u16 = 135;
const CONTROLLER_SERVICE_UUID: &'static str = "6b38635e-9fe3-41ea-a28f-74f5c807c090";
const CONTROLLER_DIRECTION_CHARACTERISTIC_UUID: &'static str =
    "6b38dee0-9fe3-41ea-a28f-74f5c807c090";
const LED_VALUES: [ColorInfo; 7] = [
    ColorInfo::None,
    ColorInfo::Color(("Red", RgbColor { r: 255, g: 0, b: 0 })),
    ColorInfo::Color((
        "Orange",
        RgbColor {
            r: 255,
            g: 165,
            b: 0,
        },
    )),
    ColorInfo::Color((
        "Yellow",
        RgbColor {
            r: 255,
            g: 255,
            b: 0,
        },
    )),
    ColorInfo::Color((
        "Green",
        RgbColor {
            r: 127,
            g: 255,
            b: 0,
        },
    )),
    ColorInfo::Color((
        "Blue",
        RgbColor {
            r: 4,
            g: 213,
            b: 237,
        },
    )),
    ColorInfo::Color((
        "Indigo",
        RgbColor {
            r: 75,
            g: 0,
            b: 130,
        },
    )),
];
static EXECUTOR: StaticCell<Executor> = StaticCell::new();
// static MICROBIT_BOARD: StaticCell<Board> = StaticCell::new();

#[nrf_softdevice::gatt_service(uuid = "6b38635e-9fe3-41ea-a28f-74f5c807c090")]
#[derive(Debug, Default)]
struct ControllerService {
    #[characteristic(uuid = "6b38dee0-9fe3-41ea-a28f-74f5c807c090", write_without_response)]
    direction_handle: u8,
    #[characteristic(uuid = "6b3897dc-9fe3-41ea-a28f-74f5c807c090", read, write, notify)]
    led_handle: u8,
    #[characteristic(uuid = "6b383ad1-9fe3-41ea-a28f-74f5c807c090", write_without_response)]
    catapult_arm_handle: u8,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy)]
enum Direction {
    Forward = 0,
    Left = 1,
    Right = 2,
    Backwards = 3,
}

#[derive(Debug, Clone, Copy)]
enum ColorInfo {
    Color((&'static str, RgbColor)),
    None,
}

#[nrf_softdevice::gatt_server]
struct ControllerServer {
    controller_service: ControllerService,
}

#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice) {
    sd.run().await;
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    rtt_init_print!();
    let mut board = Board::take().unwrap();
    let pwm = Pwm::new(board.PWM0);
    let pwm_digital_pin = board.pins.p0_12.degrade().into_push_pull_output(Level::Low);
    pwm.set_output_pin(Channel::C0, pwm_digital_pin);
    let external_i2c = twim::Twim::new(
        board.TWIM0,
        twim::Pins::from(board.i2c_external),
        twim::Frequency::K100,
    );
    let mut superbit = SuperBit::new(external_i2c, pwm);
    superbit.turn_off_all_neopixels().unwrap();
    superbit.neopixel_show().unwrap();
    superbit.move_270_servo(ServoPosition::S1, 115).unwrap();

    let softdevice_config = nrf_softdevice::Config {
        clock: Some(raw::nrf_clock_lf_cfg_t {
            source: raw::NRF_CLOCK_LF_SRC_RC as u8,
            rc_ctiv: 4,
            rc_temp_ctiv: 2,
            accuracy: 7,
        }),
        conn_gap: Some(raw::ble_gap_conn_cfg_t {
            conn_count: 2,
            event_length: 24,
        }),
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 256 }),
        gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t {
            attr_tab_size: 32768,
        }),
        gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
            adv_set_count: 1,
            periph_role_count: 3,
        }),
        gap_device_name: Some(raw::ble_gap_cfg_device_name_t {
            p_value: b"Superbit" as *const u8 as _,
            current_len: 8,
            max_len: 8,
            write_perm: unsafe { mem::zeroed() },
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(
                raw::BLE_GATTS_VLOC_STACK as u8,
            ),
        }),
        ..Default::default()
    };
    let sd = Softdevice::enable(&softdevice_config);

    let server = ControllerServer::new(sd).unwrap();
    spawner.spawn(softdevice_task(sd)).unwrap();
    let mut event_timer = Timer::new(board.TIMER2);
    let scan_data = &[0x03, 0x03, 0x09, 0x18];
    let adv_data = &[
        0x02,
        0x01,
        raw::BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE as u8,
        0x03,
        0x03,
        0x09,
        0x18,
        0x09,
        0x09,
        b'S',
        b'u',
        b'p',
        b'e',
        b'r',
        b'B',
        b'i',
        b't',
    ];
    board.display_pins.col1.set_low().unwrap();
    board.display_pins.col2.set_low().unwrap();
    board.display_pins.col3.set_low().unwrap();
    board.display_pins.col4.set_low().unwrap();
    board.display_pins.col5.set_low().unwrap();
    loop {
        let peripheral_config = peripheral::Config::default();
        let advertisement = peripheral::ConnectableAdvertisement::ScannableUndirected {
            scan_data,
            adv_data,
        };
        superbit.stop_motor(MotorPosition::M1).unwrap();
        superbit.stop_motor(MotorPosition::M3).unwrap();
        board.display_pins.row1.set_high().unwrap();
        board.display_pins.row2.set_high().unwrap();
        board.display_pins.row3.set_high().unwrap();

        let connection = peripheral::advertise_connectable(sd, advertisement, &peripheral_config)
            .await
            .unwrap();

        let server_run = gatt_server::run(&connection, &server, |event| match event {
            ControllerServerEvent::ControllerService(service_event) => match service_event {
                ControllerServiceEvent::CatapultArmHandleWrite(value) => {
                    rprintln!("arm instruction: {:?}", value);
                    if value == 1 {
                        superbit
                            .move_270_servo(ServoPosition::S1, CATAPULT_READY_POSITION)
                            .unwrap();
                        event_timer.delay_ms(100u8);
                        superbit
                            .move_270_servo(ServoPosition::S1, CATAPULT_FIRE_POSITION)
                            .unwrap();
                    } else {
                        superbit
                            .move_270_servo(ServoPosition::S1, CATAPULT_REST_POSITION)
                            .unwrap();
                    }
                }
                ControllerServiceEvent::DirectionHandleWrite(value) => {
                    rprintln!("direction instruction: {:?}", value);
                    match value {
                        direction if direction == Direction::Forward as u8 => {
                            superbit
                                .drive_motor(MotorPosition::M1, 255, MotorDirection::Forward)
                                .unwrap();
                            superbit
                                .drive_motor(MotorPosition::M3, 255, MotorDirection::Forward)
                                .unwrap();
                            event_timer.delay_ms(250u16);
                            superbit.stop_motor(MotorPosition::M1).unwrap();
                            superbit.stop_motor(MotorPosition::M3).unwrap();
                        }
                        direction if direction == Direction::Backwards as u8 => {
                            superbit
                                .drive_motor(MotorPosition::M1, 255, MotorDirection::Reverse)
                                .unwrap();
                            superbit
                                .drive_motor(MotorPosition::M3, 255, MotorDirection::Reverse)
                                .unwrap();
                            event_timer.delay_ms(250u16);
                            superbit.stop_motor(MotorPosition::M1).unwrap();
                            superbit.stop_motor(MotorPosition::M3).unwrap();
                        }
                        direction if direction == Direction::Left as u8 => {
                            superbit
                                .drive_motor(MotorPosition::M1, 255, MotorDirection::Reverse)
                                .unwrap();
                            superbit
                                .drive_motor(MotorPosition::M3, 255, MotorDirection::Forward)
                                .unwrap();
                            event_timer.delay_ms(250u16);
                            superbit.stop_motor(MotorPosition::M1).unwrap();
                            superbit.stop_motor(MotorPosition::M3).unwrap();
                        }
                        direction if direction == Direction::Right as u8 => {
                            superbit
                                .drive_motor(MotorPosition::M1, 255, MotorDirection::Forward)
                                .unwrap();
                            superbit
                                .drive_motor(MotorPosition::M3, 255, MotorDirection::Reverse)
                                .unwrap();
                            event_timer.delay_ms(250u16);
                            superbit.stop_motor(MotorPosition::M1).unwrap();
                            superbit.stop_motor(MotorPosition::M3).unwrap();
                        }
                        _ => {
                            rprintln!("invalid direction value {:?}", &value)
                        }
                    }
                }
                ControllerServiceEvent::LedHandleWrite(value) => {
                    if let Some(color_info) = LED_VALUES.get(usize::from(value)) {
                        if let ColorInfo::Color((name, color)) = *color_info {
                            superbit.set_all_neopixel_colors(color).unwrap();
                        } else {
                            superbit.turn_off_all_neopixels().unwrap();
                        }
                        superbit.neopixel_show().unwrap();
                    }
                }
                ControllerServiceEvent::LedHandleCccdWrite { notifications } => todo!(),
            },
        })
        .await;

        if let Err(e) = server_run {
            rprintln!("server exited with error: {:?}", e)
        }
    }
}
