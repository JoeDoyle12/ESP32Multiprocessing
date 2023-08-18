#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_println::{println};
use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, timer::TimerGroup, Rtc, Delay, IO, uart::Uart, i2c::I2C};


const IMU_ADDR: u8        = 0x6B;
const CTRL1_XL: u8        = 0x10;
const OUTX_L_A: u8        = 0x28;
const OUTY_L_A: u8        = 0x2A;
const OUTZ_L_A: u8        = 0x2C;

const ACCEL_CTRL: [u8; 2] = [CTRL1_XL, 0x90];
const ALIGN: [u8; 3]      = [0x01, 0x02, 0x03];

fn read_accel<T>(i2c: &mut I2C<T>) -> [u8; 6] where T: _esp_hal_i2c_Instance {
    let mut values: [u8; 6] = [0, 0, 0, 0, 0, 0];
    let mut accel: [u8; 2] = [0, 0];
 
    i2c.write_read(IMU_ADDR, &[OUTX_L_A], &mut accel).unwrap();
    values[0] = accel[0]; values[1] = accel[1];

    i2c.write_read(IMU_ADDR, &[OUTY_L_A], &mut accel).unwrap();
    values[2] = accel[0]; values[3] = accel[1];

    i2c.write_read(IMU_ADDR, &[OUTZ_L_A], &mut accel).unwrap();
    values[4] = accel[0]; values[5] = accel[1];
    
    return values;
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let mut timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();
    
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut ctrl = io.pins.gpio2.into_push_pull_output();
    ctrl.set_high().unwrap();
    
    let mut uart0 = Uart::new(peripherals.UART0, &mut system.peripheral_clock_control);

    let mut i2c = I2C::new(peripherals.I2C0, io.pins.gpio22, io.pins.gpio20, 
                           hal::prelude::_fugit_RateExtU32::kHz(400).into(), &mut system.peripheral_clock_control, &clocks);

    i2c.write(IMU_ADDR, &ACCEL_CTRL).unwrap();
    
    timer_group0.timer0.set_counter_active(true);
    let mut start = timer_group0.timer0.now();
    let mut t: (u64, [u8; 6]) = (0, [0; 6]);
    let mut total_time = 0;
    for _ in 0..100000{
        t = (timer_group0.timer0.now() - start, read_accel(&mut i2c));
        total_time += timer_group0.timer0.now() - start;
        start = timer_group0.timer0.now();
        uart0.write_bytes(&t.0.to_le_bytes()).unwrap();
        uart0.write_bytes(&t.1).unwrap();
        uart0.write_bytes(&ALIGN).unwrap();
    }
    println!("\nTotal Time: {}\nAverage Time: {}", total_time, (total_time as f64) / 100000.0);
    loop {}
}
