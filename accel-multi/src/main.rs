#![no_std]
#![no_main]

//use core::fmt::Write;
use esp_backtrace as _;
use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, timer::TimerGroup, Rtc, Delay, IO, cpu_control::CpuControl, 
    uart::Uart, i2c::I2C};
use spin::{Mutex, RwLock};

const BUFFER_SIZE: usize  = 575;

const IMU_ADDR: u8        = 0x6B;
const CTRL1_XL: u8        = 0x10;
const OUTX_L_A: u8        = 0x28;
const OUTY_L_A: u8        = 0x2A;
const OUTZ_L_A: u8        = 0x2C;

const ACCEL_CTRL: [u8; 2] = [CTRL1_XL, 0x90];
const ALIGN: [u8; 3]      = [0x99, 0x00, 0xF3];

fn read_accel<T>(i2c: &mut Mutex<I2C<T>>) -> [u8; 6] where T: _esp_hal_i2c_Instance {
    let mut values: [u8; 6] = [0, 0, 0, 0, 0, 0];
    let mut accel: [u8; 2] = [0, 0];
 
    (*i2c.lock()).write_read(IMU_ADDR, &[OUTX_L_A], &mut accel).unwrap();
    values[0] = accel[0]; values[1] = accel[1];

    (*i2c.lock()).write_read(IMU_ADDR, &[OUTY_L_A], &mut accel).unwrap();
    values[2] = accel[0]; values[3] = accel[1];

    (*i2c.lock()).write_read(IMU_ADDR, &[OUTZ_L_A], &mut accel).unwrap();
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

    let delay = Delay::new(&clocks);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut ctrl = io.pins.gpio2.into_push_pull_output();
    ctrl.set_high().unwrap();
    
    let mut uart0 = Uart::new(peripherals.UART0, &mut system.peripheral_clock_control);

    let i2c = I2C::new(peripherals.I2C0, io.pins.gpio22, io.pins.gpio20, 
                           hal::prelude::_fugit_RateExtU32::kHz(400).into(), &mut system.peripheral_clock_control, &clocks);

    let mut sync_i2c = Mutex::new(i2c);
    (*sync_i2c.lock()).write(IMU_ADDR, &ACCEL_CTRL).unwrap();
    
    let raw_ctrl = CpuControl::new(system.cpu_control);
    let control = spin::Mutex::new(raw_ctrl);

    let buffer1: [(u64, [u8; 6]); BUFFER_SIZE] = [(0, [0, 0, 0, 0, 0, 0]); BUFFER_SIZE];
    let buffer2: [(u64, [u8; 6]); BUFFER_SIZE] = [(0, [0, 0, 0, 0, 0, 0]); BUFFER_SIZE];

    let buf1 = Mutex::new(buffer1);
    let buf1_full = RwLock::new(false);
    
    let buf2 = Mutex::new(buffer2);
    let buf2_full = RwLock::new(false);
    
    let mut task = || {
        timer_group0.timer0.set_counter_active(true);
        let mut start = timer_group0.timer0.now();
        
        loop {
            {
                while *buf1_full.read() {delay.delay(2 as u32);}
            }
            for i in 0..BUFFER_SIZE {
                (*buf1.lock())[i] = (timer_group0.timer0.now() - start, read_accel(&mut sync_i2c));
                start = timer_group0.timer0.now();
            }
            {
                *buf1_full.write() = true;
            }
            {
                while *buf2_full.read() {delay.delay(2 as u32);}
            }
            for i in 0..BUFFER_SIZE {
                (*buf2.lock())[i] = (timer_group0.timer0.now() - start, read_accel(&mut sync_i2c));
                start = timer_group0.timer0.now();
            }
            {
                *buf2_full.write() = true;
            }
        } 
    };

    let cpu_guard = (*control.lock()).start_app_core(&mut task).unwrap();
    
    let mut temp: [u8; 8];
    
    loop {
        {
            while !(*buf1_full.read()) {delay.delay(2 as u32);}
        }
        let mut guard = *buf1.lock();
        for i in 0..BUFFER_SIZE {
            uart0.write_bytes(&guard[i].0.to_le_bytes()).unwrap();
            uart0.write_bytes(&guard[i].1).unwrap();
        }
        uart0.write_bytes(&ALIGN).unwrap();
        {
            *buf1_full.write() = false;
        }
        {
            while !(*buf2_full.read()) {delay.delay(2 as u32);}
        }
        guard = *buf2.lock();
        for i in 0..BUFFER_SIZE {
            uart0.write_bytes(&guard[i].0.to_le_bytes()).unwrap();
            uart0.write_bytes(&guard[i].1).unwrap();
        }
        uart0.write_bytes(&ALIGN).unwrap();
        {
            *buf2_full.write() = false;
        }
    }
}

