use std::time::Duration;
use byteorder::{LittleEndian, ReadBytesExt};

const PORT_NAME: &str     = "/dev/tty.usbserial-56470257961";
const BAUD_RATE: u32      = 115200;
const BUFFER_SIZE: usize  = 575;   // or 300 if multi
const TO_READ: usize      = 10; // or 3 if multi

const g: f64              = 9.80665;
const BYTES_TO_ACCEL: f64 = 4.0 * g / (0xFFFF as f64);

const ALIGN: [u8; 3]      = [0x99, 0x00, 0xF3];

fn print_data(data: &[u8]) -> () {
    let mut total_time: u64 = 0;
    for i in 0..BUFFER_SIZE * TO_READ {
        //let mut t: &[u8] = &data[i * 14..i * 14 + 8];
        let time: u64 = (&data[i * 14..i * 14 + 8]).read_u64::<LittleEndian>().unwrap();
        
        let accelX: f64 = ((&data[i * 14 + 8..i * 14 + 10]).read_i16::<LittleEndian>().unwrap() as f64) * BYTES_TO_ACCEL;
        let accelY: f64 = ((&data[i * 14 + 10..i * 14 + 12]).read_i16::<LittleEndian>().unwrap() as f64) * BYTES_TO_ACCEL;
        let accelZ: f64 = ((&data[i * 14 + 12..(i + 1) * 14]).read_i16::<LittleEndian>().unwrap() as f64) * BYTES_TO_ACCEL;
        
        println!("{} {} {} {}", time, accelX, accelY, accelZ);
        total_time += time;
    }
    println!("Average Time: {}\nTotal Time: {}", (total_time as f64) / ((BUFFER_SIZE * TO_READ) as f64), total_time);
}

fn main() {
    let port = serialport::new(PORT_NAME, BAUD_RATE)
        .timeout(Duration::from_secs(10))
        .open();
    
    let mut serial_buf: [u8; BUFFER_SIZE * TO_READ * 14] = [0; BUFFER_SIZE * TO_READ * 14];

    match port {
        Ok(mut port) => {
            let mut llb: u8 = 0x00;
            let mut lb: u8 = 0x00;
            let mut b: [u8; 1] = [0x00];
            loop {
                port.read_exact(&mut b).unwrap();
                if llb == ALIGN[0] && lb == ALIGN[1] && b[0] == ALIGN[2] {
                    //println!("Aligned {} {} {}", llb, lb, b[0]);
                    port.read_exact(&mut serial_buf[0..BUFFER_SIZE * 14]).unwrap();
                    port.read_exact(&mut [0, 0, 0]).unwrap();
                    break;
                }
                llb = lb;
                lb = b[0];
            }
            //println!("Receiving data on {} at {} baud:", &PORT_NAME, &BAUD_RATE); 
            //port.read_exact(&mut serial_buf);
            for i in 1..TO_READ {
                port.read_exact(&mut serial_buf[i * BUFFER_SIZE * 14..(i + 1) * BUFFER_SIZE * 14]).unwrap();
                port.read_exact(&mut [0, 0, 0]).unwrap();
                //println!("{:?}", t)
                //println!("reset");
            }
            println!("{:?}", serial_buf);
            print_data(&serial_buf);
        }
        Err(e) => {
            eprintln!("Failed to open \"{}\". Error: {}", PORT_NAME, e);
            ::std::process::exit(1);
        }
    }
}
