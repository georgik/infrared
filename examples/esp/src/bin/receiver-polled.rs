#![no_std]
#![no_main]

extern crate alloc;

use esp32c3_hal::{
    clock::ClockControl,
    pac::{self, Peripherals},
    prelude::*,
    timer::{TimerGroup},
    Rtc,
};
use esp32c3_hal::{
    gpio::{Gpio5, IO},
    gpio_types::{Input, Floating},
    pulse_control::ClockSource,
    Delay,
    PulseControl,
    utils::{smartLedAdapter, SmartLedsAdapter},
};

use smart_leds::{
    brightness,
    gamma,
    SmartLedsWrite,
    RGB8
};

use esp_backtrace as _;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

use infrared::{
    protocol::{Nec},
};

use esp_println::println;

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;

    extern "C" {
        static mut _heap_start: u32;
    }

    unsafe {
        let heap_start = &_heap_start as *const _ as usize;
        ALLOCATOR.init(heap_start as *mut u8, HEAP_SIZE);
    }
}


type IrPin = Gpio5<Input<Floating>>;
type IrReceiver = infrared::PeriodicPoll<Nec, IrPin>;
static mut RECEIVER: Option<IrReceiver> = None;

#[riscv_rt::entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let mut timer0 = timer_group0.timer0;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();


    const FREQ: u32 = 38_000;

    // Configure RMT peripheral globally
    let pulse = PulseControl::new(
        peripherals.RMT,
        &mut system.peripheral_clock_control,
        ClockSource::APB,
        0,
        0,
        0,
    )
    .unwrap();


    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    // We use one of the RMT channels to instantiate a `SmartLedsAdapter` which can
    // be used directly with all `smart_led` implementations
    let mut led = <smartLedAdapter!(1)>::new(pulse.channel0, io.pins.gpio8);

    // Initialize the Delay peripheral, and use it to toggle the LED state in a
    // loop.
    let mut delay = Delay::new(&clocks);

    let irda_pin = io.pins.gpio5.into_floating_input();
    // let mut receiver:IrReceiver = infrared::PeriodicPoll::with_pin(SAMPLERATE, irda_pin);

    // Frequency of the timer interrupt in Hz.
    let mut receiver = infrared::PeriodicPoll::<Nec, Gpio5<Input<Floating>>>::with_pin(FREQ, irda_pin);


    let white = RGB8 {
        r: 0xff,
        g: 0xff,
        b: 0xff,
    };
    let data = [white;1];
    led.write(brightness(gamma(data.iter().cloned()), 5))
        .unwrap();

    println!("Starting main loop");
    loop {

        if let Ok(Some(cmd)) = receiver.poll() {
            println!("addr:{} cmd:{}", cmd.addr, cmd.cmd);
            let color = match cmd.cmd {
                13 => RGB8 {
                    r: 0xff,
                    g: 0,
                    b: 0,
                },
                22 => RGB8 {
                    r: 0,
                    g: 0xff,
                    b: 0,
                },
                25 => RGB8 {
                    r: 0,
                    g: 0,
                    b: 0xff,
                },
                _ => RGB8 {
                    r: 0,
                    g: 0,
                    b: 0,
                }
            };
            let data = [color;1];
            led.write(brightness(gamma(data.iter().cloned()), 5))
                .unwrap();
        }
        delay.delay_us(26u32);

    }
}
