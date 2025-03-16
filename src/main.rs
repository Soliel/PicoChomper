#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]
use core::time::Duration;

use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use defmt::{unwrap, info};
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::DMA_CH0;
use embassy_rp::{bind_interrupts, peripherals::{PIO0, PIO1}};
use embassy_rp::pio::{Instance, InterruptHandler, Pio};
use embassy_rp::pio_programs::pwm::{PioPwm, PioPwmProgram};
use embassy_time::{Duration as EmbassyDuration, Timer};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

const DEFAULT_MIN_PULSE_WIDTH: u64 = 500; // uncalibrated default, the shortest duty cycle sent to a servo
const DEFAULT_MAX_PULSE_WIDTH: u64 = 2500; // uncalibrated default, the longest duty cycle sent to a servo
const DEFAULT_MAX_DEGREE_ROTATION: u64 = 180; // 160 degrees is typical
const REFRESH_INTERVAL: u64 = 5000; // The period of each cycle

// Program metadata for `picotool info`.
// This isn't needed, but it's recommended to have these minimal entries.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"Chomper"),
    embassy_rp::binary_info::rp_program_description!(c"Controls the motion of a wire controlled chomping mouth"),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    PIO1_IRQ_0 => InterruptHandler<PIO1>;
});

pub struct ServoBuilder<'d, T: Instance, const SM: usize> {
    pwm: PioPwm<'d, T, SM>,
    period: Duration,
    min_pulse_width: Duration,
    max_pulse_width: Duration,
    max_degree_rotation: u64,
}

impl<'d, T: Instance, const SM: usize> ServoBuilder<'d, T, SM> {
    pub fn new(pwm: PioPwm<'d, T, SM>) -> Self {
        Self {
            pwm,
            period: Duration::from_micros(REFRESH_INTERVAL),
            min_pulse_width: Duration::from_micros(DEFAULT_MIN_PULSE_WIDTH),
            max_pulse_width: Duration::from_micros(DEFAULT_MAX_PULSE_WIDTH),
            max_degree_rotation: DEFAULT_MAX_DEGREE_ROTATION,
        }
    }

    pub fn set_period(mut self, duration: Duration) -> Self {
        self.period = duration;
        self
    }

    pub fn set_min_pulse_width(mut self, duration: Duration) -> Self {
        self.min_pulse_width = duration;
        self
    }

    pub fn set_max_pulse_width(mut self, duration: Duration) -> Self {
        self.max_pulse_width = duration;
        self
    }

    pub fn set_max_degree_rotation(mut self, degree: u64) -> Self {
        self.max_degree_rotation = degree;
        self
    }

    pub fn build(mut self) -> Servo<'d, T, SM> {
        self.pwm.set_period(self.period);
        Servo {
            pwm: self.pwm,
            min_pulse_width: self.min_pulse_width,
            max_pulse_width: self.max_pulse_width,
            max_degree_rotation: self.max_degree_rotation,
        }
    }
}

pub struct Servo<'d, T: Instance, const SM: usize> {
    pwm: PioPwm<'d, T, SM>,
    min_pulse_width: Duration,
    max_pulse_width: Duration,
    max_degree_rotation: u64,
}

impl<'d, T: Instance, const SM: usize> Servo<'d, T, SM> {
    pub fn start(&mut self) {
        self.pwm.start();
    }

    pub fn stop(&mut self) {
        self.pwm.stop();
    }

    pub fn write_time(&mut self, duration: Duration) {
        self.pwm.write(duration);
    }

    pub fn rotate(&mut self, degree: u64) {
        let degree_per_nano_second = (self.max_pulse_width.as_nanos() as u64 - self.min_pulse_width.as_nanos() as u64)
            / self.max_degree_rotation;
        let mut duration =
            Duration::from_nanos(degree * degree_per_nano_second + self.min_pulse_width.as_nanos() as u64);
        if self.max_pulse_width < duration {
            duration = self.max_pulse_width;
        }

        self.pwm.write(duration);
    }
}
#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_rp::init(Default::default());

    let fw = include_bytes!("../cyw43_firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43_firmware/43439A0_clm.bin");

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    let Pio { mut common, sm0, ..} = Pio::new(p.PIO1, Irqs);
    let servo_prg = PioPwmProgram::new(&mut common);
    let servo_pwm = PioPwm::new(&mut common, sm0, p.PIN_4, &servo_prg);
    let mut servo = ServoBuilder::new(servo_pwm)
        .set_max_degree_rotation(120)
        .set_min_pulse_width(Duration::from_micros(350))
        .set_max_pulse_width(Duration::from_micros(2600))
        .build();

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (_net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(cyw43_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    let switch = Input::new(p.PIN_28, Pull::Up);

    servo.start();
    info!("Setting servo to neutral position.");
    servo.rotate(90);

    info!("led on!");
    control.gpio_set(0, true).await;
    Timer::after(EmbassyDuration::from_millis(500)).await;

    loop {
        if switch.is_high() {
            info!("Switch is high");
            servo.rotate(120);
        } else {
            info!("Switch is low");
            servo.rotate(60);
        }
    }
}

#[embassy_executor::task]
async fn cyw43_task(runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>) -> ! {
    runner.run().await
}
