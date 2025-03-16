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
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::pio_programs::pwm::{PioPwm, PioPwmProgram};
use embassy_time::{Duration as EmbassyDuration, Timer};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use pico_chomp_devices::ServoBuilder;

// Program metadata for `picotool info`.
// This isn't needed, but it's recommended to have these minimal entries.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"pico_chomp_host"),
    embassy_rp::binary_info::rp_program_description!(c"Controls the motion of a wire controlled chomping mouth"),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    PIO1_IRQ_0 => InterruptHandler<PIO1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_rp::init(Default::default());

    let fw = include_bytes!("../../cyw43_firmware/43439A0.bin");
    let clm = include_bytes!("../../cyw43_firmware/43439A0_clm.bin");

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
        .set_max_degree_rotation(180)
        .set_min_pulse_width(Duration::from_micros(500))
        .set_max_pulse_width(Duration::from_micros(2500))
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
            servo.rotate(120);
        } else {
            servo.rotate(60);
        }
    }
}

#[embassy_executor::task]
async fn cyw43_task(runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>) -> ! {
    runner.run().await
}
