//! This example circulary lights the LEDs on the board.
#![no_main]
#![no_std]

use panic_halt as _;

use stm32f407g_disc as board;

use crate::board::{
    hal::stm32,
    hal::{delay::Delay, prelude::*},
    led::{LedColor, Leds},
};

use cortex_m::peripheral::Peripherals;

use cortex_m_rt::entry;
// Set peripheral clock enable register adresses
const RCC_AHB1ENR: u32 = 0x40023800;

// Set the mode register adresses for GPIO
const GPIOA_MODER: u32 = 0x40020000;
const GPIOB_MODER: u32 = 0x40020400;
const GPIOC_MODER: u32 = 0x40020800;
const GPIOD_MODER: u32 = 0x40020C00;
const GPIOE_MODER: u32 = 0x40021000;
const GPIOF_MODER: u32 = 0x40021400;
const GPIOG_MODER: u32 = 0x40021800;
const GPIOH_MODER: u32 = 0x40021C00;
const GPIOI_MODER: u32 = 0x40022000;

// Set the output data register adresses for GPIO
const GPIOA_ODR: u32 = GPIOA_MODER + 0x14;
const GPIOB_ODR: u32 = GPIOB_MODER + 0x14;
const GPIOC_ODR: u32 = GPIOC_MODER + 0x14;
const GPIOD_ODR: u32 = GPIOD_MODER + 0x14;
const GPIOE_ODR: u32 = GPIOE_MODER + 0x14;
const GPIOF_ODR: u32 = GPIOF_MODER + 0x14;
const GPIOG_ODR: u32 = GPIOG_MODER + 0x14;
const GPIOH_ODR: u32 = GPIOH_MODER + 0x14;
const GPIOI_ODR: u32 = GPIOI_MODER + 0x14;

#[entry]
fn main() -> ! {
    if let (Some(p), Some(cp)) = (stm32::Peripherals::take(), Peripherals::take()) {
        let gpiod = p.GPIOD.split();

        // Initialize on-board LEDs
        let mut leds = Leds::new(gpiod);

        // Constrain clock registers
        let rcc = p.RCC.constrain();

        // Configure clock to 168 MHz (i.e. the maximum) and freeze it
        let clocks = rcc.cfgr.sysclk(168.mhz()).freeze();

        // Get delay provider
        let mut delay = Delay::new(cp.SYST, clocks);
    
    loop {

        // Schalte PD14 ein
        unsafe {
            core::ptr::write_volatile(GPIOD_ODR as *mut u32, 0x4000);
        }
        
        // Warte eine kurze Zeit
        delay.delay_ms(500_u16);

        // Schalte PD14 aus
        unsafe {
            core::ptr::write_volatile(GPIOD_ODR as *mut u32, 0);
        }

        // Warte eine kurze Zeit
        delay.delay_ms(500_u16);

    
    }
    }
    loop {
        continue;
    }

}
