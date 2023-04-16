//! This example circulary lights the LEDs on the board.
#![no_main]
#![no_std]

// Temporary disable unused warnings
#[allow(dead_code)]


///////////////////////////////////////////
////////       START IMPORTS     //////////
///////////////////////////////////////////
use panic_halt as _;
use stm32f407g_disc as board;
use crate::board::{
    hal::stm32,
    hal::{delay::Delay, prelude::*},
    led::{LedColor, Leds},
};
use cortex_m::peripheral::Peripherals;
use cortex_m_rt::entry;
///////////////////////////////////////////
////////       END IMPORTS       //////////
///////////////////////////////////////////


///////////////////////////////////////////
//////   START MEMORY ADRESSES     ////////
///////////////////////////////////////////

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
///////////////////////////////////////////
//////   END MEMORY ADRESSES    //////////
///////////////////////////////////////////


///////////////////////////////////////////
//////    START UNSAFE FUNCTIONS   ////////
///////////////////////////////////////////

// Write to a register
fn write_unsafe(gpiox: u32, operation: u32) {
    unsafe{
        core::ptr::write_volatile(gpiox as *mut u32, operation);
        }
}

// Read from a register
fn read_unsafe(gpiox: u32) -> u32 {
    unsafe{
        return core::ptr::read_volatile(gpiox as *const u32);
        }
} 

// Toggle a user chosen bit on a GPIO 
fn toggle_bit_unsafe(gpiox: u32, bit: u8) {
    write_unsafe(gpiox, read_unsafe(gpiox) ^ (1 << bit));
}

// Set a user chosen bit on a GPIO to 1
fn set_bit_high_unsafe(gpiox: u32, bit: u8) {  
    write_unsafe(gpiox, read_unsafe(gpiox) | (1 << bit));
}

// Set a user chosen bit on a GPIO to 0
fn set_bit_low_unsafe(gpiox: u32, bit: u8) {
    write_unsafe(gpiox, read_unsafe(gpiox) & !(1 << bit));
}

// Set a specfific GPIO pin to output mode
fn set_gpio_output_unsafe(gpiox_moder: u32, pin: u8) {
   set_bit_high_unsafe(gpiox_moder, pin * 2);
}

///////////////////////////////////////////
///////    END UNSAFE FUNCTIONS   /////////
///////////////////////////////////////////



///////////////////////////////////////////
//////    START DISPLAY FUNCTIONS   ///////
///////////////////////////////////////////


///////////////////////////////////////////
//////    END DISPLAY FUNCTIONS    ////////
///////////////////////////////////////////


///////////////////////////////////////////
//////    START MAIN FUNCTION     /////////
///////////////////////////////////////////
#[entry]
fn main() -> ! {
    if let (Some(p), Some(cp)) = (stm32::Peripherals::take(), Peripherals::take()) {

        // Enable GPIOD peripheral clock
        let gpiod = p.GPIOD.split();

        // Constrain clock registers
        let rcc = p.RCC.constrain();

        // Configure clock to 168 MHz (i.e. the maximum) and freeze it
        let clocks = rcc.cfgr.sysclk(168.mhz()).freeze();

        // Get delay provider
        let mut delay = Delay::new(cp.SYST, clocks);
    
        // GPIO pin setup

        // Über die "offizielle" crate 
        // let top = gpiod.pd13.into_push_pull_output();
        // let left = gpiod.pd12.into_push_pull_output();
        // let right = gpiod.pd14.into_push_pull_output();
        // let bottom = gpiod.pd15.into_push_pull_output();

        // Setze PD13 als Ausgang
        set_gpio_output_unsafe(GPIOD_MODER, 13);

        loop {
            // Schalte PD13 ein
            toggle_bit_unsafe(GPIOD_ODR, 13);

            // Warte 500ms
            delay.delay_ms(500_u16);
        
        }
    }

    // If we get here, something went terribly wrong
    loop {
        continue;
    }
}
///////////////////////////////////////////
//////    END  MAIN FUNCTION     //////////
///////////////////////////////////////////