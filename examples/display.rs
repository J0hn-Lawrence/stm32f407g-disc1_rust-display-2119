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
fn toggle_bit (gpiox: u32, bit: u8) {
    write_unsafe(gpiox, read_unsafe(gpiox) ^ (1 << bit));
}

// Set a user chosen bit on a GPIO to 1
fn set_bit_high(gpiox: u32, bit: u8) {  
    write_unsafe(gpiox, read_unsafe(gpiox) | (1 << bit));
}

// Set a user chosen bit on a GPIO to 0
fn set_bit_low(gpiox: u32, bit: u8) {
    write_unsafe(gpiox, read_unsafe(gpiox) & !(1 << bit));
}

// Set a specfific GPIO pin to output mode
fn set_gpio_output(gpiox_moder: u32, pin: u8) {
   set_bit_high(gpiox_moder, pin * 2);
}

// Set a specfific GPIO pin to input mode
fn set_gpio_input(gpiox_moder: u32, pin: u8) {
    set_bit_high(gpiox_moder, pin);
}

// Use logical AND to set a register
fn and_register(gpiox: u32, operation: u32) {
    write_unsafe(gpiox, read_unsafe(gpiox) & operation);
}

// Use logical OR to set a register
fn or_register(gpiox: u32, operation: u32) {
    write_unsafe(gpiox, read_unsafe(gpiox) | operation);
}

// Toggle register
fn toggle_register(gpiox: u32, operation: u32) {
    write_unsafe(gpiox, read_unsafe(gpiox) ^ operation);
}

///////////////////////////////////////////
///////    END UNSAFE FUNCTIONS   /////////
///////////////////////////////////////////



///////////////////////////////////////////
//////    START DISPLAY FUNCTIONS   ///////
///////////////////////////////////////////


// void  LCD_Output16BitWord(uint16_t data)
// {
// 	GPIOD->ODR &= 0x38FC;
// 	GPIOD->ODR |= ((data & 0xE000)>>5) | ((data & 0x0003)<<14) | ((data & 0x000C)>>2);	
// 	GPIOE->ODR &= 0x007F;
// 	GPIOE->ODR |= (data & 0x1FF0)<<3;
// }

// Ouput 16 bit word to the display
fn lcd_output_16_bit_word(data: u16) {
    and_register(GPIOD_ODR, 0x38FC);
    or_register(GPIOD_ODR, (((data & 0xE000)>>5) | ((data & 0x0003)<<14) | ((data & 0x000C)>>2)).into());
    and_register(GPIOE_ODR, 0x007F);
    or_register(GPIOE_ODR, ((data & 0x1FF0)<<3).into());    // NOTE: .into() is done during compile time to convert u16 to u32
}

// void LCD_WriteData (uint16_t data){
// 	GPIOD->ODR |= 0x10;		//1<<4;
// 	GPIOE->ODR |= 0x08;		//1<<3;
// 	GPIOD->ODR &= 0xFF5F; 	//(D5 & D7 = 0)
// 	LCD_Output16BitWord(data);
// 	GPIOD->ODR |= 0x20;		//1<<5;
// }

// Write data to the display
fn lcd_write_data (data:u16){
    or_register(GPIOD_ODR, 0x10); //1<<4;
    or_register(GPIOE_ODR, 0x08); //1<<3;
    and_register(GPIOD_ODR, 0xFF5F); //(D5 & D7 = 0)
    lcd_output_16_bit_word(data);
    or_register(GPIOD_ODR, 0x20); //1<<5;
}

// void LCD_WriteCommand (uint16_t cmd){
// 	GPIOD->ODR |= 0x10;		//1<<4;
// 	GPIOE->ODR &= 0xFFF7;	//~(1<<3);
// 	GPIOD->ODR &= 0xFF5F; 	//(D5 & D7 = 0)
// 	LCD_Output16BitWord(cmd);
// 	GPIOD->ODR |= 0x20;		//1<<5;
// }

// Write command to the display
fn lcd_write_command (cmd:u16){
    or_register(GPIOD_ODR, 0x10); //1<<4;
    and_register(GPIOE_ODR, 0xFFF7); //~(1<<3);
    and_register(GPIOD_ODR, 0xFF5F); //(D5 & D7 = 0)
    lcd_output_16_bit_word(cmd);
    or_register(GPIOD_ODR, 0x20); //1<<5;
}

// Initialize the GPIO pins for the display
fn lcd_pin_init (delay: &mut Delay){
    // Set the display ports to output
    // Control lines
    // WR:
    set_bit_high(GPIOD_ODR, 10);
    // CS:
    set_bit_high(GPIOD_ODR, 14);
    // DC:
    set_bit_high(GPIOE_ODR, 6);


    //Set the display ports to input
    //RD:
    set_bit_high(GPIOD_MODER, 8);

    //Port D12 auf ausgang schalten
    set_gpio_output(GPIOD_MODER, 12);

    // data lines
    set_gpio_output(GPIOD_MODER, 0);
    set_gpio_output(GPIOD_MODER, 1);
    set_gpio_output(GPIOD_MODER, 8);
    set_gpio_output(GPIOD_MODER, 9);
    set_gpio_output(GPIOD_MODER, 10);
    set_gpio_output(GPIOD_MODER, 14);
    set_gpio_output(GPIOD_MODER, 15);

    set_gpio_output(GPIOE_MODER, 7);
    set_gpio_output(GPIOE_MODER, 8);
    set_gpio_output(GPIOE_MODER, 9);
    set_gpio_output(GPIOE_MODER, 10);
    set_gpio_output(GPIOE_MODER, 11);
    set_gpio_output(GPIOE_MODER, 12);
    set_gpio_output(GPIOE_MODER, 13);
    set_gpio_output(GPIOE_MODER, 14);
    set_gpio_output(GPIOE_MODER, 15);


    set_gpio_output(GPIOD_MODER, 3);

    set_bit_low(GPIOD_ODR, 3);
    delay.delay_ms(5_u16);
    set_bit_high(GPIOD_ODR, 3)
}

// Initialize the display
fn lcd_init(delay: &mut Delay) {
    lcd_write_reg(0x0010, 0x0001); // Enter sleep mode
    lcd_write_reg(0x001E, 0x00B2); // Set initial power parameters.
    lcd_write_reg(0x0028, 0x0006); // Set initial power parameters.
    lcd_write_reg(0x0000, 0x0001); // Start the oscillator.
    lcd_write_reg(0x0001, 0x72EF); // Set pixel format and basic display orientation
    lcd_write_reg(0x0002, 0x0600);
    lcd_write_reg(0x0010, 0x0000); // Exit sleep mode.
    delay.delay_ms(30_u16);       //30ms warten // weniger geht meist auch
    lcd_write_reg(0x0011, 0x6870); // Configure pixel color format and MCU interface parameters.
    lcd_write_reg(0x0012, 0x0999); // Set analog parameters 
    lcd_write_reg(0x0026, 0x3800);
    lcd_write_reg(0x0007, 0x0033); // Enable the display
    lcd_write_reg(0x000C, 0x0005); // Set VCIX2 voltage to 6.1V.
    lcd_write_reg(0x000D, 0x000A); // Configure Vlcd63 and VCOMl 
    lcd_write_reg(0x000E, 0x2E00);
    lcd_write_reg(0x0044, (240-1) << 8); // Set the display size and ensure that the GRAM window is set to allow access to the full display buffer.
    lcd_write_reg(0x0045, 0x0000);
    lcd_write_reg(0x0046, 320-1);
    lcd_write_reg(0x004E, 0x0000); // Set cursor to 0,0
    lcd_write_reg(0x004F, 0x0000);
}

// Write a command to the display
fn lcd_write_reg(cmd:u16, data:u16){
    lcd_write_command(cmd);
    lcd_write_data(data);
}

// Set the cursor to a specific x,y position
fn lcd_set_cursor(x:u16, y:u16){
    lcd_write_reg(0x004E, x);
    lcd_write_reg(0x004F, y);
}

// Set the cursor to a specific x position
fn lcd_set_cursor_x(x:u16){
    lcd_write_reg(0x004E, x);
}

// Set the cursor to a specific y position
fn lcd_set_cursor_y(y:u16){
    lcd_write_reg(0x004E, y);
}

// Draw a pixel at the current cursor position
fn lcd_draw_pixel(color:u16){
    lcd_write_reg(0x0022, color);
}

fn clear_display (color: u16){
    and_register(GPIOE_ODR, 0x77);      //FFF7 & 7F = 77 (E3 und alle 16bit wording von E auf Null setzen)
    and_register(GPIOD_ODR, 0x385C);    //D5 und D7 und 16bit wording D = 0

    or_register(GPIOD_ODR, 0x8000);
    or_register(GPIOE_ODR, 0x100);      //(0x22 & 0x1FF0)<<3;
    or_register(GPIOD_ODR, 0x20);

    //farbe
    or_register(GPIOE_ODR, 0x8);
    and_register(GPIOD_ODR, 0x385C);    //D5 und D7 und 16bit wording D = 0
    or_register(GPIOD_ODR, (((color & 0xE000)>>5) | ((color & 0x0003)<<14) | ((color & 0x000C)>>2)).into());
    and_register(GPIOE_ODR, 0x7F);
    or_register(GPIOE_ODR, ((color & 0x1FF0)<<3).into());  //+0x8;+E3

    for _pixel in 1..=76800{
        and_register(GPIOD_ODR, 0xFFDF);    //~(1<<5)
        or_register(GPIOD_ODR, 0x20);       //1<<5
    }
}
///////////////////////////////////////////
//////    END DISPLAY FUNCTIONS    ////////
///////////////////////////////////////////


///////////////////////////////////////////
//////    START MAIN FUNCTION     /////////
///////////////////////////////////////////
#[entry]
fn main() -> ! {
    if let (Some(p), Some(cp)) = (stm32::Peripherals::take(), Peripherals::take()) {
        // BASIC GPIO SETUP
        // Enable GPIOD, GPIOE, GPIOB peripheral clock
        let gpiod = p.GPIOD.split();
        let gpioe = p.GPIOE.split();
        let gpiob = p.GPIOB.split();

        // CLOCK SETUP
        // Constrain clock registers
        let rcc = p.RCC.constrain();
        // Configure clock to 168 MHz (i.e. the maximum) and freeze it
        let clocks = rcc.cfgr.sysclk(168.mhz()).freeze();
        // Get delay provider
        let mut delay = Delay::new(cp.SYST, clocks);

        ////  DISPLAY SETUP    /////
        // Switch on the backlight of the display
        set_gpio_output(GPIOD_MODER, 13);
        // Call the GPIO pin init function for the display pins
        lcd_pin_init(&mut delay);
        // Call the display init function
        lcd_init(&mut delay);

        ////   WRITE TEXT    /////
        // Set the cursor to the top left corner
        lcd_set_cursor(0, 0);
        // Write the text to the display
        // lcd_write_text("Hello World!", 0x0000, 0xFFFF);
    }

    // If we get here, something went terribly wrong
    loop {
        continue;
    }
}
///////////////////////////////////////////
//////    END  MAIN FUNCTION     //////////
///////////////////////////////////////////