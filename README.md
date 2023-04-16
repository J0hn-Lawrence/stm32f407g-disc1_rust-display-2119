# Read here first

Display driver for a SSD2119 display controller using Rust on the STM32F407G-DISC1 board.

### Building the examples:
----------------------

The display example can be built with `cargo`:

    $ cargo build --release --example display

### Flashing the examples:
----------------------

- Plug in the board via USB. The board should show up as a USB mass storage device.
- Drag and drop the binary file from the `target` directory to mounted directory of the board.
- The binary should be at `target/thumbv7em-none-eabihf/release/examples/display`.
- The round multi LED near the USB connector should start blinking. When it stops blinking, the binary has been flashed. 
- Unplug the board and plug it back in. The flashed binary should now be running.

### License
-------
The same license as the original repository ( https://github.com/stm32-rs/stm32f407g-disc ):

[0-clause BSD license](LICENSE-0BSD.txt).
