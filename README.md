# `f4`

> Board Support Crate for the NUCLEO-F411RE


## d7018e - special studies in embedded systems

##### Name : Johannes Sjölund
##### Mail : johsjl-1@student.ltu.se
##### Personal number: 8506298978
##### Nucleo 64 support crate

## Grading

3. Implement drivers for the Nucleo 64, stm32f401re/stm32f411re, similar to the f3/bluepill support crates. Most or all of the peripherals should be supported in some basic configuration
* GPIO
* UART over USB using the ST-Link v2
* DMA
* Timers with configurations
  * Microsecond counter
  * PWM generation
  * Encoder mode
* SPI
* I^2C
The peripheral pin configuration should match the standard Arduino headers.

4. The stm32f4 supports higher clock frequency than the default 16 MHz. Implement a simple way of setting it to the maximum of 84 MHz for the stm32f401re and 100 MHz for the stm32f411re, or any valid lower value (configured using the PLL).

5. Create a demo using the RTFM core and some external peripheral such as an IMU or Bluetooth device. The demo should show how to use the RTFM core for safe concurrency and demonstrate different peripherals such as GPIO, USART, DMA, SPI etc.

### Documentation
[F3 Rust support crate](https://github.com/japaric/f3)

[Blue-pill Rust support crate](https://github.com/japaric/blue-pill/)

[Nucleo 64 Manual](docs/Nucleo-64-User-manual.pdf)

[STM32F411 Datasheet](docs/STM32F411_Datasheet.pdf)

[STM32F411 Reference Manual](docs/STM32F411_Reference_Manual.pdf)

![Nucleo F411RE pins](docs/Nucleo_f411re.png)

![Nucleo F411RE morpho pins](docs/Nucleo_f411re_morpho.png)

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
