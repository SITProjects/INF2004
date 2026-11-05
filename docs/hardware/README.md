# Hardware

Board: Raspberry Pi Pico W (RP2040, CYW43439 radio) on a Robo Pico carrier.
The kernel runs `clk_sys` at 125 MHz.

The authoritative pin and peripheral list is `TEAM.md` sections 2 and 3.
Summary:

| Subsystem | Device | Pins | RP2040 block |
|---|---|---|---|
| obstacle | HC-SR04-style ultrasonic sensor | GP17 echo, GP16 trigger | SIO, GPIO edge interrupt, TIMER |
| obstacle | hobby servo (500–2500 µs at 50 Hz) | GP12 | PWM slice 6 A |
| motion | two DC motors on the Robo Pico H-bridge (M1, M2) | GP8/GP9, GP10/GP11 | PWM slices 4 and 5 at 20 kHz |
| motion | two quadrature wheel encoders (3.3 V supply) | GP13/GP14 left, GP2/GP3 right | PIO0 state machines 2 and 3 |
| line_follow | two reflective IR modules, digital DO outputs (LM393 comparator, set by potentiometer) | GP26, GP27 | SIO inputs, polled every 10 ms |
| barcode | third reflective IR module, digital DO output | GP28 | SIO input, GPIO edge interrupt, TIMER |
| terrain | GY-511 (LSM303DLHC) on Grove port 3 | GP4 SDA, GP5 SCL | I2C0 at 400 kHz |
| communication | on-board CYW43439 radio | GP23–GP25, GP29 | owned by the kernel; gSPI on PIO1 |

Kernel device configuration that the hardware depends on:

- `config/config_device.h`: `DEVCNF_USE_IIC` = 1 (I2C driver on),
  `DEVCNF_USE_ADC` = 1 (still built; no application uses it).
- `device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h`: I2C0 on GP4/GP5.
- `device/adc/sysdepend/rp2040/adc_cnf_sysdep.h`: no ADC pin is set up at
  boot, so GP26–GP28 stay digital for the IR modules.
