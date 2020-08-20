# SX1280 Driver

Driver of LoRa device [SX1280](https://www.semtech.com/products/wireless-rf/24-ghz-transceivers/sx1280) for [RIOTOS](https://github.com/RIOT-OS/RIOT).

### Usage

The code is based on [SX1280Lib](https://os.mbed.com/teams/Semtech/code/SX1280Lib/) by Semtech and adapted to RIOT using the sx127x driver code.  
You can find an example in `tests/driver_sx1280` to send messages between two boards. 

The module and the driver are designed to work with Multitech standard channels:

| Channel | Frequency (Hz) | Bandwidth (Hz) | Spread Factor |
|---------|----------------|----------------|---------------|
| 1       | 2403000000     | 812000         | 12            |
| 2       | 2479000000     | 812000         | 12            |
| 3       | 2425000000     | 812000         | 12            |


### Installation

```bash
cd tests/driver_sx1280
make BOARD=<your board> all flash term
```

### Wiring

For the [Lambda80](https://fr.farnell.com/rf-solutions/lambda80-24s/transceiver-2mbps-2-5ghz/dp/2988571) board, you need at least 8 wires, or 9 with the RESET pin:  

| Nucleo | Lambda80   | Function                     |
|--------|------------|------------------------------|
| GND    | GND    (2) | 0V                           |
| 3V3    | Vcc    (3) | +3.3V                        |
| D8     | DIO0   (6) | Busy pin                     |
| D9     | DIO1   (7) | IRQ (TX/RX done for example) |
| D10    | NSS   (16) | SPI CS                       |
| D11    | SDI   (15) | SPI MOSI                     |
| D12    | SDO   (14) | SPI MISO                     |
| D13    | SCLK  (13) | SPI CLK                      |
| D7     | RESET (12) | Optional RESET               |

This is the default pinout but you can customize it by providing another `sx1280_params_t` during `sx1280_init`.

### Testing

This test application provides low level shell commands to interact with the
SX1280 modules.

Once the board is flashed and you are connected via serial to the shell, use the `help`
command to display the available commands:
```
> help
Command              Description
---------------------------------------
setup                Initialize LoRa modulation settings
random               Get random number from sx1280
rx_timeout           Set the RX timeout
channel              Get/Set channel frequency (in Hz)
register             Get/Set value(s) of registers of sx1280
send                 Send raw payload string
listen               Start raw payload listener
reset                Reset the sx1280 device
status               Get the sx1280 status
preamble             Get/Set the preamble length
invert_iq            Get/Set IQ swapping
reboot               Reboot the node
version              Prints current RIOT_VERSION
pm                   interact with layered PM subsystem
ps                   Prints information about running threads.
```

Once the board is booted, use `setup` to configure the basic LoRa settings:
* Bandwidth: 200kHz, 400kHz, 800kHz or 1600kHz
* Spreading factor: between 7 and 12
* Code rate: between 5 and 8

Example:
```
> setup 400 7 5 li
setup: setting 400KHz bandwidth
[Info] setup: configuration set with success
```

The `channel` command allows to change/query the RF frequency channel.
The default is 868MHz/2403MHz for Europe, change to 915MHz for America. The frequency
is given/returned in Hz.

Example:
```
> channel set 2403000000
New channel set
> channel get
Channel: 2403000000
```

Use the `send` and `listen` commands in order to exchange messages between several modules.
You need first to ensure that all modules are configured the same: use `setup` and
`channel` commands to configure them correctly.

Assuming you have 2 modules, one listening and one sending messages, do the following:
* On listening module:
```
> setup 800 12 5 li
setup: setting 800KHz bandwidth
[Info] setup: configuration set with success
> channel set 2403000000
New channel set
> listen
Listen mode set
```
* On sending module:
```
> setup 800 12 5 li
setup: setting 800KHz bandwidth
[Info] setup: configuration set with success
> channel set 2403000000
New channel set
> send This\ is\ RIOT!
```

On the listening module, the message is captured:
```
{Payload: "This is RIOT!" (14 bytes), RSSI: 190, SNR: 13, TOA: 4480 us}
```


This is the list of tested boards:
- nucleo-f446re
- nucleo-f446ze
- nucleo-f411re


### Missing features

This is the list of the task required to finish the driver:

- Set the sx1280 to sleep when not in use
- Use the right settings for power consumption, like DCDC or LDO and RC or XOSC
- Figure out the Time-On-Air according to the spec
- Finish other utility functions for LoRa
- Configure the driver for more MODEM type like GFSK, BLE, FLRC or Ranging.

### License

The files `drivers/include/sx1280.h` and `drivers/sx1280/sx1280_getset.c` are largely imported from [SX1280Lib](https://os.mbed.com/teams/Semtech/code/SX1280Lib/),  
licensed under [Apache license](https://os.mbed.com/handbook/Apache-Licence)

All the other files are inspired by the sx127x driver written by Inria Chile.