## About

This is a interactive shell test for SX1302 LoRa radio.

If you have other hardware (boards, Semtech based LoRa module), you can adapt
the configuration to your needs by copying an adapted version of
`drivers/sx1302/include/sx1302_params.h` file to your application directory.

## Wiring

Using the SX1302 CoreCell Gateway Interface Board V1a, and a Nucleo-64:

| Nucleo         | CN1 |
| -------------- | --- |
| GND            | 39  |
| 5V             | 2   |
| POWER EN (D8)  | 12  |
| RESET    (D9)  | 16  |
| CS       (D10) | 24  |
| MOSI     (D11) | 19  |
| MISO     (D12) | 21  |
| CLK      (D13) | 23  |

with CN1 being the 40 pins connector, starting with 1 in top left:  
1 3 5 ...  
2 4 6 ...  

## Testing

This test application provides low level shell commands to interact with the SX1302 modules.

Once the board is flashed and you are connected via serial to the shell, use the help command to display the available commands:
```
> help
Command              Description
---------------------------------------
setup                Initialize LoRa modulation settings
send                 Send raw payload string
listen               Start raw payload listener
reset                Reset the sx1302 device
eui                  Get the unique identifier
status               Get the TX status
test_reg             Test writing and reading from registers
syncword             Get/Set the syncword
channel              Get/Set channel frequency (in Hz)
rx_timeout           Set the RX timeout
idle                 Stop the RX mode
reboot               Reboot the node
version              Prints current RIOT_VERSION
pm                   interact with layered PM subsystem
ps                   Prints information about running threads.
random_init          initializes the PRNG
random_get           returns 32 bit of pseudo randomness
```

Once the board is booted, use `setup` to configure the basic LoRa settings:
* Bandwidth: 125kHz, 250kHz or 500kHz
* Spreading factor: between 5 and 12
* Code rate: between 5 and 8

Example:
```
> setup 125 7 5
setup: setting 125KHz bandwidth
[Info] setup: configuration set with success
```

The `channel` command allows to change/query the RF frequency channel.
The default is 868.3MHz for Europe. The frequency is given/returned in Hz.

Example:
```
> channel set 868300000
New channel set
> channel get
Channel: 868300000
```

Use the `send` and `listen` commands in order to exchange messages between several modules.
You need first to ensure that all modules are configured the same: use `setup` and
`channel` commands to configure them correctly.

Assuming you have 2 modules, one listening and one sending messages, do the following:
* On listening module:
```
> setup 125 7 5
setup: setting 125KHz bandwidth
[Info] setup: configuration set with success
> channel set 868300000
New channel set
> listen
Listen mode set
```
* On sending module:
```
> setup 125 7 5
setup: setting 125KHz bandwidth
[Info] setup: configuration set with success
> channel set 868300000
New channel set
> send 1 This\ is\ RIOT!
```

## Development

There are many unimplemented features, you should get an error when running into one.

## Liscence

Most of the code come from [sx1302_hal](https://github.com/Lora-net/sx1302_hal), in libloragw,  
combined with the sx127x driver.

