# PiStorm32-lite-gateware
## Gateware for the PiStorm32-lite card

This repository holds the Efinix FPGA project and sources.

## Binaries

Readymade PiStorm32-lite FPGA bitstreams can be found in the releases.
Also Linux Pistorm and EMU68 will automatically bundle the latest stable release version.


## Developing FPGA Firmware

To build the project a Efinix Efinity toolchain is needed.
The toolchain is not freely downloadable. To gain access to the download area a valid key file is needed.
The cheapest way to get hold of a key file is by purchasing a Efinix Xyloni Evaluation Kit.

You only need the Efinity toolchain if you intend to develop for the PiStorm32-lite FPGA.

[Link to Efinix](https://www.efinixinc.com/)

[Purchase link to the Xyloni Evaluation Kit](https://www.digikey.de/short/j9dqp75h)

For JTAG (if needed) a standard FT232H/FT2232H dongle is sufficient. Refer to the Efinity Documentation on how to use these dongles in the toolchain.

## Loading the FPGA

The PiStorm32-lite Linux and EMU68 Baremetal implementation automatically load the FPGA on startup.

Loading of the FPGA bitstream is accomplished by a SPI bitbang (SPI Passive x1) interface on the PiStorm32 card.
Initiating the download is achieved by setting Raspberry GPIO6 and GPIO7 (CRESET1 and CRESET2) to '0' and wait for approx. 1mS.
This produces a negative edge on the FPGA CRESET pin. Setting GPIO6 and GPIO7 to '1' again starts the FPGA config mechanism.
Then clocking out the bitstream to the FPGA.

Refer to the Efinix Trion documentation for details.

![image](https://user-images.githubusercontent.com/16537586/209782101-ad3c88c2-4121-4f49-8186-86e1db99953d.png)

![image](https://user-images.githubusercontent.com/16537586/209780479-55bac8d4-ce67-4cec-9b40-1d5dfeee4d8e.png)
```
#define PIN_CRESET1 6
#define PIN_CRESET2 7
#define PIN_TESTN 17
#define PIN_CCK 22
#define PIN_SS 24
#define PIN_CBUS0 14
#define PIN_CBUS1 15
#define PIN_CBUS2 18
#define PIN_CDI0 10
#define PIN_CDI1 25
#define PIN_CDI2 9
#define PIN_CDI3 8
#define PIN_CDI4 11
#define PIN_CDI5 1
#define PIN_CDI6 16
#define PIN_CDI7 13
```
