# CAN configuration tips

This document summarizes the key steps for configuring CAN on a STM32CubeIDE project.

## Hardware

Please first unsure you have access to a STM32 MCU which supports CAN (only Classical CAN is required for this driver), with a CAN Controller. This controller may be included in the STM32-based board (like in the DM-MC02 board used for the development of this driver) 

**Clock config**

What matters here is to what value **To FDCAN** or **To CAN** frequency is set. It can be set to any value, the simplest setup being to use the HSE clock directly, but not mandatory.

Please take note of this value. e.g. $f_{To FDCAN} = 80MHz$

**Get FDCAN timing parameters**
Knowing the above base frequency, one should find timing parameters for the desired CAN bitrate.

The following website can be used to find such parameters : [Kvaser CAN Bus Bit Timing Calculator](https://kvaser.com/support/calculators/bit-timing-calculator/)

One may follow a related guide for detailled step by step instructions.

As an example, The following values were used for the project :

- Clock frequency, e.g. 80 000 (kHz)
- Tolerance, e.g. 4678 (Ppm)
- Node delay, e.g. 65 (ns) (according to SIT1042 CAN controller's datasheet, used on the DM-MC02 board)
- Nominal bitrate, e.g. 500 000 (bit/s)
- Data bitrate, e.g. 2 000 000 (bit/s) (can be any value for Classical CAN)

To get the following values :
| (Nom 500k) | Tseg1 | Tseg2 | SJW | PRESC | (Data 2M) | Tseg1 | Tseg2 | SJW | PRESC |
|------------|-------|-------|-----|-------|-----------|-------|-------|-----|-------|
|            |    69 |    10 |  10 |     2 |           |    16 |    16 |  15 |     5 |


**Required pinout**
Depending on which port to use, the user can select the corresponding pins.

For example, to use CAN1 port
    * Connectivity/FDCAN1
        * Activated = Check
    * Verify used pins (CAN TX/ CAN RX) according to datasheet ! For the DMMC-02 board for CAN1, use PD1 and PD2 pins. One may move the pins with Ctrl+Click and dragging it.

**FDCAN config**

- Activate the desired FDCAN port as described above
- In `Configuration` tab:

| Param | Value | | Param | Value | | Param | Value |
| ----- | ----- | - | ----- | ----- | - | ----- | ----- |
| Frame Format | Classic Mode | | Mode | Normal Mode | | Auto Retransmission | Enable |
| Message Ram Offset | 0 | | Std Filters Nbr | at least 1 | | Ext Filters Nbr | 0 |
| Rx Fifo0 Elmt Nbr | 1 | | Rx Fifo0 Elmt Size | 8 bytes | | Rx Fifo1 Elmt Nbr | 0 |
| Rx Buffers Nbr | at least 1 | | Rx Buffers Size | 8 bytes | | Tx Buffers Nbr | at least 1 |
| Tx Fifo Queue Elmts Nbr | 1 | | Tx Fifo Queue Mode | FIFO mode | | Tx Elmt Size | 8 bytes |

Please set the timing configurations according to the values obtained from the previous section as well, and please note that `Nominal Baud Rate` should now have the target bitrate value. 

The other parameters can be set to default values, and changed if need be.

**Wiring**

The following points are of particular importance:
- The Gnd should be common to all devices of the CAN network
- The CAN_H and CAN_L lines should be connected to all devices of the CAN network, and the wires twisted together for as many chunks as possible to reduce noise.
- The CAN_H and CAN_L lines should be terminated at both ends of the CAN network.
    * This is done by connecting a 120 Ohm resistor between CAN_H and CAN_L at both ends of the CAN network. 
    * Only two such resistors are needed, no need to add one for each added device
    * To check for correct termination, one may use a multimeter to measure the resistance between CAN_H and CAN_L. The resistance should be around 60 Ohm.
- Please note that the CAN controllers on the DM-MC02 board are only provided power if either
    * USB-C power is provided to the board, or
    * 24V power is provided to the board.
Thus additionnaly the USB-C port should be used to power the board (perhaps alongside its main power supply method)

## Testing and debugging

In case the setup does not work right away, some debugging should be done.

**Start with testing Loopback mode**
The following guide offers a testing code for the setup: [STM32 FDCAN in Loopback Mode](https://controllerstech.com/stm32-fdcan-in-loopback-mode/)

If this works, then the issue can be
* Synchronization issues (check timings, signal frequency)
* Wiring issues
* Power issues (use USB or 24V to provide power to the CAN controller, DM-MC02 specific)

Using an oscilloscode at this point, with External Loopback Mode can be of great help.