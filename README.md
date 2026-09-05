# How to Fix & Add an Auto-Range Option to Your Boonton 92 Series RF Millivolt Meter

## Background

Some years ago I came across a 92BD RF Millivolt Meter in reasonably good condition. After a while it started showing problems that indicated the chopper tube was nearing end of life. Since these tubes are no longer available, the meter was effectively unusable.

I found a possible solution: replace the entire front end with a couple of low-drift op-amps and their associated gain-selection circuitry. This design was created by Jacques Audet, built on perfboard with a connector suitable for the 92 series backplane. I started laying out a PCB for it, but never actually built one.

## The SSR Idea

As part of another project, I became interested in solid-state relays (SSRs) that use MOSFETs, with the drive circuit electrically isolated from the signal. These devices can be driven with just a few mA, the MOSFET turns on within a microsecond, and it has very low Rds(on) resistance.

That's when the idea came up: replace the chopper tube with two SSRs (both housed in a single physical package). My original MCU-based auto-ranging extension board was redesigned into a new PCB that would handle both:

- The (already working) auto-ranging function
- The drive circuit for the SSRs

## Signal Path & Wiring

Since signal levels are quite low, and noise had to be minimized, I built an additional small PCB that mounts directly behind the signal input connector, with short leads to the SSR PCB and a single lead to the original analog input on the backplane.

A set of twisted control wires runs from the SSR PCB to a few free pins on backplane connector **J103**, which was originally intended for the auto-ranging board. On this connector, one additional connection is also needed: a jumper wire supplying +5V DC to the auto-ranging connector J103.

## System Overview

The current design consists of two parts, both controlled by the MCU:

1. **Auto-ranging circuit**
2. **SSR control circuit**

For the MCU I used an **ESP32** (with both Bluetooth and WiFi disabled). The choice was based on prior projects and the large number of available I/O pins.

### Handling Negative Control Voltages

The 92 series meter uses negative control voltages (up to −15V DC), while the MCU's pins are only 3.3V-tolerant. To bridge this gap, the design uses **9 optocouplers**:

- 8 for range selection
- 1 for auto-range mode

The analog voltage only needs to be measured after being reduced to below 3V, which is done with a small resistor divider network.

### Chopper Timing Emulation

The chopper section uses the original chopper tube connector, plugged directly into the PCB. Its voltage level is reduced to below 3.3V DC before being fed to the MCU as an input.

The MCU adjusts the signal output to the SSRs to match a contact timing similar to that of the original chopper tube, ensuring a proper **break-before-make** SPDT contact behavior is maintained.

Driving the opto input side of the SSR is simple — it only requires a single resistor to limit the LED current in the optocoupler.

## PCB Revision Notes

**V2.2 PCB** — Works well, but required some tweaks:

- Adjusted physical size
- Repositioned connectors
- Changed some GPIO port assignments
- Added a JTAG port
- Removed the serial port
- Added a few resistors

> **Note:** The small PCB embedded within the larger board needs to be separated and used behind the instrument's input connector, mounted on two short standoffs using the existing holes from the chopper tube bracket.

![20230806_115556](https://github.com/ph-wheels/Boonton_92BD/assets/10708995/48fdfadd-2625-4f61-920f-065d6dd4d30e)

Build Notes & Firmware Update

This picture is from the V2.2 PCB — V2.7 has yet to be built and should give a much cleaner result. A few additional pictures (in the main folder) provide more detail on the build and required modifications.

The small SSR PCB only holds the original capacitors that came from the chopper tube socket. Note that the black stripe must be connected to the ground pads of C1 and C3. The SSR is then connected via short coax cables to the input connector and to the main analog PCB.

The SSR PCB was cut down primarily to reduce PCB fabrication cost — the main PCB has so much unused real estate that separating it out was the obvious choice.

## Calibration

Calibration of instrument it's self after de SSR mod can be done with the Boonton instrument calibration guidelines, once this is completed and no abnormal behaviour was found it's time to adjust the values for al and ah used by the auto range circuit.

This can easily be done by selecting the 1000mV range, apply a input signal of 1030 mV and read the ADC value with the 'a' command of the menu, the amount displayed will be your ah value.

Then repeat the same procedure but apply a input signal of 413 mV and read the ADC with the 'a' command of the menu, the amount displayed will be your al value.

Next type al xxx <cr> (where xxx is the value obtained in the last step) and ah yyy <cr> (where yyy is the value obtained in the first step)

Than check if they have been entered correctly by typing 'a' <cr>, that's all !!

Firmware Redesign (Aug 30, 2026)

A completely redesigned firmware is now available. Early tests look promising, showing:

Improved signal stability
Cleaner code
Solid task isolation between auto-range, SSR driver, and the menu system with parameter storage

This was made possible largely thanks to Espressif's ESP32 FreeRTOS implementation — a real game-changer for the codebase.

Have fun building!
