# ExpressLRS

## Community

Please join at

 * [RCGroups Discussion](https://www.rcgroups.com/forums/showthread.php?3437865-ExpressLRS-DIY-LoRa-based-race-optimized-RC-link-system)
 * [Discord Chat](https://discord.gg/dS6ReFY)

## ESP32/ESP8285/STM32 based LoRa Radio Link

![Build Status](https://github.com/gretel/ExpressLRS/workflows/Build%20ExpressLRS/badge.svg)

ExpressLRS is an open source RC link for RC aircraft. It is based on **SX127x** hardware combined with an **ESP8285**, **ESP32** or **STM32** for RX and TX respectively.

ExpressLRS can be flashed into existing Frsky R9M hardware (RX and TX) or custom PCBs can be make to suit. It can run at 200 Hz, 100 Hz or 50 Hz depending on if you prefer range or low latency. This system does FHSS over a configurable number of channels. 
At 200 Hz it is the fastest 900/433 MHz RC link on the market with a Stick -> OpenTX > RF -> RX -> Serial Packet Latency of ~10ms or ~6.5ms on firmwares with **crsfshot** support. 

Due to the optimized packet structure only basic telemetry that gives uplink/downlink information is currently supported. 

TX and RX modules communicate via the standard CRSF serial protocol for easy use with Betaflight and openTX.

Compared to commerical systems ExpressLRS is also very affordable, a TX module can be built for $30 and recievers for $15-20.

### TTGO LoRa boards

Now supports using easily available TTGO LoRa boards as TX modules for much easier assembly! Build information below.

![TTGO LoRa](img/TTGO_BOARD.jpg)

<img src="img/R9M_and_ExpressLRS_modules.jpg" width="50%"><img src="img/module_inhousing.jpg" width="50%">
<img src="img/IMG_20181025_210516.jpg" width="50%"><img src="img/IMG_20181025_210535.jpg" width="50%">

### Building a TX Module

For the build you will need a TTGO LoRa board, with or without an OLED. These boards are readily available from ebay, aliexpress, and banggood. The only others parts required are some wire, 5 pin female header, and your favourite 5V regulator that can take the transmitters battery voltage range.

Note - The board I bought came with a female SMA pigtail. Check if your antenna is suitable.

 * [LILYGO TTGO LoRa](http://www.lilygo.cn/prod_view.aspx?TypeId=50003&Id=1134&FId=t3:50003:3)
 * [AliExpress TTGO LoRa (no OLED)](https://www.aliexpress.com/item/4000059700341.html)
 * [AliExpress TTGO LoRa (with OLED)](https://www.aliexpress.com/item/32840238513.html)

### Enclosure

STLs for printing your own enclosure are available in the [STL folder](https://github.com/AlessandroAU/ExpressLRS/tree/master-dev/STL).

<img src="img/ttgo_lora_wiring_diagram.png" width="50%"><img src="img/TTGO_BOARD_2.png" width="50%">

### Building a RX

- https://github.com/AlessandroAU/ExpressLRS/tree/master-dev/PCB/Mini_Rx_v0.1

### Legal Stuff
The use and operation of this type of device may require a license and some countries may forbid its use. It is entirely up to the end user to ensure compliance with local regulations. This is experimental software/hardware and there is no guarantee of stability or reliability.
