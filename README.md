# Davis ISS Weather Receiver for M5Stack StickS3

Receives wireless data from a Davis Instruments ISS (the wireless sensor
mast used by the Vantage Pro2 and Vantage Vue) using a CC1101 sub-1GHz
radio module, and shows temperature, humidity, wind, rain, solar/UV (if
your ISS has those sensors) and battery status on the StickS3's screen.

## Power note: the 3V3_L2 pin

CC1101 breakout boards run on **3.3V only** -- never wire them to 5V. The
StickS3's Hat2-Bus header conveniently exposes a `3V3_L2` pin (pin 13), so
you don't need a separate regulator. That said, `3V3_L2` is one of several
switchable power domains managed internally by the StickS3's M5PM1 power
chip, and its default on/off state isn't otherwise documented -- **measure
it with a multimeter after calling `M5.begin()`** before connecting the
CC1101. If it reads 0V, you'll need the
[M5PM1 library](https://github.com/m5stack/M5PM1) to enable that power
rail (see the StickS3's "Low-Power Configuration" docs for the relevant
API) before the radio will power up.

## Wiring (Hat2-Bus 2.54-16P header)

WIRING (StickS3 Hat2-Bus 2.54-16P header) -- this matches the official
Bruce firmware pinout for the M5StickS3 CC1101 (wiki.bruce.computer), so
the same physical wiring works whether you're running this sketch or a
Bruce build. Note GDO2 is intentionally left unconnected per that pinout;
this sketch doesn't use it anyway.

CC1101 SCK       -> G5   CC1101 CSN  -> G2
CC1101 MOSI      -> G6   CC1101 GDO0 -> G3
CC1101 MISO/GDO1 -> G4   CC1101 GDO2 -> Not Connected
CC1101 VCC       -> 3V3 (Hat2-Bus 3V3 pin) -- verify it reads ~3.3V first
CC1101 GND       -> GND (Hat2-Bus GND pin)
<img width="393" height="719" alt="m5sticks3-cc1101" src="https://github.com/user-attachments/assets/dea3aab7-365d-4dd3-b069-2a81a71e83bf" />



Libraries needed (Arduino Library Manager): "M5Unified" (M5Stack).

Make sure your CC1101 module is the **915 MHz** variant for a North
American / Australian / New Zealand Vantage Pro2 (the cheap 433MHz eBay
boards will not receive it -- check the can/antenna markings). European/UK
stations use 868MHz; see "Other regions" below.

The StickS3 is a tiny board with limited free pins and a small 250mAh
battery -- if you're running this continuously, expect noticeably shorter
battery life than on a mains-powered or larger-battery device, since the
CC1101 receiver and constant channel-hop scanning are always-on.

## Software setup

1. Arduino IDE with the ESP32 board package installed, board set to the
   M5StickS3 profile (see
   [M5Stack's Arduino quick start](https://docs.m5stack.com/en/arduino/m5sticks3/program)).
2. Install the **M5Unified** library via Library Manager (or
   `https://github.com/m5stack/M5Unified`). M5GFX comes along with it.
3. Open `DavisWeatherReceiver.ino` (keep `CC1101.h/.cpp` and
   `DavisProtocol.h/.cpp` in the same sketch folder) and upload.

No other CC1101 library is required -- `CC1101.h/.cpp` talks to the chip
directly over SPI so the Davis-specific register configuration is fully
visible and easy to tune.

## How it works / limitations

The Davis ISS hops across 51 frequencies (902.5-927.5MHz, 0.5MHz spacing)
and sends one ~7ms packet roughly every 2.5 seconds. The *order* of that hop
sequence is a fixed but undocumented pattern that's the same for every
station of a given region -- some open-source projects (linked below)
capture and hard-code that exact sequence so their receiver "follows" the
ISS hop-for-hop and rarely misses a packet.

This sketch takes the simpler route: it continuously **scans** through all
51 channels (about 90ms dwell each, full sweep in well under 5 seconds),
listening for a valid sync word + CRC on each one. Because the ISS is
hopping on its own independent schedule, the receiver's sweep and the
ISS's transmissions drift in and out of alignment, so packets get caught
periodically rather than continuously -- expect each field to update every
few seconds to roughly a minute, rather than every single 2.5s
transmission. For a "glance at the current weather" display this is fine;
if you want every packet, port over the real hop table and timer-driven
synchronization from the projects below.

Each Davis message only carries one extra sensor reading per packet (wind
speed/direction are in every packet; temperature, humidity, rain, wind
gust, solar and UV each cycle through on their own schedule), so it can
take 10-20 seconds after first lock to populate every field on screen.

## Other regions (EU/UK 868MHz stations)

This sketch's channel table (`CC1101::setDavisChannel`) is hard-coded for
the 51-channel NA/AU/NZ table. European/UK Vantage Pro2 units use a
different, smaller 868MHz channel set -- if that's your station, you'll
need to adjust the channel count/frequencies in `CC1101.cpp` accordingly;
the rest of the protocol decoding is identical.

## Credits / further reading

This protocol was reverse-engineered by the ham radio / weather hacking
community, not by Davis. Sources used for this sketch:

- https://github.com/dekay/DavisRFM69/wiki -- RF & message protocol writeup
- https://github.com/cmatteri/CC1101-Weather-Receiver -- a CC1101-based
  receiver with full hop-table synchronization (a good reference if you
  want to improve on this sketch's simple scanning approach)
- https://madscientistlabs.blogspot.com -- DeKay's original reverse
  engineering blog posts
