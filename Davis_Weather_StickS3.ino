// DavisWeatherReceiver.ino
//
// Receives Davis Instruments ISS (Vantage Pro2 / Vue) wireless weather data
// using a CC1101 sub-1GHz module, and displays it on an M5Stack StickS3's
// screen.
//
// WIRING (StickS3 Hat2-Bus 2.54-16P header) -- this matches the official
// Bruce firmware pinout for the M5StickS3 CC1101 (wiki.bruce.computer), so
// the same physical wiring works whether you're running this sketch or a
// Bruce build. Note GDO2 is intentionally left unconnected per that pinout;
// this sketch doesn't use it anyway.
//   CC1101 SCK       -> G5   CC1101 CSN  -> G2
//   CC1101 MOSI      -> G6   CC1101 GDO0 -> G3
//   CC1101 MISO/GDO1 -> G4   CC1101 GDO2 -> Not Connected
//   CC1101 VCC       -> 3V3 (Hat2-Bus 3V3 pin) -- verify it reads ~3.3V first
//   CC1101 GND       -> GND (Hat2-Bus GND pin)
//
// Libraries needed (Arduino Library Manager): "M5Unified" (M5Stack).
//
// Region note: this sketch is configured for the North America / Australia /
// New Zealand 915 MHz, 51-channel hop table (902.5-927.5 MHz). European/UK
// Vantage Pro2 units use 868 MHz with a different, smaller channel set --
// see README.md if that's your station.
//
// Protocol references:
//   https://github.com/dekay/DavisRFM69/wiki  (RF Protocol & Message Protocol)
//   https://github.com/cmatteri/CC1101-Weather-Receiver

#include "M5Unified.h"
#include "CC1101.h"
#include "DavisProtocol.h"

// ---- User-configurable settings -----------------------------------------
#define CC1101_SCK    5
#define CC1101_MISO   4
#define CC1101_MOSI   6
#define CC1101_CS     2
#define CC1101_GDO0   3
#define CC1101_GDO2  -1   // not connected in the Bruce-compatible wiring; unused by this sketch

#define CC1101_XTAL_HZ   26000000UL   // most cheap CC1101 boards; try
                                       // 27000000UL if reception never locks
#define EXPECTED_TX_ID   0            // ISS dip-switch ID (0 = switch set to "1")
#define CHANNEL_DWELL_MS 90           // time to wait on each channel before
                                       // hopping if nothing is heard

CC1101 radio;
DavisWeather weather;

uint8_t currentChannel = 0;
uint32_t channelDeadline = 0;
uint32_t totalHops = 0;

void nextChannel() {
  currentChannel = (currentChannel + 1) % 51;
  radio.setDavisChannel(currentChannel);
  channelDeadline = millis() + CHANNEL_DWELL_MS;
  totalHops++;
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(1);
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(WHITE, BLACK);
  M5.Display.fillScreen(BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.println("Davis ISS Receiver");
  M5.Display.println("Initializing CC1101...");

  radio.begin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CS, CC1101_GDO0, CC1101_GDO2);
  radio.configureForDavis(CC1101_XTAL_HZ);
  nextChannel();
}

void drawScreen(int8_t rssiDbm) {
  auto &d = M5.Display;
  d.fillScreen(BLACK);
  d.setCursor(0, 0);

  uint32_t ageSec = weather.lastUpdateMs ? (millis() - weather.lastUpdateMs) / 1000 : 0;

  d.setTextColor(YELLOW, BLACK);
  d.printf("Davis ISS  id:%d  ch:%2d\n", weather.transmitterId, currentChannel);
  d.setTextColor(WHITE, BLACK);
  d.printf("RSSI:%4d dBm   pkts:%u  crc-err:%u\n", rssiDbm, weather.packetsReceived, weather.crcErrors);
  d.println();

  if (weather.haveTemp) {
    d.printf("Temp:     %5.1f F\n", weather.tempF);
  } else {
    d.println("Temp:     --");
  }

  if (weather.haveHumidity) {
    d.printf("Humidity: %5.1f %%\n", weather.humidityPct);
  } else {
    d.println("Humidity: --");
  }

  if (weather.haveWind) {
    d.printf("Wind:     %3d mph %s (%3.0f deg)\n", weather.windSpeedMph,
              davisCompassPoint(weather.windDirDeg), weather.windDirDeg);
  } else {
    d.println("Wind:     --");
  }

  if (weather.haveWindGust) {
    d.printf("Gust:     %3d mph (10 min max)\n", weather.windGustMph);
  }

  if (weather.haveRainRate || weather.haveRain) {
    d.printf("Rain:     %.2f in today,  %.2f in/hr\n", weather.rainTodayIn, weather.rainRateInPerHr);
  }

  if (weather.haveSolar) {
    d.printf("Solar:    %4.0f W/m2\n", weather.solarWm2);
  }
  if (weather.haveUv) {
    d.printf("UV index: %.1f\n", weather.uvIndex);
  }

  d.println();
  d.setTextColor(weather.lowBattery ? RED : GREEN, BLACK);
  d.printf("Battery: %s", weather.lowBattery ? "LOW" : "OK");
  d.setTextColor(WHITE, BLACK);
  d.printf("   last update: %lus ago\n", (unsigned long)ageSec);
}

uint32_t lastScreenDraw = 0;

void loop() {
  M5.update();

  int avail = radio.rxBytesAvailable();
  if (avail < 0) {
    // FIFO overflow -- flush and keep scanning
    radio.flushRx();
    nextChannel();
  } else if (avail >= 12) { // 10 payload bytes + RSSI + LQI (PKTCTRL1 APPEND_STATUS)
    uint8_t raw[12];
    radio.readFifo(raw, 12);

    uint8_t pkt[10];
    for (uint8_t i = 0; i < 10; i++) pkt[i] = davisReverseBits(raw[i]);
    uint8_t rawRssi = raw[10];
    // raw[11] low 7 bits = LQI, bit7 = CRC_OK per CC1101 hardware CRC -- we
    // ignore that bit since hardware CRC checking is disabled and we run
    // our own Davis CRC-16 check instead.

    bool ok = davisParsePacket(pkt, weather, EXPECTED_TX_ID);
    drawScreen(radio.lastRssiDbm(rawRssi));
    lastScreenDraw = millis();
    (void)ok;

    nextChannel();
  } else if ((int32_t)(millis() - channelDeadline) > 0) {
    nextChannel();
  }

  // Periodically refresh the "last update Ns ago" line even when no new
  // packet has arrived.
  if (millis() - lastScreenDraw > 1000) {
    uint8_t rawRssi = (uint8_t)radio.readStatusReg(CC1101_RSSI);
    drawScreen(radio.lastRssiDbm(rawRssi));
    lastScreenDraw = millis();
  }
}
