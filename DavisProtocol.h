// DavisProtocol.h
// Decoding for Davis Instruments ISS (Vantage Pro2 / Vue) wireless packets,
// based on the community reverse-engineering effort documented at
// https://github.com/dekay/DavisRFM69/wiki (RF Protocol & Message Protocol
// pages) and cross-checked against cmatteri/CC1101-Weather-Receiver, a
// CC1101-based implementation of the same protocol.
#pragma once
#include <Arduino.h>

struct DavisWeather {
  // Updated on every packet (any message type), since wind is sent every
  // transmission.
  bool     haveWind = false;
  uint8_t  windSpeedMph = 0;
  float    windDirDeg = 0;

  bool     haveTemp = false;
  float    tempF = 0;

  bool     haveHumidity = false;
  float    humidityPct = 0;

  bool     haveWindGust = false;
  uint8_t  windGustMph = 0;

  bool     haveUv = false;
  float    uvIndex = 0;

  bool     haveSolar = false;
  float    solarWm2 = 0;

  bool     haveRainRate = false;
  float    rainRateInPerHr = 0;

  bool     haveRain = false;
  uint8_t  rainBucketTips = 0;   // raw 0-127 cumulative counter from the ISS
  float    rainTodayIn = 0;      // accumulated by us from the deltas

  bool     lowBattery = false;
  uint8_t  transmitterId = 0;    // 0-7 (dip-switch ID minus 1)
  uint8_t  lastMsgType = 0;      // raw header nibble, for debugging

  uint32_t lastUpdateMs = 0;
  uint16_t packetsReceived = 0;
  uint16_t crcErrors = 0;

  // Internal bookkeeping for rain accumulation
  bool     haveRainBaseline = false;
};

// Reverses the bit order of a single byte. Davis ISS units transmit each
// byte least-significant-bit first; the CC1101 packet engine delivers bytes
// MSB-first, so every payload byte needs this fix-up before use.
uint8_t davisReverseBits(uint8_t b);

// CRC-16/CCITT (poly 0x1021, init 0, no reflect) used by Davis. Matches the
// "all bytes including the CRC bytes should reduce to zero" check described
// in the dekay wiki.
uint16_t davisCrc16(const uint8_t *data, uint8_t len);

// Validates and parses a 10-byte (already bit-reversed) Davis packet,
// updating `state` in place. Returns true if the CRC was valid and the
// packet was applied.
bool davisParsePacket(const uint8_t *pkt, DavisWeather &state, uint8_t expectedId);

// Small helper: 16-point compass direction text for a degree value.
const char *davisCompassPoint(float deg);
