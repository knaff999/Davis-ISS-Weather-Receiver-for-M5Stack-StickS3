// DavisProtocol.cpp
#include "DavisProtocol.h"

uint8_t davisReverseBits(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

uint16_t davisCrc16(const uint8_t *data, uint8_t len) {
  uint16_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= ((uint16_t)data[i]) << 8;
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

static bool checkCrc(const uint8_t *pkt) {
  uint16_t rxCrc = ((uint16_t)pkt[6] << 8) | pkt[7];
  bool isRepeaterPacket = !(pkt[8] == 0xFF && pkt[9] == 0xFF);
  uint16_t calc;
  if (!isRepeaterPacket) {
    calc = davisCrc16(pkt, 6); // bytes 0-5
  } else {
    uint8_t buf[8] = { pkt[0], pkt[1], pkt[2], pkt[3], pkt[4], pkt[5], pkt[8], pkt[9] };
    calc = davisCrc16(buf, 8);
  }
  return calc == rxCrc;
}

const char *davisCompassPoint(float deg) {
  static const char *pts[] = { "N","NNE","NE","ENE","E","ESE","SE","SSE",
                                "S","SSW","SW","WSW","W","WNW","NW","NNW" };
  int idx = (int)((deg / 22.5f) + 0.5f) % 16;
  if (idx < 0) idx += 16;
  return pts[idx];
}

bool davisParsePacket(const uint8_t *pkt, DavisWeather &state, uint8_t expectedId) {
  if (!checkCrc(pkt)) {
    state.crcErrors++;
    return false;
  }

  uint8_t header = pkt[0];
  uint8_t msgType = (header >> 4) & 0x0F;
  uint8_t txId    = header & 0x07;
  bool    lowBatt = (header & 0x08) != 0;

  if (txId != expectedId) {
    // Packet from a different transmitter ID (another nearby station, a
    // repeater, or noise that happened to pass the CRC). Ignore it.
    return false;
  }

  state.transmitterId = txId;
  state.lowBattery = lowBatt;
  state.lastMsgType = msgType;
  state.lastUpdateMs = millis();
  state.packetsReceived++;

  // Wind speed/direction are present in every packet.
  uint8_t windSpeed = pkt[1];
  uint8_t windDirRaw = pkt[2];
  state.windSpeedMph = windSpeed;
  state.windDirDeg = (windDirRaw == 0) ? 360.0f : (9.0f + windDirRaw * 342.0f / 255.0f);
  state.haveWind = true;

  uint8_t b3 = pkt[3], b4 = pkt[4], b5 = pkt[5];

  switch (msgType) {
    case 0x8: { // Temperature
      int16_t raw = ((int16_t)b3 << 8) | b4;
      state.tempF = raw / 160.0f;
      state.haveTemp = true;
      break;
    }
    case 0xA: { // Humidity
      uint16_t raw = (((uint16_t)(b4 >> 4)) << 8) | b3;
      state.humidityPct = raw / 10.0f;
      state.haveHumidity = true;
      break;
    }
    case 0x9: { // Wind gust (10-minute max)
      state.windGustMph = b3;
      state.haveWindGust = true;
      break;
    }
    case 0x4: { // UV index
      if (b3 != 0xFF) {
        uint16_t raw = (((uint16_t)b3 << 8) | b4) >> 6;
        state.uvIndex = raw / 50.0f;
        state.haveUv = true;
      }
      break;
    }
    case 0x6: { // Solar radiation
      if (b3 != 0xFF) {
        uint16_t raw = (((uint16_t)b3 << 8) | b4) >> 6;
        state.solarWm2 = raw * 1.757936f;
        state.haveSolar = true;
      }
      break;
    }
    case 0x5: { // Rain rate
      if (b3 == 0xFF) {
        state.rainRateInPerHr = 0;
      } else {
        bool strong = (b4 & 0x40) != 0;
        uint16_t timeBetween = (((uint16_t)(b4 & 0x30)) >> 4) * 250 + b3;
        if (timeBetween == 0) timeBetween = 1;
        float mmPerHr = strong ? (11520.0f / timeBetween) : (720.0f / timeBetween);
        state.rainRateInPerHr = mmPerHr / 25.4f; // mm -> in
      }
      state.haveRainRate = true;
      break;
    }
    case 0xE: { // Rain bucket tip counter (running total, wraps at 0x7F)
      uint8_t tips = b3 & 0x7F;
      if (state.haveRainBaseline) {
        uint8_t delta = (tips - state.rainBucketTips) & 0x7F;
        // A delta of 0 is "no new tip"; treat large deltas as noise/skip
        // (e.g. a missed wrap) rather than crediting a huge rain total.
        if (delta > 0 && delta < 100) {
          state.rainTodayIn += delta * 0.01f; // 0.01" per tip (US ISS)
        }
      } else {
        state.haveRainBaseline = true;
      }
      state.rainBucketTips = tips;
      state.haveRain = true;
      break;
    }
    default:
      // Message types 0x2 (Vue supercap), 0x3, 0x7 (Vue solar cell) and
      // 0xF (Leaf/Soil station) aren't decoded by this sketch.
      break;
  }

  return true;
}
