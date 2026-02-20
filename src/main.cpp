#include <Arduino.h>
#include "BluetoothSerial.h"

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>

// ============================================================
//  Config
// ============================================================
static const char* REMOTE_MAC_STR = "00:80:25:43:6D:20";
static const char* REMOTE_NAME    = "RACER RS 436D20";

static const uint32_t FAST_INTERVAL_MS   = 100;
static const uint32_t MEDIUM_INTERVAL_MS = 500;
static const uint32_t SLOW_INTERVAL_MS   = 1000;
static const uint32_t STATIC_INTERVAL_MS = 2000;

static const uint32_t RECONNECT_DELAY_MS = 5000;

static const bool PRINT_RAW_ANSWERS = false; // true = loguje každý Answer paket

static const char* WIFI_SSID = "Ondrej.it";
static const char* WIFI_PASS = "D0majed0ma!";

// ============================================================
//  Bluetooth + Web
// ============================================================
BluetoothSerial SerialBT;
WebServer server(80);

// ============================================================
//  Enums (from Rust)
// ============================================================
enum class KettlerDeviceType : uint8_t {
  Bike         = 1,
  Crosstrainer = 2,
  Racer        = 3,
  Rowing       = 4,
  Treadmill    = 5,
};

enum class KettlerDeviceState : uint16_t {
  Up   = 0,
  Down = 1,
};

enum class KettlerBrakeMode : uint16_t {
  ConstantPower = 0,
  ConstantBrake = 1,
};

enum class KettlerPowerRange : uint8_t {
  Below = 0,
  In    = 1,
  Above = 2,
};

enum class KettlerValue : uint16_t {
  DeviceState = 6,
  BrakeMode   = 7,

  Pulse = 8,
  Rpm   = 9,

  PowerSet = 10,
  PowerGet = 11,
  PowerMin = 12,
  PowerMax = 13,

  Speed     = 14, // deprecated
  Distance  = 15,
  Energy    = 16,
  Time      = 17,
  TimeMode  = 18,

  DeviceName = 20,

  DeviceType = 24,
  DeviceId   = 25,

  InPowerRange = 26,

  BrakeLevel    = 27,
  BrakeLevelMin = 28,
  BrakeLevelMax = 29,

  InclineSet = 30,
  InclineGet = 31,
  InclineMin = 32,
  InclineMax = 33,

  SpeedSet = 34,
  SpeedGet = 35,
  SpeedMin = 36,
  SpeedMax = 37,

  Online = 38,
};

enum class KettlerInstruction : uint8_t {
  Read   = 1,
  Write  = 2,
  Answer = 3,
};

// ============================================================
//  Helpers: MAC parsing, endian helpers
// ============================================================
bool parseMac(const char* macStr, uint8_t out[6]) {
  int vals[6];
  if (sscanf(macStr, "%x:%x:%x:%x:%x:%x",
             &vals[0], &vals[1], &vals[2], &vals[3], &vals[4], &vals[5]) != 6) {
    return false;
  }
  for (int i = 0; i < 6; i++) out[i] = (uint8_t)vals[i];
  return true;
}

// Rust u16_to_2u8: [MSB, LSB]
static inline void u16_to_2u8_be(uint16_t v, uint8_t out[2]) {
  out[0] = (uint8_t)(v >> 8);
  out[1] = (uint8_t)(v & 0xFF);
}
static inline uint16_t from_2u8_to_u16_be(const uint8_t* b) {
  return ((uint16_t)b[0] << 8) | (uint16_t)b[1];
}

static inline bool is_special_byte(uint8_t b) {
  return (b == 0x02 || b == 0x03 || b == 0x10);
}

// ============================================================
//  CRC16 (exactly like your Rust crc.rs)
// ============================================================
static const uint16_t CRC16_POLY = 0x8408;
static uint16_t crc_table[256];
static bool crc_table_ready = false;

void crc_init_table() {
  for (uint16_t i = 0; i < 256; i++) {
    uint16_t data = i;
    uint16_t crc  = 0;
    for (int bit = 0; bit < 8; bit++) {
      if (((data ^ crc) & 1) != 0) crc = (crc >> 1) ^ CRC16_POLY;
      else                        crc = (crc >> 1);
      data >>= 1;
    }
    crc_table[i] = crc;
  }
  crc_table_ready = true;
}

uint16_t kettler_crc16(const uint8_t* bytes, size_t len) {
  if (!crc_table_ready) crc_init_table();

  uint16_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    uint8_t idx = (uint8_t)((crc ^ bytes[i]) & 0xFF);
    crc = (crc >> 8) ^ crc_table[idx];
  }
  return crc;
}

// ============================================================
//  Simple ByteVec (write_channel + buffers)
// ============================================================
struct ByteVec {
  uint8_t* buf = nullptr;
  size_t len = 0;
  size_t cap = 0;

  ~ByteVec() { if (buf) free(buf); buf = nullptr; len = cap = 0; }

  void reserve(size_t n) {
    if (cap >= n) return;
    size_t newCap = cap ? cap : 256;
    while (newCap < n) newCap *= 2;
    buf = (uint8_t*)realloc(buf, newCap);
    cap = newCap;
  }

  void clear() { len = 0; }

  void push(uint8_t b) {
    reserve(len + 1);
    buf[len++] = b;
  }

  void append(const uint8_t* data, size_t n) {
    reserve(len + n);
    memcpy(buf + len, data, n);
    len += n;
  }

  void drain_front(size_t n) {
    if (n >= len) { len = 0; return; }
    memmove(buf, buf + n, len - n);
    len -= n;
  }
};

static ByteVec write_channel;

// ============================================================
//  Kettler Device Data
// ============================================================
struct KettlerDeviceData {
  // targets
  bool has_power_target = false;    uint16_t power_target = 0;
  bool has_speed_target = false;    uint16_t speed_target = 0;
  bool has_incline_target = false;  uint16_t incline_target = 0;

  // live values
  bool has_power = false;           uint16_t power = 0;
  bool has_speed = false;           uint16_t speed = 0;
  bool has_rpm = false;             uint16_t rpm = 0;
  bool has_pulse = false;           uint16_t pulse = 0;

  bool has_incline = false;         uint16_t incline = 0;
  bool has_incline_min = false;     uint16_t incline_min = 0;
  bool has_incline_max = false;     uint16_t incline_max = 0;

  bool has_distance = false;        uint16_t distance = 0;
  bool has_energy = false;          uint16_t energy = 0;
  bool has_time = false;            uint16_t time = 0;
  bool has_time_mode = false;       uint16_t time_mode = 0;

  bool has_online = false;          bool online = false;

  bool has_speed_min = false;       uint16_t speed_min = 0;
  bool has_speed_max = false;       uint16_t speed_max = 0;

  bool has_power_min = false;       uint16_t power_min = 0;
  bool has_power_max = false;       uint16_t power_max = 0;

  bool has_brake_level = false;     uint8_t brake_level = 0;
  bool has_brake_level_min = false; uint8_t brake_level_min = 0;
  bool has_brake_level_max = false; uint8_t brake_level_max = 0;

  bool has_device_name = false;     String device_name;
  bool has_device_id = false;       String device_id;

  bool has_device_type = false;     KettlerDeviceType device_type = KettlerDeviceType::Racer;

  bool has_device_state = false;    KettlerDeviceState device_state = KettlerDeviceState::Up;
  bool has_brake_mode = false;      KettlerBrakeMode brake_mode = KettlerBrakeMode::ConstantPower;
  bool has_power_range = false;     KettlerPowerRange power_range = KettlerPowerRange::In;
};

static KettlerDeviceData kdata;

// ============================================================
//  Package Analyzer (Rust-like)
// ============================================================
struct PackageAnalyzer {
  ByteVec package_data;
  bool unescape_next = false;
  bool in_crc_bytes = false;
  uint32_t num_crc_bytes = 0;
  uint32_t num_data_bytes = 0;
  bool is_first_byte = true;

  void reset() {
    package_data.clear();
    unescape_next = false;
    in_crc_bytes = false;
    num_crc_bytes = 0;
    num_data_bytes = 0;
    is_first_byte = true;
  }

  enum ResultType { Nothing, Clear, Package };

  ResultType next_byte(uint8_t byte) {
    if (is_first_byte) {
      is_first_byte = false;
      if (byte != 0x02) { reset(); return Clear; }
      return Nothing;
    }

    if (!unescape_next) {
      if (byte == 0x02) { reset(); return Clear; }
      if (byte == 0x03) {
        if (in_crc_bytes) { reset(); return Clear; }
        in_crc_bytes = true;
        return Nothing;
      }
      if (byte == 0x10) {
        unescape_next = true;
        return Nothing;
      }

      package_data.push(byte);
      if (in_crc_bytes) num_crc_bytes++; else num_data_bytes++;
    } else {
      if (byte == 0x22 || byte == 0x23 || byte == 0x30) {
        unescape_next = false;
        uint8_t realb = byte ^ 0x20;
        package_data.push(realb);
        if (in_crc_bytes) num_crc_bytes++; else num_data_bytes++;
      } else {
        reset();
        return Clear;
      }
    }

    if (num_crc_bytes == 2) return Package;
    return Nothing;
  }
};

static PackageAnalyzer analyzer;
static ByteVec last_package;

// ============================================================
//  Send framing + escaping + CRC
// ============================================================
void send_data_framed(const uint8_t* payload, size_t payload_len) {
  write_channel.push(0x02);

  for (size_t i = 0; i < payload_len; i++) {
    uint8_t b = payload[i];
    if (is_special_byte(b)) {
      write_channel.push(0x10);
      write_channel.push(b ^ 0x20);
    } else {
      write_channel.push(b);
    }
  }

  write_channel.push(0x03);

  uint16_t crc = kettler_crc16(payload, payload_len);
  uint8_t crc_bytes[2];
  u16_to_2u8_be(crc, crc_bytes);
  write_channel.push(crc_bytes[0]);
  write_channel.push(crc_bytes[1]);
}

void send_instruction(KettlerValue value, KettlerInstruction instr, const uint8_t* add, size_t add_len) {
  uint8_t value_bytes[2]; u16_to_2u8_be((uint16_t)value, value_bytes);
  uint8_t len_bytes[2];   u16_to_2u8_be((uint16_t)add_len, len_bytes);

  ByteVec payload;
  payload.reserve(5 + add_len);
  payload.push(value_bytes[0]);
  payload.push(value_bytes[1]);
  payload.push((uint8_t)instr);
  payload.push(len_bytes[0]);
  payload.push(len_bytes[1]);
  if (add_len) payload.append(add, add_len);

  send_data_framed(payload.buf, payload.len);
}

void send_instruction_u0(KettlerValue value, KettlerInstruction instr) {
  send_instruction(value, instr, nullptr, 0);
}
void send_instruction_u8(KettlerValue value, KettlerInstruction instr, uint8_t v) {
  send_instruction(value, instr, &v, 1);
}
void send_instruction_u16(KettlerValue value, KettlerInstruction instr, uint16_t v) {
  uint8_t b[2]; u16_to_2u8_be(v, b);
  send_instruction(value, instr, b, 2);
}

// ============================================================
//  Handshake
// ============================================================
void send_handshake() {
  static const uint8_t hs[] = {
    0x00, 0x01, 0x01, 0x00, 0x12, 0x2d, 0x24, 0xf2, 0x24, 0x96,
    0xa4, 0xff, 0x98, 0x29, 0xf9, 0x21, 0xbe, 0x9d, 0xaa, 0x9e,
    0x4d, 0x01, 0x17,
  };
  send_data_framed(hs, sizeof(hs));
}

// ============================================================
//  Parse helpers
// ============================================================
bool parse_u8(const uint8_t* data, size_t len, uint8_t& out) {
  if (len != 1) return false;
  out = data[0];
  return true;
}
bool parse_u16(const uint8_t* data, size_t len, uint16_t& out) {
  if (len != 2) return false;
  out = from_2u8_to_u16_be(data);
  return true;
}
bool parse_bool_u8(const uint8_t* data, size_t len, bool& out) {
  uint8_t v;
  if (!parse_u8(data, len, v)) return false;
  out = (v != 0);
  return true;
}
String parse_string(const uint8_t* data, size_t len) {
  String s;
  s.reserve(len);
  for (size_t i = 0; i < len; i++) s += (char)data[i];
  return s;
}

bool parse_device_type(const uint8_t* data, size_t len, KettlerDeviceType& out) {
  uint8_t c;
  if (!parse_u8(data, len, c)) return false;
  switch ((char)c) {
    case 'B': out = KettlerDeviceType::Bike;         return true;
    case 'X': out = KettlerDeviceType::Crosstrainer; return true;
    case 'T': out = KettlerDeviceType::Treadmill;    return true;
    case 'R': out = KettlerDeviceType::Rowing;       return true;
    case 'S': out = KettlerDeviceType::Racer;        return true;
    default: return false;
  }
}

bool parse_power_range(const uint8_t* data, size_t len, KettlerPowerRange& out) {
  uint8_t v;
  if (!parse_u8(data, len, v)) return false;
  if (v > 2) return false;
  out = (KettlerPowerRange)v;
  return true;
}

bool parse_device_state(const uint8_t* data, size_t len, KettlerDeviceState& out) {
  uint16_t v;
  if (!parse_u16(data, len, v)) return false;
  if (v > 1) return false;
  out = (KettlerDeviceState)v;
  return true;
}

bool parse_brake_mode(const uint8_t* data, size_t len, KettlerBrakeMode& out) {
  uint16_t v;
  if (!parse_u16(data, len, v)) return false;
  if (v > 1) return false;
  out = (KettlerBrakeMode)v;
  return true;
}

// ============================================================
//  Process incoming instruction
// ============================================================
void process_instruction_answer(const uint8_t* b, size_t blen) {
  if (blen < 5) return;

  uint16_t value_int = from_2u8_to_u16_be(b);
  uint8_t instruction_int = b[2];
  uint16_t add_len = from_2u8_to_u16_be(b + 3);

  if (blen != (size_t)add_len + 5) return;
  if (instruction_int != (uint8_t)KettlerInstruction::Answer) return;

  const uint8_t* add = b + 5;

  if (PRINT_RAW_ANSWERS) {
    Serial.printf("ANS value=%u instr=%u addLen=%u add=", value_int, instruction_int, add_len);
    for (uint16_t i = 0; i < add_len; i++) Serial.printf("%02X ", add[i]);
    Serial.println();
  }

  KettlerValue value = (KettlerValue)value_int;

  switch (value) {
    case KettlerValue::DeviceName:  kdata.device_name = parse_string(add, add_len); kdata.has_device_name = true; break;
    case KettlerValue::DeviceId:    kdata.device_id   = parse_string(add, add_len); kdata.has_device_id   = true; break;

    case KettlerValue::DeviceType: {
      KettlerDeviceType dt;
      if (parse_device_type(add, add_len, dt)) { kdata.device_type = dt; kdata.has_device_type = true; }
      break;
    }

    case KettlerValue::BrakeMode: {
      KettlerBrakeMode bm;
      if (parse_brake_mode(add, add_len, bm)) { kdata.brake_mode = bm; kdata.has_brake_mode = true; }
      break;
    }

    case KettlerValue::DeviceState: {
      KettlerDeviceState ds;
      if (parse_device_state(add, add_len, ds)) { kdata.device_state = ds; kdata.has_device_state = true; }
      break;
    }

    case KettlerValue::InPowerRange: {
      KettlerPowerRange pr;
      if (parse_power_range(add, add_len, pr)) { kdata.power_range = pr; kdata.has_power_range = true; }
      break;
    }

    case KettlerValue::Pulse: { uint16_t v; if (parse_u16(add, add_len, v)) { kdata.pulse = v; kdata.has_pulse = true; } break; }
    case KettlerValue::Rpm:   { uint16_t v; if (parse_u16(add, add_len, v)) { kdata.rpm   = v; kdata.has_rpm   = true; } break; }

    case KettlerValue::Online: {
      bool v;
      if (parse_bool_u8(add, add_len, v)) { kdata.online = !v; kdata.has_online = true; }
      break;
    }

    case KettlerValue::SpeedSet: { uint16_t v; if (parse_u16(add, add_len, v)) { kdata.speed_target = v; kdata.has_speed_target = true; } break; }
    case KettlerValue::SpeedGet: { uint16_t v; if (parse_u16(add, add_len, v)) { kdata.speed        = v; kdata.has_speed        = true; } break; }
    case KettlerValue::SpeedMin: { uint16_t v; if (parse_u16(add, add_len, v)) { kdata.speed_min    = v; kdata.has_speed_min    = true; } break; }
    case KettlerValue::SpeedMax: { uint16_t v; if (parse_u16(add, add_len, v)) { kdata.speed_max    = v; kdata.has_speed_max    = true; } break; }

    case KettlerValue::InclineSet: { uint16_t v; if (parse_u16(add, add_len, v)) { kdata.incline_target = v; kdata.has_incline_target = true; } break; }
    case KettlerValue::InclineGet: { uint16_t v; if (parse_u16(add, add_len, v)) { kdata.incline        = v; kdata.has_incline        = true; } break; }
    case KettlerValue::InclineMin: { uint16_t v; if (parse_u16(add, add_len, v)) { kdata.incline_min    = v; kdata.has_incline_min    = true; } break; }
    case KettlerValue::InclineMax: { uint16_t v; if (parse_u16(add, add_len, v)) { kdata.incline_max    = v; kdata.has_incline_max    = true; } break; }

    case KettlerValue::PowerSet: { uint16_t v; if (parse_u16(add, add_len, v)) { kdata.power_target = v; kdata.has_power_target = true; } break; }
    case KettlerValue::PowerGet: { uint16_t v; if (parse_u16(add, add_len, v)) { kdata.power        = v; kdata.has_power        = true; } break; }
    case KettlerValue::PowerMin: { uint16_t v; if (parse_u16(add, add_len, v)) { kdata.power_min    = v; kdata.has_power_min    = true; } break; }
    case KettlerValue::PowerMax: { uint16_t v; if (parse_u16(add, add_len, v)) { kdata.power_max    = v; kdata.has_power_max    = true; } break; }

    case KettlerValue::Distance: { uint16_t v; if (parse_u16(add, add_len, v)) { kdata.distance     = v; kdata.has_distance     = true; } break; }
    case KettlerValue::Energy:   { uint16_t v; if (parse_u16(add, add_len, v)) { kdata.energy       = v; kdata.has_energy       = true; } break; }
    case KettlerValue::Time:     { uint16_t v; if (parse_u16(add, add_len, v)) { kdata.time         = v; kdata.has_time         = true; } break; }
    case KettlerValue::TimeMode: { uint16_t v; if (parse_u16(add, add_len, v)) { kdata.time_mode    = v; kdata.has_time_mode    = true; } break; }

    case KettlerValue::BrakeLevel:    { uint8_t v; if (parse_u8(add, add_len, v)) { kdata.brake_level     = v; kdata.has_brake_level     = true; } break; }
    case KettlerValue::BrakeLevelMin: { uint8_t v; if (parse_u8(add, add_len, v)) { kdata.brake_level_min = v; kdata.has_brake_level_min = true; } break; }
    case KettlerValue::BrakeLevelMax: { uint8_t v; if (parse_u8(add, add_len, v)) { kdata.brake_level_max = v; kdata.has_brake_level_max = true; } break; }

    default: break;
  }
}

// ============================================================
//  Process package + CRC check
// ============================================================
void process_package(const uint8_t* pkg, size_t pkg_len) {
  if (pkg_len < 3) return;

  const size_t data_len = pkg_len - 2;
  const uint8_t* data = pkg;
  const uint8_t* crc_bytes = pkg + data_len;

  uint16_t crc_recv = from_2u8_to_u16_be(crc_bytes);
  uint16_t crc_calc = kettler_crc16(data, data_len);

  if (crc_recv != crc_calc) return;

  process_instruction_answer(data, data_len);
}

// ============================================================
//  IO pumps
// ============================================================
void pump_write_channel() {
  if (!SerialBT.connected()) return;
  if (write_channel.len == 0) return;

  size_t written = SerialBT.write(write_channel.buf, write_channel.len);
  if (written > 0) write_channel.drain_front(written);
}

void pump_read_channel() {
  while (SerialBT.connected() && SerialBT.available() > 0) {
    uint8_t b = (uint8_t)SerialBT.read();
    auto r = analyzer.next_byte(b);

    if (r == PackageAnalyzer::Clear) {
      analyzer.reset();
    } else if (r == PackageAnalyzer::Package) {
      last_package.clear();
      last_package.append(analyzer.package_data.buf, analyzer.package_data.len);
      analyzer.reset();
      process_package(last_package.buf, last_package.len);
    }
  }
}

// ============================================================
//  Poll profile
// ============================================================
static inline bool is_treadmill() {
  return kdata.has_device_type && (kdata.device_type == KettlerDeviceType::Treadmill);
}

void poll_fast() {
  // FAST: tohle chceme často
  send_instruction_u0(KettlerValue::Rpm,      KettlerInstruction::Read);
  send_instruction_u0(KettlerValue::Pulse,    KettlerInstruction::Read);
  send_instruction_u0(KettlerValue::PowerGet, KettlerInstruction::Read);
  send_instruction_u0(KettlerValue::SpeedGet, KettlerInstruction::Read);
}

void poll_medium() {
  // MEDIUM: stav + odpor + režimy
  send_instruction_u0(KettlerValue::BrakeLevel,   KettlerInstruction::Read);
  send_instruction_u0(KettlerValue::Online,       KettlerInstruction::Read);
  send_instruction_u0(KettlerValue::DeviceState,  KettlerInstruction::Read);
  send_instruction_u0(KettlerValue::BrakeMode,    KettlerInstruction::Read);
  send_instruction_u0(KettlerValue::InPowerRange, KettlerInstruction::Read);

  // u treadmillu se hodí incline get i častěji (ale dejme zatím medium)
  if (is_treadmill()) {
    send_instruction_u0(KettlerValue::InclineGet, KettlerInstruction::Read);
  }
}

void poll_slow() {
  // SLOW: kumulativní věci
  send_instruction_u0(KettlerValue::Distance, KettlerInstruction::Read);
  send_instruction_u0(KettlerValue::Energy,   KettlerInstruction::Read);
  send_instruction_u0(KettlerValue::Time,     KettlerInstruction::Read);
  send_instruction_u0(KettlerValue::TimeMode, KettlerInstruction::Read);
}

void poll_static_until_ready() {
  // nejdřív zjistit device type
  if (!kdata.has_device_type) {
    send_instruction_u0(KettlerValue::DeviceType, KettlerInstruction::Read);
    return;
  }

  // static ID info (jen pokud chybí)
  if (!kdata.has_device_name) send_instruction_u0(KettlerValue::DeviceName, KettlerInstruction::Read);
  if (!kdata.has_device_id)   send_instruction_u0(KettlerValue::DeviceId,   KettlerInstruction::Read);

  // limity (jen pokud chybí)
  if (!kdata.has_power_min) send_instruction_u0(KettlerValue::PowerMin, KettlerInstruction::Read);
  if (!kdata.has_power_max) send_instruction_u0(KettlerValue::PowerMax, KettlerInstruction::Read);

  if (!kdata.has_speed_min) send_instruction_u0(KettlerValue::SpeedMin, KettlerInstruction::Read);
  if (!kdata.has_speed_max) send_instruction_u0(KettlerValue::SpeedMax, KettlerInstruction::Read);

  if (!kdata.has_brake_level_min) send_instruction_u0(KettlerValue::BrakeLevelMin, KettlerInstruction::Read);
  if (!kdata.has_brake_level_max) send_instruction_u0(KettlerValue::BrakeLevelMax, KettlerInstruction::Read);

  // set values taky občas chceme znát (jen když chybí)
  if (!kdata.has_power_target) send_instruction_u0(KettlerValue::PowerSet, KettlerInstruction::Read);
  if (!kdata.has_speed_target) send_instruction_u0(KettlerValue::SpeedSet, KettlerInstruction::Read);

  // incline pouze treadmill
  if (is_treadmill()) {
    if (!kdata.has_incline_min) send_instruction_u0(KettlerValue::InclineMin, KettlerInstruction::Read);
    if (!kdata.has_incline_max) send_instruction_u0(KettlerValue::InclineMax, KettlerInstruction::Read);
    if (!kdata.has_incline_target) send_instruction_u0(KettlerValue::InclineSet, KettlerInstruction::Read);
    if (!kdata.has_incline) send_instruction_u0(KettlerValue::InclineGet, KettlerInstruction::Read);
  }
}

// ============================================================
//  Optional setters (like Rust)
// ============================================================
void kettler_set_brake_level(uint8_t lvl) {
  send_instruction_u8(KettlerValue::BrakeLevel, KettlerInstruction::Write, lvl);
}
void kettler_set_brake_mode(KettlerBrakeMode mode) {
  send_instruction_u8(KettlerValue::BrakeMode, KettlerInstruction::Write, (uint8_t)mode);
}
void kettler_set_online(bool on) {
  send_instruction_u8(KettlerValue::Online, KettlerInstruction::Write, (uint8_t)(!on));
}

// ============================================================
//  Web: CORS + helpers
// ============================================================
void add_cors_headers() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

void send_json(int code, const JsonDocument& doc) {
  add_cors_headers();
  String out;
  serializeJson(doc, out);
  server.send(code, "application/json; charset=utf-8", out);
}

void send_text(int code, const char* text) {
  add_cors_headers();
  server.send(code, "text/plain; charset=utf-8", text);
}
//  Web: handlers
// ============================================================
void handle_index() {
    String path = "/index.html";
    if (SPIFFS.exists(path)) {
        File file = SPIFFS.open(path, "r");
        server.streamFile(file, "text/html");
        file.close();
    } else {
        send_text(404, "index.html not found");
    }
}

void handle_beta() {
  String path = "/beta.html";
  if (SPIFFS.exists(path)) {
    File file = SPIFFS.open(path, "r");
    server.streamFile(file, "text/html");
    file.close();
  } else {
    send_text(404, "beta.html not found");
  }
}

// Obsluha statických souborů je řešena v onNotFound handleru

void handle_get_rpm() {
  JsonDocument doc;
  doc["connected"] = SerialBT.connected();
  if (kdata.has_rpm) doc["rpm"] = kdata.rpm;
  else doc["rpm"] = nullptr;
  send_json(200, doc);
}

void handle_get_brake() {
  JsonDocument doc;
  doc["connected"] = SerialBT.connected();
  if (kdata.has_brake_level) doc["brake"] = kdata.brake_level;
  else doc["brake"] = nullptr;
  send_json(200, doc);
}

void handle_set_brake() {
  if (!SerialBT.connected()) {
    JsonDocument doc;
    doc["success"] = false;
    doc["message"] = "Kettler neni pripojen";
    send_json(503, doc);
    return;
  }

  if (!server.hasArg("plain")) {
    JsonDocument doc;
    doc["success"] = false;
    doc["message"] = "Chybi body";
    send_json(400, doc);
    return;
  }

  JsonDocument in;
  if (deserializeJson(in, server.arg("plain"))) {
    JsonDocument doc;
    doc["success"] = false;
    doc["message"] = "Neplatny JSON";
    send_json(400, doc);
    return;
  }

  if (!in["brake"].is<int>()) {
    JsonDocument doc;
    doc["success"] = false;
    doc["message"] = "Chybi pole brake";
    send_json(400, doc);
    return;
  }

  int brake = in["brake"].as<int>();
  int v = brake;
  if (v < 1) v = 1;
  if (v > 20) v = 20;

  // stejně jako v Rustu: online=true, mode=ConstantBrake, nastav level
  kettler_set_online(true);
  kettler_set_brake_mode(KettlerBrakeMode::ConstantBrake);
  kettler_set_brake_level((uint8_t)v);

  Serial.printf("[API] POST /api/brake <- %d (clamped %d)\n", brake, v);

  JsonDocument doc;
  doc["success"] = true;
  doc["brake"] = v;
  String msg = "Brake level nastaven na ";
  msg += v;
  doc["message"] = msg;
  send_json(200, doc);
}

void handle_set_online() {
  if (!SerialBT.connected()) {
    JsonDocument doc;
    doc["success"] = false;
    doc["message"] = "Kettler neni pripojen";
    send_json(503, doc);
    return;
  }

  if (!server.hasArg("plain")) {
    JsonDocument doc;
    doc["success"] = false;
    doc["message"] = "Chybi body";
    send_json(400, doc);
    return;
  }

  JsonDocument in;
  if (deserializeJson(in, server.arg("plain"))) {
    JsonDocument doc;
    doc["success"] = false;
    doc["message"] = "Neplatny JSON";
    send_json(400, doc);
    return;
  }

  if (!in["online"].is<bool>()) {
    JsonDocument doc;
    doc["success"] = false;
    doc["message"] = "Chybi pole online";
    send_json(400, doc);
    return;
  }

  bool on = in["online"].as<bool>();
  kettler_set_online(on);

  JsonDocument doc;
  doc["success"] = true;
  doc["online"] = on;
  doc["message"] = on ? "Online=true odeslano" : "Online=false odeslano";
  send_json(200, doc);
}

void handle_get_all() {
  JsonDocument doc;
  doc["connected"] = SerialBT.connected();

  if (kdata.has_device_name) doc["device_name"] = kdata.device_name; else doc["device_name"] = nullptr;
  if (kdata.has_device_id)   doc["device_id"] = kdata.device_id; else doc["device_id"] = nullptr;
  if (kdata.has_device_type) doc["device_type"] = (uint8_t)kdata.device_type; else doc["device_type"] = nullptr;

  if (kdata.has_online) doc["online"] = kdata.online; else doc["online"] = nullptr;

  if (kdata.has_device_state) doc["device_state"] = (uint16_t)kdata.device_state; else doc["device_state"] = nullptr;
  if (kdata.has_brake_mode) doc["brake_mode"] = (uint16_t)kdata.brake_mode; else doc["brake_mode"] = nullptr;
  if (kdata.has_power_range) doc["power_range"] = (uint8_t)kdata.power_range; else doc["power_range"] = nullptr;

  if (kdata.has_pulse) doc["pulse"] = kdata.pulse; else doc["pulse"] = nullptr;
  if (kdata.has_rpm) doc["rpm"] = kdata.rpm; else doc["rpm"] = nullptr;

  if (kdata.has_speed) doc["speed_get"] = kdata.speed; else doc["speed_get"] = nullptr;
  if (kdata.has_speed_target) doc["speed_set"] = kdata.speed_target; else doc["speed_set"] = nullptr;
  if (kdata.has_speed_min) doc["speed_min"] = kdata.speed_min; else doc["speed_min"] = nullptr;
  if (kdata.has_speed_max) doc["speed_max"] = kdata.speed_max; else doc["speed_max"] = nullptr;

  if (kdata.has_power) doc["power_get"] = kdata.power; else doc["power_get"] = nullptr;
  if (kdata.has_power_target) doc["power_set"] = kdata.power_target; else doc["power_set"] = nullptr;
  if (kdata.has_power_min) doc["power_min"] = kdata.power_min; else doc["power_min"] = nullptr;
  if (kdata.has_power_max) doc["power_max"] = kdata.power_max; else doc["power_max"] = nullptr;

  if (kdata.has_brake_level) doc["brake_level"] = kdata.brake_level; else doc["brake_level"] = nullptr;
  if (kdata.has_brake_level_min) doc["brake_level_min"] = kdata.brake_level_min; else doc["brake_level_min"] = nullptr;
  if (kdata.has_brake_level_max) doc["brake_level_max"] = kdata.brake_level_max; else doc["brake_level_max"] = nullptr;

  if (kdata.has_distance) doc["distance"] = kdata.distance; else doc["distance"] = nullptr;
  if (kdata.has_energy) doc["energy"] = kdata.energy; else doc["energy"] = nullptr;
  if (kdata.has_time) doc["time"] = kdata.time; else doc["time"] = nullptr;
  if (kdata.has_time_mode) doc["time_mode"] = kdata.time_mode; else doc["time_mode"] = nullptr;

  // incline only meaningful for treadmill; but still expose if present
  if (kdata.has_incline) doc["incline_get"] = kdata.incline; else doc["incline_get"] = nullptr;
  if (kdata.has_incline_target) doc["incline_set"] = kdata.incline_target; else doc["incline_set"] = nullptr;
  if (kdata.has_incline_min) doc["incline_min"] = kdata.incline_min; else doc["incline_min"] = nullptr;
  if (kdata.has_incline_max) doc["incline_max"] = kdata.incline_max; else doc["incline_max"] = nullptr;

  send_json(200, doc);
}

void handle_options_any() {
  add_cors_headers();
  server.send(204);
}

// ============================================================
//  Connect / Reconnect
// ============================================================
bool connectToBike() {
  uint8_t mac[6];
  if (!parseMac(REMOTE_MAC_STR, mac)) {
    Serial.println("Invalid MAC string.");
    return false;
  }

  Serial.printf("Connecting to \"%s\" (%s)...\n", REMOTE_NAME, REMOTE_MAC_STR);

  SerialBT.enableSSP();

  // přímé připojení na channel=1 (obejde discovery)
  if (!SerialBT.connect(mac, 1, ESP_SPP_SEC_NONE, ESP_SPP_ROLE_MASTER)) return false;

  Serial.println("Connected. Sending handshake...");
  analyzer.reset();
  last_package.clear();
  write_channel.clear();

  // (volitelně) reset “has_*” flags, aby se static znovu načetl po reconnectu
  // pokud chceš tvrdý reset, odkomentuj:
  // kdata = KettlerDeviceData();

  send_handshake();
  return true;
}

// ============================================================
//  Arduino setup/loop
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  // BT init (client mode)
  if (!SerialBT.begin("ESP32-KDRI-Client", true)) {
    Serial.println("BluetoothSerial.begin failed");
    while (true) delay(1000);
  }

  crc_init_table();
  analyzer.reset();

  // Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Connecting WiFi SSID=%s", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi connected, IP: ");
  Serial.println(WiFi.localIP());

  // Inicializace SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An error occurred while mounting SPIFFS");
    return;
  }

  // Web server routes
  server.on("/", HTTP_GET, handle_index);
  server.on("/beta", HTTP_GET, handle_beta);
  server.on("/api/all", HTTP_GET, handle_get_all);
  server.on("/api/rpm", HTTP_GET, handle_get_rpm);

  server.on("/api/brake", HTTP_GET, handle_get_brake);
  server.on("/api/brake", HTTP_POST, handle_set_brake);

  server.on("/api/online", HTTP_POST, handle_set_online);

  // OPTIONS preflight
  server.on("/api/all", HTTP_OPTIONS, handle_options_any);
  server.on("/api/rpm", HTTP_OPTIONS, handle_options_any);
  server.on("/api/brake", HTTP_OPTIONS, handle_options_any);
  server.on("/api/online", HTTP_OPTIONS, handle_options_any);

  // Handler pro statické soubory
  server.onNotFound([]() {
    String path = server.uri();
    if (path.endsWith("/")) {
        path += "index.html";
    }
    
    String contentType = "text/plain";
    if (path.endsWith(".html")) contentType = "text/html";
    else if (path.endsWith(".css")) contentType = "text/css";
    else if (path.endsWith(".js")) contentType = "application/javascript";
    else if (path.endsWith(".png")) contentType = "image/png";
    else if (path.endsWith(".jpg") || path.endsWith(".jpeg")) contentType = "image/jpeg";
    else if (path.endsWith(".gif")) contentType = "image/gif";
    else if (path.endsWith(".ico")) contentType = "image/x-icon";
    
    if (SPIFFS.exists(path)) {
        File file = SPIFFS.open(path, "r");
        server.streamFile(file, contentType);
        file.close();
    } else {
        server.send(404, "text/plain", "File not found");
    }
  });

  server.begin();
  Serial.println("HTTP server started.");

  Serial.println("ESP32 Kettler client ready.");
}

void loop() {
  static uint32_t lastFast = 0;
  static uint32_t lastMedium = 0;
  static uint32_t lastSlow = 0;
  static uint32_t lastStatic = 0;

  // web
  server.handleClient();

  // BT connect loop
  if (!SerialBT.connected()) {
    if (!connectToBike()) {
      Serial.println("Connect failed, retrying...");
      SerialBT.disconnect();
      delay(RECONNECT_DELAY_MS);
      return;
    }
  }

  // BT IO
  pump_read_channel();
  pump_write_channel();

  // Poll profile timers
  uint32_t now = millis();

  if (now - lastFast >= FAST_INTERVAL_MS) {
    lastFast = now;
    poll_fast();
  }
  if (now - lastMedium >= MEDIUM_INTERVAL_MS) {
    lastMedium = now;
    poll_medium();
  }
  if (now - lastSlow >= SLOW_INTERVAL_MS) {
    lastSlow = now;
    poll_slow();
  }
  if (now - lastStatic >= STATIC_INTERVAL_MS) {
    lastStatic = now;
    poll_static_until_ready();
  }

  delay(2);
}
