#include <Arduino.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <DHT.h>
#include <Adafruit_NeoPixel.h>
#include "IRLremote.h"
#include <Stepper.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <LedControl.h>
#include <TM1637.h>
#include "rfid_parser.h"


// ===================== CONFIG =====================
#define MAX_COMPONENTS 4
#define MAX_LEDS 35
#define MAX_SERVO 3
#define MAX_PIN 19
// Configurables
// #ifndef TOKAT_MAX_TOKENS
// #define TOKAT_MAX_TOKENS 2   // up to 40 tokens
// #endif
// #ifndef TOKAT_TOKEN_BYTES
// #define TOKAT_TOKEN_BYTES 2  // 20 chars + 1 NUL
// #endif
#define MAX_TOKEN 40

const char COMMAND_SEP = '_';
#define SEP "::"                   // multi-character separator
#define SEP_LEN (sizeof(SEP) - 1)  // length at compile time

String args;


// ===================== POINTER =======================

Adafruit_NeoPixel* neoPixel = nullptr;
Stepper* stepperMotor = nullptr;
LiquidCrystal_I2C* lcd = nullptr;
TM1637* tm = nullptr;
LedControl* ledMatrix = nullptr;
SoftwareSerial* bluetoothSerial = nullptr;
SoftwareSerial* rfidSerial = nullptr;
DHT* tempSensor = nullptr;
CNec IRLremote;

// ===================== PIN MANAGER =====================
namespace PinManager {
// #if defined(__AVR_ATmega2560__)
//   static const uint8_t MAX_PIN = 69;  // 0..53, A0..A15 (54..69)
// #elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
// #else
//   #error "Add board mapping for PinManager"
// #endif

static int8_t owner[MAX_PIN + 1];

void begin() {
  for (int i = 0; i <= MAX_PIN; i++) owner[i] = -1;
}
inline bool valid(int p) {
  return p >= 0 && p <= MAX_PIN;
}

inline bool isAnalogCapable(int pin) {
#ifdef NUM_ANALOG_INPUTS
  for (int i = 0; i < NUM_ANALOG_INPUTS; i++) {
    int mapped = -1;
#ifdef analogInputToDigitalPin
    mapped = analogInputToDigitalPin(i);
#endif
#if defined(A0)
    if (mapped == -1) mapped = A0 + i;
#endif
    if (mapped == pin) return true;
  }
#endif
  return false;
}

inline int lookup(const char* in) {
  if (!in || !*in) return -1;
  // Trim leading spaces
  while (*in == ' ' || *in == '\t' || *in == '\r' || *in == '\n') ++in;

  if (in[0] == 'A' || in[0] == 'a') {
    long idx = strtol(in + 1, nullptr, 10);
    if (idx < 0 || idx >= NUM_ANALOG_INPUTS) return -1;
    return A0 + (int)idx;
  }
  if (in[0] == 'D' || in[0] == 'd') {
    return (int)strtol(in + 1, nullptr, 10);
  }
  return (int)strtol(in, nullptr, 10);
}

inline int lookup(const String& in) {
  String s = in;
  s.trim();
  if (!s.length()) return -1;
  char c0 = s.charAt(0);
  if (c0 == 'A' || c0 == 'a') {
    int idx = s.substring(1).toInt();
    if (idx < 0 || idx >= NUM_ANALOG_INPUTS) return -1;
    return A0 + idx;
  }
  if (c0 == 'D' || c0 == 'd') return s.substring(1).toInt();
  return s.toInt();
}

inline bool reserve(int pin, uint8_t mode = 0xFF, bool needAnalog = false) {
  if (!valid(pin)) return false;
  if (needAnalog && !isAnalogCapable(pin)) return false;
  if (owner[pin] != -1) return false;
  owner[pin] = 1;
  if (mode != 0xFF) pinMode(pin, mode);
  return true;
}
inline void release(int pin) {
  if (valid(pin)) owner[pin] = -1;
}
}


// ===================== COMPONENT MODEL =====================
enum ComponentType : uint8_t {
  COMP_UNKNOWN = 0,
  // Raw I/O
  COMP_DIGITAL_IN,   // dr
  COMP_BUTTON,       // bt
  COMP_DIGITAL_OUT,  // dw
  COMP_ANALOG_IN,    // ar
  COMP_PWM_OUT,      // aw
  COMP_TM,           // Digital Display
  COMP_BUZZER,
  COMP_RGB,
  // Libraries / devices
  COMP_SERVO,       // servo
  COMP_LCD,         // lcd
  COMP_DHT,         // dht
  COMP_IR_TINY,     // ir
  COMP_RFID_UART,   // rfid (SoftwareSerial)
  COMP_BT_UART,     // bluetooth (SoftwareSerial)
  COMP_STEPPER,     // stepper
  COMP_LED_STRIP,   // leds (FastLED)
  COMP_LED_MATRIX,  // matrix (MD_MAX72XX)
  COMP_MOTOR,
  // Extra sensors
  COMP_JOYSTICK,    // js  (ax, ay, sw)
  COMP_ULTRASONIC,  // us  (trig, echo)
  COMP_THERMISTOR   // th  (analog in + beta calc)

};

struct Component {
  ComponentType type = COMP_UNKNOWN;
  void* obj = nullptr;      // library obj pointer (Servo*, LCD*, DHT*, etc.)
  uint8_t pins[6] = { 0 };  // up to 4 pins
  uint8_t nPins = 0;
  uint16_t param[4] = { 0 };  // generic ints (e.g., steps, baud, modules)
};

struct ComponentManager {
  Component items[MAX_COMPONENTS];
  uint8_t count = 0;
  int add(const Component& c) {
    if (count >= MAX_COMPONENTS) return -1;
    items[count] = c;
    return count++;
  }
  int findByPin(int pin) {
    for (uint8_t i = 0; i < count; i++)
      for (uint8_t j = 0; j < items[i].nPins; j++)
        if (items[i].pins[j] == pin) return i;
    return -1;
  }
  bool removeByPin(int pin) {
    int idx = findByPin(pin);
    if (idx < 0) return false;
    items[idx] = items[count - 1];
    count--;
    return true;
  }
} CM;

struct Token {
  const char* ptr;
  uint8_t len;

  bool valid() const {
    return ptr && len > 0;
  }

  void toBuffer(char* out, size_t outSize) const {
    if (!out || outSize == 0) return;
    if (!valid()) {
      out[0] = '\0';
      return;
    }
    size_t n = len;
    if (n > outSize - 1) n = outSize - 1;
    memcpy(out, ptr, n);
    out[n] = '\0';
  }

  int asInt() const {
    if (!valid()) return 0;
    char buf[21];
    toBuffer(buf, sizeof(buf));
    return strtol(buf, nullptr, 0);
  }

  int asPin() const {
    if (!valid()) return -1;
    char buf[21];
    toBuffer(buf, sizeof(buf));

    // Check for analog pins "A0".."A5"
    if (buf[0] == 'A' || buf[0] == 'a') {
      int analogIdx = atoi(buf + 1);  // parse after 'A'
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
      return analogIdx + 14;  // UNO/Nano analog pins start at 14
#elif defined(ARDUINO_AVR_MEGA2560)
      return analogIdx + 54;  // Mega analog pins start at 54
#else
      return analogIdx;  // fallback: return index itself
#endif
    }

    // Otherwise treat as digital pin number
    return atoi(buf);
  }
};

// Then declare the function that returns Token
inline Token nextToken(const char** cursor) {
  Token t = { nullptr, 0 };
  if (!cursor || !*cursor) return t;

  const char* p = *cursor;
  if (*p == '\0') return t;

  const char* e = strstr(p, "::");
  size_t len = e ? (size_t)(e - p) : strlen(p);
  if (len > 20) len = 20;

  t.ptr = p;
  t.len = (uint8_t)len;

  *cursor = e ? (e + 2) : (p + len);
  return t;
}

//== == == == == == == == == == = POOLS == == == == == == == == == == =
Servo gServos[MAX_SERVO];
bool gServoUsed[MAX_SERVO] = { false };
int allocServo() {
  for (int i = 0; i < MAX_SERVO; i++)
    if (!gServoUsed[i]) {
      gServoUsed[i] = true;
      return i;
    }
  return -1;
}
void freeServo(int idx) {
  if (idx >= 0 && idx < MAX_SERVO) gServoUsed[idx] = false;
}

// Sensor Helps

// This is function to Trigger the ultrasonic sensor and measure the distance
double ultraSonicDistance(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);                  // Set the trigger pin to low
  delayMicroseconds(2);                        // Wait for 2 microseconds
  digitalWrite(trigPin, HIGH);                 // set the trigger pin to high
  delayMicroseconds(10);                       // Wait for 10 microseconds
  digitalWrite(trigPin, LOW);                  // Set the trigger pin to low
  long microseconds = pulseIn(echoPin, HIGH);  // Measure the time for the echo to retur
  return (double)(microseconds / 29 / 2);      // Convert the time to distance in cm
}

// MOTORS

bool moveMotor(uint8_t speed, uint8_t direction, uint8_t pin1, uint8_t pin2, uint8_t enablePin) {
  // Control the motor direction based on the specified direction
  if (speed > 255) {
    speed = 254;
  } else if (speed < 1) {
    speed = 1;
  }
  switch (direction) {
    case 1:                           // clockwise
      digitalWrite(pin1, HIGH);       // Set pin1 high to turn clockwise
      digitalWrite(pin2, LOW);        // Set pin2 low
      analogWrite(enablePin, speed);  // Set motor speed
      return true;
    case 2:                           // anticlockwise
      digitalWrite(pin1, LOW);        // Set pin1 low to turn anti-clockwise
      digitalWrite(pin2, HIGH);       // Set pin2 high
      analogWrite(enablePin, speed);  // Set motor speed
      return true;
    case 3:                       // stop
      analogWrite(enablePin, 0);  // Stop the motor
      return true;
  }

  return false;
}

// RFID
static RfidState g_state = WAITING_FOR_STX;
static int rfid_g_nibble = -1;   // counts 0..11 across the 12 hex nibbles
static uint8_t rfid_g_frame[6];  // 5 data bytes + 1 checksum
static const uint8_t STX = 0x02;
static const uint8_t ETX = 0x03;
// Debounce window (adjust as you like)
static const unsigned long RFID_DEBOUNCE_MS = 300;

// Track last-seen tag and time
static uint32_t g_lastTagDec = 0;
static unsigned long g_lastTagTs = 0;

static RfidState dataParser(RfidState s, uint8_t c) {
  switch (s) {
    case WAITING_FOR_STX:
    case DATA_VALID:
      if (c == STX) {
        rfid_g_nibble = -1;
        return READING_DATA;
      }
      return WAITING_FOR_STX;

    case READING_DATA:
      {
        // Collect 12 ASCII-hex nibbles -> rfid_g_frame[0..5]
        if (++rfid_g_nibble < 12) {
          int n = hexNib(c);
          if (n < 0) return WAITING_FOR_STX;  // invalid char -> resync
          if ((rfid_g_nibble & 1) == 0) {
            rfid_g_frame[rfid_g_nibble >> 1] = (uint8_t)(n << 4);
          } else {
            rfid_g_frame[rfid_g_nibble >> 1] |= (uint8_t)n;
          }
          return READING_DATA;
        }

        // Next char must be ETX
        if (c != ETX) return WAITING_FOR_STX;

        // Verify checksum: XOR of 5 data bytes equals checksum byte
        uint8_t x = 0;
        for (int i = 0; i < 5; ++i) x ^= rfid_g_frame[i];
        if (x != rfid_g_frame[5]) return WAITING_FOR_STX;

        return DATA_VALID;
      }

    default:
      return WAITING_FOR_STX;
  }
}

// Returns true if this tag should be processed (i.e., not a recent duplicate)
static bool rfidShouldProcess(const uint8_t data[6]) {
  // Convert to your standard 32-bit number (last 4 bytes)
  uint32_t tagDec = ((uint32_t)data[1] << 24) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 8) | (uint32_t)data[4];

  unsigned long now = millis();

  // If same tag within debounce window -> ignore
  if (tagDec == g_lastTagDec && (now - g_lastTagTs) < RFID_DEBOUNCE_MS) {
    return false;
  }

  // Update “last seen”
  g_lastTagDec = tagDec;
  g_lastTagTs = now;
  return true;
}



// Copies out the parsed 6 bytes (5 data + checksum), then resets for next frame
void getData(uint8_t* outData, uint8_t& length) {
  length = sizeof(rfid_g_frame);
  for (uint8_t i = 0; i < length; ++i) outData[i] = rfid_g_frame[i];
  g_state = WAITING_FOR_STX;  // ready for the next frame
  // also clear nibble count
  rfid_g_nibble = -1;
}

// Last 4 bytes compose the common 32-bit “card number”
uint32_t getTag(const uint8_t* data) {
  return ((uint32_t)data[1] << 24) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 8) | (uint32_t)data[4];
}

void printHexTag(const uint8_t* data) {
  for (int i = 0; i < 5; i++) {
    if (data[i] < 0x10) Serial.print("0");  // leading zero
    Serial.print(data[i], HEX);
  }
}

bool rfidAvailable() {
  while (rfidSerial->available() > 0) {
    g_state = dataParser(g_state, (uint8_t)rfidSerial->read());
    if (g_state == DATA_VALID) {
      // --- drain any leftovers in the UART buffer ---
      while (rfidSerial->available() > 0) {
        rfidSerial->read();  // discard
      }
      return true;
    }
  }

  // drain whatever is in there
  return false;
}


//== == == == == == == == == == = HELPERS == == == == == == == == == == =

// Parse hex string "#RRGGBB" or "RRGGBB" into R, G, B values
static uint8_t hexNib(char c) {
  return (c >= '0' && c <= '9') ? c - '0' : (c >= 'a' && c <= 'f') ? c - 'a' + 10
                                          : (c >= 'A' && c <= 'F') ? c - 'A' + 10
                                                                   : 0;
}
void hexToRGB(const char* hex, uint8_t& r, uint8_t& g, uint8_t& b) {
  if (hex[0] == '#') hex++;
  if (!isxdigit(hex[0]) || !isxdigit(hex[1]) || !isxdigit(hex[2]) || !isxdigit(hex[3]) || !isxdigit(hex[4]) || !isxdigit(hex[5])) {
    r = g = b = 0;
    return;
  }
  r = (hexNib(hex[0]) << 4) | hexNib(hex[1]);
  g = (hexNib(hex[2]) << 4) | hexNib(hex[3]);
  b = (hexNib(hex[4]) << 4) | hexNib(hex[5]);
}

static inline bool isEmptyTok(const char* s) {
  return !s || *s == '\0';
}

char* analogName(int pinNumber, char* out, uint8_t n) {
  // Ensure n >= 3 for "A0", >= 4 for "A15", >= 6 for big "69"
  if (pinNumber >= 14 && pinNumber <= 19) {
    // Uno/Nano A0..A5
    if (n >= 3) {
      out[0] = 'A';
      out[1] = char('0' + (pinNumber - 14));
      out[2] = '\0';
    }
    return out;
  }
  if (pinNumber >= 54 && pinNumber <= 69) {
    // Mega A0..A15
    uint8_t idx = pinNumber - 54;  // 0..15
    if (idx < 10) {
      if (n >= 3) {
        out[0] = 'A';
        out[1] = char('0' + idx);
        out[2] = '\0';
      }
    } else {
      if (n >= 4) {
        out[0] = 'A';
        out[1] = '1';
        out[2] = char('0' + (idx - 10));
        out[3] = '\0';
      }
    }
    return out;
  }
  // Fallback: just the number (no printf)
  itoa(pinNumber, out, 10);
  return out;
}

// Case-insensitive, minimal checks, tiny flash footprint
static inline ComponentType parseComponentType(const char* t) {
  if (!t || !*t) return COMP_UNKNOWN;
  auto lc = [](char c) -> char {
    return (char)(c | 0x20);
  };
  char a = lc(t[0]), b = lc(t[1]);

  // Fast path: 2-letter codes
  if (t[0] && t[1] && !t[2]) {
    if (a == 'd' && b == 'r') return COMP_DIGITAL_IN;
    if (a == 'b' && b == 't') return COMP_BUTTON;
    if (a == 'd' && b == 'w') return COMP_DIGITAL_OUT;
    if (a == 'a' && b == 'r') return COMP_ANALOG_IN;
    if (a == 'a' && b == 'w') return COMP_PWM_OUT;
    if (a == 't' && b == 'm') return COMP_TM;
    if (a == 'u' && b == 's') return COMP_ULTRASONIC;
    if (a == 't' && b == 'h') return COMP_THERMISTOR;
    if (a == 'j' && b == 's') return COMP_JOYSTICK;
    if (a == 'i' && b == 'r') return COMP_IR_TINY;
  }

  // Longer tokens (check just enough chars to be unique)
  switch (a) {
    case 's':
      if (b == 'e' && t[2] == 'r') return COMP_SERVO;    // "servo"
      if (b == 't' && t[2] == 'e') return COMP_STEPPER;  // "stepper"
      break;
    case 'l':
      if (b == 'c' && t[2] == 'd') return COMP_LCD;                       // "lcd"
      if (b == 'e' && t[2] == 'd' && t[3] == 's') return COMP_LED_STRIP;  // "leds"
      break;
    case 'm':
      if (b == 'a') return COMP_LED_MATRIX;  // "matrix"
      if (b == 'o') return COMP_MOTOR;       // "motor"
      break;
    case 'd':
      if (b == 'h' && t[2] == 't') return COMP_DHT;  // "dht"
      break;
    case 'r':
      if (b == 'f' && t[2] == 'i') return COMP_RFID_UART;  // "rfi"
      if (b == 'g' && t[2] == 'b') return COMP_RGB;        // "rgb"
      break;
    case 'b':
      if (b == 'l') return COMP_BT_UART;  // "bluetooth"
      if (b == 'u') return COMP_BUZZER;   // "buzzer"
      break;
    case 't':
      if (b == 'h') return COMP_THERMISTOR;  // "thermistor"/"th"
      break;
    case 'j':
      if (b == 'o') return COMP_JOYSTICK;  // "joystick"
      break;
    case 'u':
      if (b == 'l') return COMP_ULTRASONIC;  // "ultrasonic"
      break;
  }
  return COMP_UNKNOWN;
}



bool isSensorType(ComponentType t) {
  return (t == COMP_DIGITAL_IN || t == COMP_ANALOG_IN || t == COMP_DHT || t == COMP_IR_TINY || t == COMP_JOYSTICK || t == COMP_ULTRASONIC || t == COMP_THERMISTOR || t == COMP_BUTTON || t == COMP_RFID_UART);
}
const __FlashStringHelper* typeName(ComponentType t) {
  switch (t) {
    case COMP_DIGITAL_IN: return F("dr");
    case COMP_BUTTON: return F("bt");
    case COMP_ANALOG_IN: return F("ar");
    case COMP_DHT: return F("dht");
    case COMP_IR_TINY: return F("ir");
    case COMP_JOYSTICK: return F("js");
    case COMP_ULTRASONIC: return F("ul");
    case COMP_THERMISTOR: return F("th");
    case COMP_RFID_UART: return F("rfi");
    default: return F("?");
  }
}

//== == == == == == == == == == = REGISTER HELPERS == == == == == == == == == == =
bool makeDigitalIn(Component& out, int pin) {
  if (!PinManager::reserve(pin, INPUT)) return false;
  out = {};
  out.type = COMP_DIGITAL_IN;
  out.pins[0] = pin;
  out.nPins = 1;
  return true;
}

bool makeButton(Component& out, int pin) {
  if (!PinManager::reserve(pin, INPUT_PULLUP)) return false;
  out = {};
  out.type = COMP_BUTTON;
  out.pins[0] = pin;
  out.nPins = 1;
  return true;
}
bool makeDigitalOut(Component& out, int pin) {
  if (!PinManager::reserve(pin, OUTPUT)) return false;
  out = {};
  out.type = COMP_DIGITAL_OUT;
  out.pins[0] = pin;
  out.nPins = 1;
  return true;
}
bool makeAnalogIn(Component& out, int apin) {
  if (!PinManager::reserve(apin, 0xFF, /*needAnalog*/ true)) return false;
  out = {};
  out.type = COMP_ANALOG_IN;
  out.pins[0] = apin;
  out.nPins = 1;
  return true;
}
bool makePWMOut(Component& out, int pin) {
  if (!PinManager::reserve(pin, OUTPUT)) return false;
  out = {};
  out.type = COMP_PWM_OUT;
  out.pins[0] = pin;
  out.nPins = 1;
  return true;
}

// Libraries
bool makeServo(Component& out, int pin) {
  int idx = allocServo();
  if (idx < 0) return false;
  if (!PinManager::reserve(pin, OUTPUT)) {
    freeServo(idx);
    return false;
  }

  Servo* s = &gServos[idx];
  if (s->attach(pin) == INVALID_SERVO) {
    PinManager::release(pin);
    freeServo(idx);
    return false;
  }
  out = {};
  out.type = COMP_SERVO;
  out.pins[0] = pin;
  out.nPins = 1;
  out.obj = s;
  return true;
}
bool makeLCD(Component& out, const String& payload) {
  const char* command = payload.c_str();
  nextToken(&command);
  int rows = nextToken(&command).asInt();
  int cols = nextToken(&command).asInt();
  int addr = nextToken(&command).asInt();
  if (cols <= 0 || rows <= 0) return false;
  // check pins
  int a4 = PinManager::lookup("A4");
  int a5 = PinManager::lookup("A5");

  if (!PinManager::reserve(a4, 0xFF, true)) return false;
  if (!PinManager::reserve(a5, 0xFF, true)) return false;
  static LiquidCrystal_I2C lcdObj(addr, cols, rows);
  lcd = &lcdObj;
  lcd->init();
  lcd->backlight();
  out = {};
  out.type = COMP_LCD;
  out.obj = lcd;
  out.param[0] = cols;
  out.param[1] = rows;
  out.param[2] = addr;
  out.nPins = 2;
  out.pins[0] = a5;
  out.pins[1] = a4;
  return true;
}
bool makeDHT(Component& out, int pin, const String& payload) {
  const char* command = payload.c_str();
  nextToken(&command);
  nextToken(&command);
  uint8_t type = nextToken(&command).asInt();
  if (!PinManager::reserve(pin, INPUT)) return false;
  static DHT d(pin, type == 1 ? DHT11 : DHT22);  // persists after return
  d.begin();
  tempSensor = &d;
  out = {};
  out.type = COMP_DHT;
  out.pins[0] = pin;
  out.nPins = 1;
  return true;
}
bool makeIRTiny(Component& out, int pin) {
  if (!PinManager::reserve(pin)) return false;
  IRLremote.begin(pin);
  out = {};
  out.type = COMP_IR_TINY;
  out.pins[0] = pin;
  out.nPins = 1;
  return true;
}

bool makeMotor(Component& out, const String& payload) {
  const char* command = payload.c_str();
  nextToken(&command);
  int en1Pin = nextToken(&command).asPin();
  int in1Pin = nextToken(&command).asPin();
  int in2Pin = nextToken(&command).asPin();
  out = {};
  if (!PinManager::reserve(en1Pin, OUTPUT)) return false;
  if (!PinManager::reserve(in1Pin, OUTPUT)) return false;
  if (!PinManager::reserve(in2Pin, OUTPUT)) return false;
  int en2Pin = nextToken(&command).asPin();
  int numberMotors = en2Pin > 0 ? 2 : 1;
  if (numberMotors == 2) {
    int in3Pin = nextToken(&command).asPin();
    int in4Pin = nextToken(&command).asPin();
    if (!PinManager::reserve(en2Pin, OUTPUT)) return false;
    if (!PinManager::reserve(in3Pin, OUTPUT)) return false;
    if (!PinManager::reserve(in4Pin, OUTPUT)) return false;
    out.pins[3] = en2Pin;
    out.pins[4] = in3Pin;
    out.pins[5] = in4Pin;
  }
  out.type = COMP_MOTOR;
  out.pins[0] = en1Pin;
  out.pins[1] = in1Pin;
  out.pins[2] = in2Pin;
  out.nPins = numberMotors == 2 ? 6 : 3;
  out.param[0] = numberMotors;
  return true;
}

bool makeTM(Component& out, const String& payload) {
  const char* command = payload.c_str();
  nextToken(&command);
  int dioPin = nextToken(&command).asPin();
  int clkPin = nextToken(&command).asPin();
  if (!PinManager::reserve(dioPin) || !PinManager::reserve(clkPin)) {
    return false;
  }
  static TM1637 tmObj(clkPin, dioPin);  // constructed the first time only
  tm = &tmObj;
  tm->begin();
  tm->setBrightness(100);
  tm->clearScreen();
  out = {};
  out.type = COMP_TM;
  out.pins[0] = dioPin;
  out.pins[1] = clkPin;
  out.nPins = 2;

  return true;
}

bool makeSoftUart(Component& out, ComponentType t, const String& payload) {
  const char* command = payload.c_str();
  nextToken(&command);
  int rx = nextToken(&command).asPin();
  int tx = nextToken(&command).asPin();
  int baud = nextToken(&command).asInt();
  if (!PinManager::reserve(rx, INPUT) || !PinManager::reserve(tx, OUTPUT)) {
    if (PinManager::valid(rx)) PinManager::release(rx);
    if (PinManager::valid(tx)) PinManager::release(tx);
    return false;
  }
  if (t == COMP_BT_UART) {
    static SoftwareSerial bt(rx, tx);
    bluetoothSerial = &bt;
    bluetoothSerial->begin(baud);
  } else if (t == COMP_RFID_UART) {
    static SoftwareSerial ss(rx, tx);
    rfidSerial = &ss;
    rfidSerial->begin(baud);
  }
  out = {};
  out.type = t;
  out.pins[0] = rx;
  out.pins[1] = tx;
  out.nPins = 2;
  out.param[0] = baud;
  return true;
}
bool makeStepper(Component& out, const String& payload) {
  const char* command = payload.c_str();
  nextToken(&command);

  int pin1 = nextToken(&command).asPin();
  int pin2 = nextToken(&command).asPin();
  int pin3 = nextToken(&command).asPin();
  int pin4 = nextToken(&command).asPin();

  int steps = nextToken(&command).asInt();
  int speed = nextToken(&command).asInt();

  if (!PinManager::reserve(pin1, OUTPUT) || !PinManager::reserve(pin2, OUTPUT) || !PinManager::reserve(pin3, OUTPUT) || !PinManager::reserve(pin4, OUTPUT)) {
    if (PinManager::valid(pin1)) PinManager::release(pin1);
    if (PinManager::valid(pin2)) PinManager::release(pin2);
    if (PinManager::valid(pin3)) PinManager::release(pin3);
    if (PinManager::valid(pin4)) PinManager::release(pin4);
    return false;
  }
  static Stepper st(steps, pin1, pin2, pin3, pin4);
  stepperMotor = &st;
  stepperMotor->setSpeed(speed);
  out = {};
  out.type = COMP_STEPPER;
  out.pins[0] = pin1;
  out.pins[1] = pin2;
  out.pins[2] = pin3;
  out.pins[3] = pin4;
  out.nPins = 4;
  out.param[0] = steps;
  return true;
}

static inline uint8_t neoTypeFromCode(uint8_t code) {
  uint8_t type = 0;
  uint8_t order = (uint8_t)(code & 0x0F);

  switch (order) {
    case 0: type |= NEO_RGB; break;
    case 1: type |= NEO_GRB; break;
    case 2: type |= NEO_BRG; break;
    case 3: type |= NEO_RBG; break;
    case 4: type |= NEO_GBR; break;
    case 5: type |= NEO_BGR; break;
    case 6: type |= NEO_RGBW; break;
    case 7: type |= NEO_GRBW; break;
    case 8: type |= NEO_WRGB; break;
    default: type |= NEO_GRB; break;  // sensible default
  }

  type |= (code & 0x80) ? NEO_KHZ800 : NEO_KHZ400;
  return type;
}

bool makePassiveBuzzer(Component& out, const String& payload) {
  const char* command = payload.c_str();
  nextToken(&command);

  int buzzerPin = nextToken(&command).asPin();
  if (!PinManager::reserve(buzzerPin)) {
    return false;
  }
  out = {};
  out.type = COMP_BUZZER;
  out.pins[0] = buzzerPin;
  out.nPins = 1;

  return true;
}

bool makeRGB(Component& out, const String& payload) {
  const char* command = payload.c_str();
  nextToken(&command);
  int redPin = nextToken(&command).asPin();
  int greenPin = nextToken(&command).asPin();
  int bluePin = nextToken(&command).asPin();
  if (!PinManager::reserve(redPin, OUTPUT)) return false;
  if (!PinManager::reserve(greenPin, OUTPUT)) return false;
  if (!PinManager::reserve(bluePin, OUTPUT)) return false;
  out = {};
  out.type = COMP_RGB;
  out.nPins = 3;
  out.pins[0] = redPin;
  out.pins[1] = greenPin;
  out.pins[2] = bluePin;
  return true;
}


// LED strip (see note about template pin below)
bool makeLEDStrip(Component& out, int pin, const String& payload) {
  const char* command = payload.c_str();
  nextToken(&command);
  nextToken(&command);
  int count = nextToken(&command).asInt();
  uint8_t code = nextToken(&command).asInt();  // default GRB@800k
  uint8_t type = neoTypeFromCode(code);
  uint8_t brightness = nextToken(&command).asInt();
  if (count <= 0 || count > MAX_LEDS) return false;
  if (!PinManager::reserve(pin, OUTPUT)) return false;

  // Create a NeoPixel neoPixel (GRB, 800 kHz typical WS2812B)
  static Adafruit_NeoPixel strip(count, pin, type);
  neoPixel = &strip;
  neoPixel->begin();
  neoPixel->clear();
  neoPixel->setBrightness(brightness);  // optional

  out = {};
  out.type = COMP_LED_STRIP;
  out.pins[0] = pin;
  out.nPins = 1;
  out.param[0] = count;  // save count for bounds checking
  return true;
}

bool makeLedMatrix(Component& out, const String& payload) {
  const char* command = payload.c_str();
  nextToken(&command);

  int dinPin = nextToken(&command).asInt();
  int csPin = nextToken(&command).asInt();
  int clkPin = nextToken(&command).asInt();
  if (!PinManager::reserve(dinPin, OUTPUT)) return false;
  if (!PinManager::reserve(csPin, OUTPUT)) return false;
  if (!PinManager::reserve(clkPin, OUTPUT)) return false;
  static LedControl m(dinPin, clkPin, csPin, 1);
  ledMatrix = &m;
  ledMatrix->shutdown(0, false);
  /* Set the brightness to a medium values */
  ledMatrix->setIntensity(0, 8);
  /* and clear the display */
  ledMatrix->clearDisplay(0);

  out = {};
  out.type = COMP_LED_MATRIX;
  out.pins[0] = dinPin;
  out.pins[1] = csPin;
  out.pins[2] = clkPin;
  out.nPins = 3;
  return true;
}

// ===== Extra sensors =====
// Joystick: ax (A), ay (A), sw (D)
bool makeJoystick(Component& out, const String& payload) {
  const char* command = payload.c_str();
  nextToken(&command);

  int axPin = nextToken(&command).asPin();
  int ayPin = nextToken(&command).asPin();
  int swPin = nextToken(&command).asPin();

  if (!PinManager::reserve(axPin, 0xFF, true)) return false;
  if (!PinManager::reserve(ayPin, 0xFF, true)) {
    PinManager::release(axPin);
    return false;
  }
  if (!PinManager::reserve(swPin, INPUT_PULLUP)) {
    PinManager::release(ayPin);
    PinManager::release(axPin);
    return false;
  }
  out = {};
  out.type = COMP_JOYSTICK;
  out.pins[0] = axPin;
  out.pins[1] = ayPin;
  out.pins[2] = swPin;
  out.nPins = 3;
  return true;
}


// Ultrasonic (HC-SR04): trig, echo
bool makeUltrasonic(Component& out, const String& payload) {
  const char* command = payload.c_str();
  nextToken(&command);

  int trig = nextToken(&command).asPin();
  int echo = nextToken(&command).asPin();

  if (!PinManager::reserve(trig, OUTPUT)) return false;
  if (!PinManager::reserve(echo, INPUT)) {
    PinManager::release(trig);
    return false;
  }
  out = {};
  out.type = COMP_ULTRASONIC;
  out.pins[0] = trig;
  out.pins[1] = echo;
  out.nPins = 2;
  return true;
}

// Thermistor: analog pin  + optional params (beta,Rfixed,R0,T0C)
bool makeThermistor(Component& out, const String& payload) {
  const char* command = payload.c_str();
  uint8_t cursor;
  nextToken(&command);

  int apin = nextToken(&command).asPin();
  if (!PinManager::reserve(apin, 0xFF, true)) return false;
  out = {};
  out.type = COMP_THERMISTOR;
  out.pins[0] = apin;
  out.nPins = 1;
  return true;
}


// ===================== SENSE (compact line) =====================
bool cmdSense() {
  for (uint8_t i = 0; i < CM.count; i++) {
    Component& c = CM.items[i];
    if (!isSensorType(c.type)) continue;

    Serial.print(typeName(c.type));
    Serial.print(':');
    int pin = c.pins[0];
    char officialPinName[6];
    analogName(pin, officialPinName, sizeof(officialPinName));  // "A0"
    Serial.print(officialPinName);
    Serial.print(':');

    switch (c.type) {
      case COMP_RFID_UART:
        if (rfidAvailable()) {
          uint8_t data[6], len;
          getData(data, len);

          // Debounce: only print/process if not a recent duplicate
          if (rfidShouldProcess(data)) {
            // Print hex (5 bytes)
            printHexTag(data);
          } else {
            // Optional: show nothing for duplicates
            Serial.print(F("0"));
          }
        } else {
          Serial.print(F("0"));
        }
        break;
      case COMP_DIGITAL_IN:
        {
          Serial.print(digitalRead(pin) ? F("1") : F("0"));
        }
        break;
      case COMP_BUTTON:
        {
          Serial.print(digitalRead(pin) == LOW ? F("1") : F("0"));
        }
        break;

      case COMP_ANALOG_IN:
        {
          Serial.print(analogRead(pin));
        }
        break;

      case COMP_DHT:
        {
          float h = tempSensor->readHumidity();
          float t = tempSensor->readTemperature();
          Serial.print(h);
          Serial.print('-');
          Serial.print(t);
        }
        break;

      case COMP_IR_TINY:
        {
          if (IRLremote.available()) {
            auto data = IRLremote.read();
            Serial.print(data.command);
          }
        }
        break;

      case COMP_JOYSTICK:
        {

          int ax = c.pins[0], ay = c.pins[1], sw = c.pins[2];

          // https://medium.com/@melaniechow/using-a-joystick-sensor-on-an-arduino-3498d7399464
          // This function was inspired by this Article
          int y = (analogRead(ay) * 4.9);
          delay(50);  // small pause needed between reading
          int x = (analogRead(ax) * 4.9);
          delay(50);

          x = (x - 2457);
          y = (y - 2541);

          double val = atan2(y, x) * 180 / 3.14159265358979;

          if (val < 0) {
            val += 360;
          }

          //convert to a double
          double new_x = x / 100.0;
          double new_y = y / 100.0;
          double distance = sqrt((new_x * new_x) + (new_y * new_y));
          Serial.print(digitalRead(sw) ? F("1") : F("0"));
          Serial.print('-');
          Serial.print(distance > 15 ? val : 0);
          Serial.print('-');
          Serial.print(distance > 15 ? F("1") : F("0"));
        }
        break;

      case COMP_ULTRASONIC:
        {
          int trig = c.pins[0], echo = c.pins[1];
          double cm = ultraSonicDistance(trig, echo);
          Serial.print(cm);
        }
        break;

      case COMP_THERMISTOR:
        {
          int apin = c.pins[0];
          int adc = analogRead(apin);
          // Beta equation

          float beta = 3950.0, Rf = 10000.0, R0 = 10000.0, T0C = 25.0;
          float T0K = T0C + 273.15;
          // voltage divider: R_therm = Rf * (ADC / (1023 - ADC))
          float Rth = (adc >= 1023) ? 1e9 : (adc <= 0 ? 1e-9 : Rf * ((float)adc / (1023.0f - adc)));
          float invT = (1.0f / T0K) + (1.0f / beta) * log(Rth / R0);
          float TK = 1.0f / invT;
          float TC = TK - 273.15f;
          Serial.print(TC, 2);
        }
        break;

      default: break;
    }
    Serial.print(';');
  }
  Serial.println(F("SENSE_COMPLETE"));

  return true;
}

// ===================== READ / WRITE (pin-number only) =====================

bool commandDigitalWrite(Component& component, int pin, const String& payload) {
  const char* command = payload.c_str();
  nextToken(&command);
  nextToken(&command);
  int state = nextToken(&command).asInt() == 1 ? HIGH : LOW;
  digitalWrite(pin, state);
  return true;
}

bool commandAnalogWrite(Component& component, int pin, const String& payload) {
  const char* command = payload.c_str();
  nextToken(&command);
  nextToken(&command);
  int state = nextToken(&command).asInt();
  analogWrite(pin, constrain(state, 0, 255));
  return true;
}

bool commandMoveServo(Component& component, const String& payload) {
  const char* command = payload.c_str();
  nextToken(&command);
  nextToken(&command);
  Servo* s = static_cast<Servo*>(component.obj);
  int angle = nextToken(&command).asInt();
  s->write(angle);
  return true;
}

bool commandLCDScreen(Component& component, const String& payload) {
  const char* command = payload.c_str();
  nextToken(&command);
  nextToken(&command);

  int option = nextToken(&command).asInt();
  if (option == 1) {
    lcd->clear();
    return true;
  } else if (option == 2) {
    lcd->backlight();
    return true;
  } else if (option == 3) {
    lcd->noBacklight();
    return true;
  } else if (option == 4) {
    lcd->blink_off();
    return true;
  } else if (option == 5) {
    uint8_t row = nextToken(&command).asInt();
    uint8_t col = nextToken(&command).asInt();
    lcd->setCursor(col, row);
    lcd->blink();
    return true;
  } else if (option == 6) {
    lcd->scrollDisplayRight();
    return true;
  } else if (option == 7) {
    lcd->scrollDisplayLeft();
    return true;
  } else if (option == 8) {
    Token tokenRow1 = nextToken(&command);
    Token tokenRow2 = nextToken(&command);
    char row1[21];
    char row2[21];
    tokenRow1.toBuffer(row1, sizeof(row1));
    tokenRow2.toBuffer(row2, sizeof(row2));
    lcd->clear();
    lcd->setCursor(0, 0);
    lcd->print(row1);
    lcd->setCursor(0, 1);
    lcd->print(row2);
    if (component.param[1] == 4) {
      char row3[21];
      char row4[21];
      Token tokenRow3 = nextToken(&command);
      Token tokenRow4 = nextToken(&command);
      tokenRow3.toBuffer(row3, sizeof(row3));
      tokenRow4.toBuffer(row4, sizeof(row4));
      lcd->setCursor(0, 2);
      lcd->print(row3);
      lcd->setCursor(0, 3);
      lcd->print(row4);
    }
    return true;
  } else if (option == 9) {
    uint8_t row = nextToken(&command).asInt();
    uint8_t col = nextToken(&command).asInt();
    lcd->setCursor(col, row);
    Token messageToken = nextToken(&command);
    char message[21];
    messageToken.toBuffer(message, sizeof(message));
    lcd->print(message);
    return true;
  }
  return false;
}

bool commandStepper(Component& component, const String& payload) {
  const char* command = payload.c_str();
  nextToken(&command);
  nextToken(&command);

  int steps = nextToken(&command).asInt();
  stepperMotor->step(steps);
  return true;
}

bool commandLedStrip(Component& component, const String& payload) {
  const char* command = payload.c_str();
  nextToken(&command);
  nextToken(&command);

  uint8_t option = nextToken(&command).asInt();
  if (option == 1) {
    neoPixel->show();
    return true;
  }
  uint8_t r, g, b;
  char color[20];
  uint8_t position = nextToken(&command).asInt();
  Token colorToken = nextToken(&command);
  colorToken.toBuffer(color, sizeof(color));
  hexToRGB(color, r, g, b);
  neoPixel->setPixelColor(position, r, g, b);

  return true;
}

bool commandLedMatrix(Component& component, const String& payload) {
  const char* command = payload.c_str();
  nextToken(&command);
  nextToken(&command);

  uint8_t option = nextToken(&command).asInt();
  if (option == 1) {
    int row = nextToken(&command).asInt();
    int col = nextToken(&command).asInt();
    bool isOn = nextToken(&command).asInt() == 1;
    ledMatrix->setLed(0, row, col, isOn);
    return true;
  }

  ledMatrix->setColumn(0, 0, nextToken(&command).asInt());
  ledMatrix->setColumn(0, 1, nextToken(&command).asInt());
  ledMatrix->setColumn(0, 2, nextToken(&command).asInt());
  ledMatrix->setColumn(0, 3, nextToken(&command).asInt());
  ledMatrix->setColumn(0, 4, nextToken(&command).asInt());
  ledMatrix->setColumn(0, 5, nextToken(&command).asInt());
  ledMatrix->setColumn(0, 6, nextToken(&command).asInt());
  ledMatrix->setColumn(0, 7, nextToken(&command).asInt());
  return true;
}

bool commandMotor(Component& component, const String& payload) {
  const char* command = payload.c_str();
  nextToken(&command);
  nextToken(&command);
  uint8_t whichMotor = nextToken(&command).asInt();
  uint8_t speed = nextToken(&command).asInt();
  uint8_t direction = nextToken(&command).asInt();
  uint8_t enablePin = whichMotor == 1 ? component.pins[0] : component.pins[3];
  uint8_t pin1 = whichMotor == 1 ? component.pins[1] : component.pins[4];
  uint8_t pin2 = whichMotor == 1 ? component.pins[2] : component.pins[5];
  return moveMotor(speed, direction, pin1, pin2, enablePin);
}

bool commandTmDisplay(Component& component, const String& payload) {
  const char* command = payload.c_str();
  nextToken(&command);
  nextToken(&command);
  bool colonOn = nextToken(&command).asInt() == 1;
  char messageChar[10];
  Token messageToken = nextToken(&command);
  messageToken.toBuffer(messageChar, sizeof(messageChar));
  const char* message = messageChar;
  if (isEmptyTok(message)) {
    message = "    ";
  }
  if (colonOn) {
    tm->colonOn();
  } else {
    tm->colonOff();
  }
  tm->display(message);
  return true;
}

bool commandBuzzer(Component& component, int pin, const String& payload) {
  const char* command = payload.c_str();
  nextToken(&command);
  nextToken(&command);

  int note = nextToken(&command).asInt();
  if (note == 0) {
    noTone(pin);
  } else {
    tone(pin, note);
  }

  return true;
}

bool commandBluetooth(Component& component, const String& payload) {
  const char* command = payload.c_str();
  nextToken(&command);
  nextToken(&command);
  char m[30];
  nextToken(&command).toBuffer(m, sizeof(m));
  bluetoothSerial->println(m);
  return true;
}

bool commandRGB(Component& component, const String& payload) {
  const char* command = payload.c_str();
  nextToken(&command);
  nextToken(&command);
  int redColor = nextToken(&command).asInt();
  int greenColor = nextToken(&command).asInt();
  int blueColor = nextToken(&command).asInt();
  analogWrite(component.pins[0], redColor);
  analogWrite(component.pins[1], greenColor);
  analogWrite(component.pins[2], blueColor);
}

bool cmdWrite(const String& payload) {
  const char* command = payload.c_str();
  char type[21];
  Token typeToken = nextToken(&command);
  typeToken.toBuffer(type, sizeof(type));
  int pin = nextToken(&command).asPin();
  ComponentType ct = parseComponentType(type);
  if (pin < 0) return false;
  int idx = CM.findByPin(pin);
  if (idx < 0) return false;

  Component& c = CM.items[idx];

  switch (ct) {
    case COMP_DIGITAL_OUT: return (c.type == COMP_DIGITAL_OUT) ? commandDigitalWrite(c, pin, payload) : false;
    case COMP_PWM_OUT: return (c.type == COMP_PWM_OUT) ? commandAnalogWrite(c, pin, payload) : false;
    case COMP_SERVO: return (c.type == COMP_SERVO) ? commandMoveServo(c, payload) : false;
    case COMP_LCD: return (c.type == COMP_LCD) ? commandLCDScreen(c, payload) : false;
    case COMP_STEPPER: return (c.type == COMP_STEPPER) ? commandStepper(c, payload) : false;
    case COMP_LED_STRIP: return (c.type == COMP_LED_STRIP) ? commandLedStrip(c, payload) : false;
    case COMP_LED_MATRIX: return (c.type == COMP_LED_MATRIX) ? commandLedMatrix(c, payload) : false;
    case COMP_MOTOR: return (c.type == COMP_MOTOR) ? commandMotor(c, payload) : false;
    case COMP_TM: return (c.type == COMP_TM) ? commandTmDisplay(c, payload) : false;
    case COMP_BUZZER: return (c.type == COMP_BUZZER) ? commandBuzzer(c, pin, payload) : false;  // this one still needs pin
    case COMP_RGB: return (c.type == COMP_RGB) ? commandRGB(c, payload) : false;
    case COMP_BT_UART: return (c.type == COMP_BT_UART) ? commandBluetooth(c, payload) : false;

    default: return false;
  }
}


// ===================== REGISTER  / LIST =====================


bool handleRegister(const String& payload) {
  const char* command = payload.c_str();
  char type[21];
  char pinName[21];
  Token typeToken = nextToken(&command);
  typeToken.toBuffer(type, sizeof(type));
  int pin = nextToken(&command).asPin();
  if (!type || !*type) return false;

  ComponentType ct = parseComponentType(type);

  Component component;
  bool ok = false;

  switch (ct) {
    case COMP_DIGITAL_IN: ok = makeDigitalIn(component, pin); break;
    case COMP_BUTTON: ok = makeButton(component, pin); break;
    case COMP_DIGITAL_OUT: ok = makeDigitalOut(component, pin); break;
    case COMP_ANALOG_IN: ok = makeAnalogIn(component, pin); break;
    case COMP_PWM_OUT: ok = makePWMOut(component, pin); break;

    case COMP_SERVO: ok = makeServo(component, pin); break;
    case COMP_LCD: ok = makeLCD(component, payload); break;  // parses cols/rows/addr internally
    case COMP_DHT: ok = makeDHT(component, pin, payload); break;
    case COMP_IR_TINY: ok = makeIRTiny(component, pin); break;
    case COMP_RFID_UART: ok = makeSoftUart(component, COMP_RFID_UART, payload); break;
    case COMP_BT_UART: ok = makeSoftUart(component, COMP_BT_UART, payload); break;
    case COMP_STEPPER: ok = makeStepper(component, payload); break;
    case COMP_LED_STRIP: ok = makeLEDStrip(component, pin, payload); break;
    case COMP_LED_MATRIX: ok = makeLedMatrix(component, payload); break;
    case COMP_MOTOR: ok = makeMotor(component, payload); break;
    case COMP_TM: ok = makeTM(component, payload); break;
    case COMP_BUZZER: ok = makePassiveBuzzer(component, payload); break;  // your impl reserves the pin from payload[1]
    case COMP_RGB: ok = makeRGB(component, payload); break;

    case COMP_JOYSTICK: ok = makeJoystick(component, payload); break;
    case COMP_ULTRASONIC: ok = makeUltrasonic(component, payload); break;
    case COMP_THERMISTOR: ok = makeThermistor(component, payload); break;

    default: return false;
  }

  if (!ok) return false;

  CM.add(component);
  return true;
}


// ===================== RESTART =====================
void doRestart() {
  Serial.println(F("Restarting..."));
  delay(20);
  wdt_enable(WDTO_15MS);
  while (true) {}
}

void printSuccessFailed(bool success) {
  if (success) {
    Serial.println(F("OK"));
  } else {
    Serial.println(F("ERR"));
  }
}

// ===================== ROUTER =====================
void handleLine(const String& raw) {
  String line = raw;
  line.trim();
  bool ok = false;
  if (!line.length()) return;

  if (line.equalsIgnoreCase("sense")) {
    ok = cmdSense();
  }

  if (line.equalsIgnoreCase("restart")) {
    doRestart();
    return;
  }

  if (line.startsWith("register::")) {
    ok = handleRegister(line.substring(10));
  }

  if (line.startsWith("write::")) {
    ok = cmdWrite(line.substring(7));
  }

  printSuccessFailed(ok);
}

// ===================== ARDUINO LIFECYCLE =====================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(30);  // small, so blocking is bounded
  delay(50);
  PinManager::begin();
  Serial.println("System:READY");
}

void loop() {
  if (Serial.available()) {
    args = Serial.readStringUntil('|');  // user types: pinA, 13|
    args.trim();                         // extra safety
    if (!args.length()) return;          // empty
    handleLine(args);
  }
}
