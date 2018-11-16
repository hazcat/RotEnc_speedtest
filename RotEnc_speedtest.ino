// portions of this file were based on code from the makelangelo firmware
// http://www.github.com/MarginallyClever/makelangeloFirmware


// encoder pins
#define BTN_EN1 31  //[RAMPS14-SMART-ADAPTER]
#define BTN_EN2 33  //[RAMPS14-SMART-ADAPTER]
#define BTN_ENC 35  //[RAMPS14-SMART-ADAPTER]

#define BAUD (57600)  // How fast is the Arduino talking?

#define BLEN_C 2
#define BLEN_B 1
#define BLEN_A 0

#define ENCROT0 0
#define ENCROT1 2
#define ENCROT2 3
#define ENCROT3 1

int8_t lcd_turn = 0;
static uint8_t lcd_rot_old = 0;
unsigned long delta_time_start = 0;

uint8_t bm_a = 0;
uint8_t bm_b = 0;

// original code
void read_input_pot() {
  // detect potentiometer changes
  int en1 = digitalRead(BTN_EN1);
  int en2 = digitalRead(BTN_EN2);
  uint8_t rot = 0;
  if (en1 == LOW) rot = B01;
  if (en2 == LOW) rot |= B10;

  // potentiometer uses grey code.  Pattern is 0 3
  if (rot != lcd_rot_old) {
    switch (rot) {
      case ENCROT0:
        if (lcd_rot_old == ENCROT3) lcd_turn++;
        if (lcd_rot_old == ENCROT1) lcd_turn--;
        break;
      case ENCROT1:
        if (lcd_rot_old == ENCROT0) lcd_turn++;
        if (lcd_rot_old == ENCROT2) lcd_turn--;
        break;
      case ENCROT2:
        if (lcd_rot_old == ENCROT1) lcd_turn++;
        if (lcd_rot_old == ENCROT3) lcd_turn--;
        break;
      case ENCROT3:
        if (lcd_rot_old == ENCROT2) lcd_turn++;
        if (lcd_rot_old == ENCROT0) lcd_turn--;
        break;
    }
    lcd_rot_old = rot;
  }
}

volatile uint8_t* port_enc_a = nullptr;
volatile uint8_t* port_enc_b = nullptr;

void read_input_pot_direct_read_from_pins() {
  // doing two reads. Grab the port value and then do the two reads
  uint8_t port = (*port_enc_a);

  uint8_t rot = ((port & bm_a) == bm_a) ? B01 : 0;
  if ((port & bm_b) == bm_b) {
    rot |= B10;
  }

  // potentiometer uses grey code.  Pattern is 0 3
  if (rot != lcd_rot_old) {
    switch (rot) {
      case ENCROT0:
        if (lcd_rot_old == ENCROT3) lcd_turn++;
        if (lcd_rot_old == ENCROT1) lcd_turn--;
        break;
      case ENCROT1:
        if (lcd_rot_old == ENCROT0) lcd_turn++;
        if (lcd_rot_old == ENCROT2) lcd_turn--;
        break;
      case ENCROT2:
        if (lcd_rot_old == ENCROT1) lcd_turn++;
        if (lcd_rot_old == ENCROT3) lcd_turn--;
        break;
      case ENCROT3:
        if (lcd_rot_old == ENCROT2) lcd_turn++;
        if (lcd_rot_old == ENCROT0) lcd_turn--;
        break;
    }
    lcd_rot_old = rot;
  }
}

uint8_t shift_a = 0;
uint8_t shift_b = 0;
static uint8_t last_input = 0;
static uint8_t bitmask_a_and_b = 0;

#define LCD_INPUT_REVERSE 1

#if LCD_INPUT_REVERSE
constexpr int8_t kDIR_A = -1;
constexpr int8_t kDIR_B = 1;
#else
constexpr int8_t kDIR_A = 1;
constexpr int8_t kDIR_B = -1;
#endif

#define INPUT_PINS_ON_SAME_PORT 1

/*
This table is a flattened 4x4 2d array.
*/
// clang-format off
static const int8_t enc_states[] PROGMEM = {
    0,          kDIR_A, kDIR_B, 0,
    kDIR_B,     0,      0,      kDIR_A,
    kDIR_A,     0,      0,      kDIR_B, 
    0,          kDIR_B, kDIR_A, 0};
// clang-format on

void read_input_pot_state_array() {
#if INPUT_PINS_ON_SAME_PORT
  uint8_t port = (*port_enc_a);
  uint8_t encoder = ((port & bm_a) == bm_a) ? B01 : 0;
  if ((port & bm_b) == bm_b) {
    encoder |= B10;
  }
#else
  uint8_t encoder = (((*port_enc_a) & bm_a) == bm_a) ? B01 : 0;
  if (((*port_enc_b) & bm_b) == bm_b) {
    encoder |= B10;
  }
#endif

  if (encoder != lcd_rot_old) {
    // build an array index: shift encoder left by 2 bits and combine the new
    // and old values
    uint8_t index = (encoder * 4) + lcd_rot_old;
    lcd_turn += pgm_read_byte(&enc_states[index]);
    lcd_rot_old = encoder;
  }
}

void consume_input() {
  // grab the time prior to mucking with serial output
  unsigned long now = micros();
  unsigned long delta_ms = now - delta_time_start;

  Serial.print("Turn: ");
  Serial.print(lcd_turn);
  Serial.print(" uS: ");
  Serial.println(delta_ms);
  lcd_turn = 0;
  delta_time_start = micros();
}

void setup() {
  Serial.begin(BAUD);
  pinMode(BTN_EN1, INPUT);
  pinMode(BTN_EN2, INPUT);
  pinMode(BTN_ENC, INPUT);
  digitalWrite(BTN_EN1, HIGH);
  digitalWrite(BTN_EN2, HIGH);
  digitalWrite(BTN_ENC, HIGH);

  bm_a = digitalPinToBitMask(BTN_EN1);
  uint8_t port1 = digitalPinToPort(BTN_EN1);
  bm_b = digitalPinToBitMask(BTN_EN2);
  uint8_t port2 = digitalPinToPort(BTN_EN2);

  port_enc_a = portInputRegister(port1);
  port_enc_b = portInputRegister(port2);

  lcd_turn = 0;
  delta_time_start = micros();
}

static int g_loopctr = 0;

void loop() {
  // original code
  // read_input_pot();

  // modified to read from input pins. Note: assumes two input pins are on the
  // same port read_input_pot_direct_read_from_pins();

  // revised code
  read_input_pot_state_array();

  ++g_loopctr;
  if (g_loopctr > 1000) {
    g_loopctr = 0;
    consume_input();
  }
}

/**
 * This file is part of RotEnc-speedtest.
 *
 * RotEnc-speedtest is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RotEnc-speedtest is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with makelangelo-firmware.  If not, see <http://www.gnu.org/licenses/>.
 */
 
