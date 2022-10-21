// config

#define POLL_RATE 500 // 0.5ms
#define DEADZONE 0.20

#define SLEEP_TIMEOUT 5 // minutes
#define SLEEP_BUTTON_TIMEOUT 5 // seconds

/* pins */

#define X_AXIS_PIN 34
#define Y_AXIS_PIN 35
#define BUTTON_A_PIN 19
#define BUTTON_B_PIN 23
#define BUTTON_C_PIN 18
#define BUTTON_D_PIN 5

#define START_BUTTON_PIN 15
#define SELECT_BUTTON_PIN 13

#define LED_PIN 22

/* ble */

#define BLE_AXIS_MIN 0
#define BLE_AXIS_MAX 2048

#define DEBUG 1

// end config

#include <Arduino.h>
#include <BleGamepad.h>
#include <TickTwo.h>
#include <Preferences.h>

const uint axis_count = 2;
const uint button_count = 4;
const uint special_button_count = 3;

const byte X_AXIS_KEY = 0;
const byte Y_AXIS_KEY = 1;

const byte axes[] = { X_AXIS, Y_AXIS };
const byte axis_pins[] = { X_AXIS_PIN, Y_AXIS_PIN };
float axis_states[] = { 0.0, 0.0 };

const char * axis_min_names[] = { "x-axis-min", "y-axis-min" };
const char * axis_max_names[] = { "x-axis-max", "y-axis-max" };

float axis_min[] = { -1.0, -1.0 };
float axis_max[] = { 1.0, 1.0 };

const byte A_KEY = 0;
const byte B_KEY = 1;
const byte C_KEY = 2;
const byte D_KEY = 3;

const byte buttons[] = { BUTTON_1, BUTTON_2, BUTTON_3, BUTTON_4 };
const byte button_pins[] = { BUTTON_A_PIN, BUTTON_B_PIN, BUTTON_C_PIN, BUTTON_D_PIN };
byte button_states[] = { HIGH, HIGH, HIGH, HIGH };

const byte START_KEY = 0;
const byte SELECT_KEY = 1;
const byte MENU_KEY = 2;

const byte special_buttons[] = { START_BUTTON, SELECT_BUTTON, HOME_BUTTON };
const byte special_button_pins[] = { START_BUTTON_PIN, SELECT_BUTTON_PIN, 0 };
byte special_button_states[] = { HIGH, HIGH, HIGH };

float axis_states_old[axis_count];
float axis_states_raw[axis_count];
byte button_states_old[button_count];
byte special_button_states_old[special_button_count];

bool IS_CONNECTED = false;
byte LED_STATE = HIGH;

const byte MENU_STATE_WAITING = 0;
const byte MENU_STATE_LOW = 1;
const byte MENU_STATE_HIGH = 2;
const byte MENU_STATE_EXPIRED = 3;
byte MENU_STATE = MENU_STATE_HIGH;

bool DIGITAL_MODE = false;
bool CALIBRATION_MODE = false;

BleGamepad ble_gamepad("Neo Geo Mini Gamepad", "SNK", 100);
BleGamepadConfiguration ble_gamepad_cfg;

Preferences preferences;

void poll ();
void poll_calibrate ();
void toggle_led ();
void deep_sleep ();
void delayed_press ();

TickTwo poll_interval(poll, POLL_RATE, 0, MICROS_MICROS);
TickTwo calibrate_interval(poll_calibrate, POLL_RATE, 0, MICROS_MICROS);
TickTwo led_interval(toggle_led, 1000, 0); // 1 second
TickTwo sleep_timeout(deep_sleep, SLEEP_TIMEOUT * 60000, 0);
TickTwo delayed_press_timeout(delayed_press, 120, 1); // 120ms
TickTwo button_sleep_timeout(deep_sleep, SLEEP_BUTTON_TIMEOUT * 1000, 1);

void stop_all_timers () {
  poll_interval.stop();
  calibrate_interval.stop();
  delayed_press_timeout.stop();
  led_interval.stop();
  sleep_timeout.stop();
  button_sleep_timeout.stop();
}

void update_all_timers () {
  poll_interval.update();
  calibrate_interval.update();
  delayed_press_timeout.update();
  led_interval.update();
  sleep_timeout.update();
  button_sleep_timeout.update();
}

// https://github.com/Minimuino/thumbstick-deadzones

float map_range (float value, float old_min, float old_max, float new_min, float new_max) {
  if (value < old_min) value = old_min;
  if (value > old_max) value = old_max;

  return (new_min + (new_max - new_min) * (value - old_min) / (old_max - old_min));
}

// https://github.com/Minimuino/thumbstick-deadzones

float dz_scaled_radial (float input) {
  float input_abs = abs(input);
  if (input_abs < DEADZONE) {
    return 0.0;
  }

  float sign = input / input_abs;

  return sign * map_range(input_abs, DEADZONE, 1.0, 0.0, 1.0);
}

void delayed_press () {
  MENU_STATE = MENU_STATE_EXPIRED;
}

void write_button (byte i, byte state) {
  byte button = buttons[i];
  if (state == LOW) {
    ble_gamepad.press(button);
  } else {
    ble_gamepad.release(button);
  }
}

void set_button_state (byte i, byte state) {
  button_states[i] = state;
}

void write_special_button (byte i, byte state) {
  byte button = special_buttons[i];

  if (state == LOW) {
    ble_gamepad.pressSpecialButton(button);
  } else {
    ble_gamepad.releaseSpecialButton(button);
  }
}

void set_special_button_state (byte i, byte state) {
  special_button_states[i] = state;
}

byte compute_dpad_value (bool dpad_up, bool dpad_right, bool dpad_down, bool dpad_left) {
  byte dpad_value = DPAD_CENTERED;

  if (dpad_up) {
    if (dpad_right) {
      dpad_value = DPAD_UP_RIGHT;
    } else if (dpad_left) {
      dpad_value = DPAD_UP_LEFT;
    } else {
      dpad_value = DPAD_UP;
    }
  } else if (dpad_down) {
    if (dpad_right) {
      dpad_value = DPAD_DOWN_RIGHT;
    } else if (dpad_left) {
      dpad_value = DPAD_DOWN_LEFT;
    } else {
      dpad_value = DPAD_DOWN;
    }
  } else if (dpad_left) {
    dpad_value = DPAD_LEFT;
  } else if (dpad_right) {
    dpad_value = DPAD_RIGHT;
  }

  return dpad_value;
}

// https://gamingprojects.wordpress.com/2017/08/04/converting-analog-joystick-to-digital-joystick-signals/

const float slope = 0.414214;

byte adc (float x, float y) {
  bool dpad_up = false;
  bool dpad_down = false;
  bool dpad_left = false;
  bool dpad_right = false;

  // squared deadzone?

  const float slope_y = slope * y;
  const float slope_x = slope * x;

  if (x > 0.0) {
    if (x > slope_y) dpad_right = true;
  } else if (x < 0.0) {
    if (x < slope_y) dpad_left = true;
  }

  if (y > 0.0) {
    if (y > slope_x) dpad_down = true;
  } else if (y < 0.0) {
    if (y < slope_x) dpad_up = true;
  }

  return compute_dpad_value(dpad_up, dpad_left, dpad_down, dpad_right);
}

void write_axis_dpad (float x_state, float y_state) {
  byte dpad_value = adc(x_state, y_state);
  ble_gamepad.setHat(dpad_value);
}

void write_axis (byte i, float state) {
  byte axis = axes[i];
  int16_t state_ble = map_range(state, -1.0, 1.0, BLE_AXIS_MIN, BLE_AXIS_MAX);

  switch (axis) {
    case X_AXIS:
      ble_gamepad.setX(state_ble);
      break;

    case Y_AXIS:
      ble_gamepad.setY(state_ble);
      break;
  }
}

void set_axis_calibrate (byte i, float state) {
  if (state > axis_max[i]) {
    #ifdef DEBUG
      Serial.print("axis " + String(i) + " new max: " + String(state) + "\n");
    #endif
    axis_max[i] = state;
  }
  else if (state < axis_min[i]) {
    #ifdef DEBUG
      Serial.print("axis " + String(i) + " new min: " + String(state) + "\n");
    #endif
    axis_min[i] = state;
  }
}

void set_axis_state (byte i, float state) {
  float state_calibrated;

  state_calibrated = map_range(state, axis_min[i], axis_max[i], -1.0, 1.0);

  float dz_state = dz_scaled_radial(state_calibrated);
  axis_states[i] = dz_state;
}

void reset_inputs () {
  byte i;
  for (i = 0; i < button_count; i++) {
    set_button_state(i, HIGH);
  }
  for (i = 0; i < special_button_count; i++) {
    set_special_button_state(i, HIGH);
  }
  for (i = 0; i < axis_count; i++) {
    set_axis_state(i, 0.0);
  }
}

void setup () {
  #ifdef DEBUG
    Serial.begin(115200);
    Serial.print("setup\n");
  #endif

  preferences.begin("neogeo-ble", false);

  byte i;

  for (i = 0; i < button_count; i++) {
    pinMode(button_pins[i], INPUT);
  }

  for (i = 0; i < special_button_count; i++) {
    byte pin = special_button_pins[i];
    if (pin != 0) pinMode(special_button_pins[i], INPUT_PULLUP);
    byte button = special_buttons[i];
    switch (button) {
      case START_BUTTON:
        ble_gamepad_cfg.setIncludeStart(true);
        break;
      case SELECT_BUTTON:
        ble_gamepad_cfg.setIncludeSelect(true);
        break;
      case HOME_BUTTON:
        ble_gamepad_cfg.setIncludeHome(true);
        break;
      case BACK_BUTTON:
        ble_gamepad_cfg.setIncludeBack(true);
        break;
      case MENU_BUTTON:
        ble_gamepad_cfg.setIncludeMenu(true);
        break;
    }
  }

  for (i = 0; i < axis_count; i++) {
    pinMode(axis_pins[i], INPUT);
  }

  pinMode(LED_PIN, OUTPUT);

  if (digitalRead(BUTTON_A_PIN) == LOW) {
    DIGITAL_MODE = false;
  } else if (digitalRead(BUTTON_D_PIN) == LOW) {
    DIGITAL_MODE = true;
  } else {
    DIGITAL_MODE = preferences.getBool("digital-mode", false);
  }

  #ifdef DEBUG
    if (DIGITAL_MODE)
      Serial.print("digital mode\n");
    else
      Serial.print("analog mode\n");
  #endif

  if (digitalRead(BUTTON_C_PIN) == LOW) {
    CALIBRATION_MODE = true;
    #ifdef DEBUG
      Serial.print("CALIBRATION MODE\n");
    #endif
  }

  ble_gamepad_cfg.setAutoReport(false);
  ble_gamepad_cfg.setControllerType(CONTROLLER_TYPE_GAMEPAD);
  ble_gamepad_cfg.setButtonCount(button_count);

  ble_gamepad_cfg.setAxesMin(BLE_AXIS_MIN); // 0 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal
  ble_gamepad_cfg.setAxesMax(BLE_AXIS_MAX); // 32767 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal

  ble_gamepad_cfg.setHatSwitchCount(1);
  ble_gamepad_cfg.setWhichAxes(true, true, false, false, false, false, false, false);

  esp_sleep_enable_ext0_wakeup((gpio_num_t)START_BUTTON_PIN, LOW);

  reset_inputs();

  if (CALIBRATION_MODE) {

    preferences.clear();

    for (i = 0; i < axis_count; i++) {
      axis_min[i] = 0.0;
      axis_max[i] = 0.0;
    }

    calibrate_interval.start();

  } else {

    for (i = 0; i < axis_count; i++) {
      axis_min[i] = preferences.getFloat(axis_min_names[i], -1.0);
      axis_max[i] = preferences.getFloat(axis_max_names[i], 1.0);

      #ifdef DEBUG
        Serial.print("axis " + String(i) +
          " min: " + String(axis_min[i]) +
          ", max: " + String(axis_max[i]) + "\n"
        );
      #endif
    }

    ble_gamepad.begin(&ble_gamepad_cfg);

    poll_interval.start();
    led_interval.start();
    sleep_timeout.start();
  }
}

void toggle_led () {
  digitalWrite(LED_PIN, LED_STATE = !LED_STATE);
}

void write_inputs () {
  byte i;

  for (i = 0; i < button_count; i++) {
    write_button(i, button_states[i]);
  }
  for (i = 0; i < special_button_count; i++) {
    write_special_button(i, special_button_states[i]);
  }
  if (DIGITAL_MODE) {
    write_axis_dpad(axis_states[X_AXIS_KEY], axis_states[Y_AXIS_KEY]);
    for (i = 0; i < axis_count; i++) {
      write_axis(i, 0.0);
    }
  } else {
    write_axis_dpad(0.0, 0.0);
    for (i = 0; i < axis_count; i++) {
      write_axis(i, axis_states[i]);
    }
  }

  ble_gamepad.sendReport();
}

void deep_sleep () {
  stop_all_timers();

  #ifdef DEBUG
    Serial.print("entering sleep mode\n");
  #endif
  reset_inputs();
  write_inputs();

  preferences.putBool("digital-mode", DIGITAL_MODE);

  // delay needed otherwise buttons remain pressed briefly on disconnect (?)
  delay(120);
  esp_deep_sleep_start();
}

void handle_soft_menu () {
  byte start_state = special_button_states[START_KEY];
  byte select_state = special_button_states[SELECT_KEY];

  if (MENU_STATE == MENU_STATE_LOW) {
    if (start_state == HIGH && select_state == HIGH) {
      MENU_STATE = MENU_STATE_HIGH;
      set_special_button_state(MENU_KEY, HIGH);
    }
  }

  if (MENU_STATE == MENU_STATE_HIGH) {
    if (start_state == LOW || select_state == LOW) {
      MENU_STATE = MENU_STATE_WAITING;
      delayed_press_timeout.start();
    }
  }

  if (MENU_STATE == MENU_STATE_WAITING) {
    if (start_state == LOW && select_state == LOW) {
      MENU_STATE = MENU_STATE_LOW;
      delayed_press_timeout.stop();
      set_special_button_state(MENU_KEY, LOW);
    }
  }

  if (MENU_STATE == MENU_STATE_WAITING || MENU_STATE == MENU_STATE_LOW) {
    set_special_button_state(START_KEY, HIGH);
    set_special_button_state(SELECT_KEY, HIGH);
  }

  if (MENU_STATE == MENU_STATE_EXPIRED) {
    if (start_state == HIGH && select_state == HIGH) {
      MENU_STATE = MENU_STATE_HIGH;
    }
  }
}

void poll_calibrate () {
  byte i;

  for (i = 0; i < special_button_count; i++) {
    special_button_states_old[i] = special_button_states[i];
    byte pin = special_button_pins[i];
    if (pin == 0) continue;

    byte button_state = digitalRead(pin);
    set_special_button_state(i, button_state);
  }

  for (i = 0; i < axis_count; i++) {
    axis_states_old[i] = axis_states_raw[i];
    int16_t axis_state_raw = analogRead(axis_pins[i]);
    float axis_state_raw_mapped = map_range(axis_state_raw, 0.0, 4095.0, -1.0, 1.0);
    axis_states_raw[i] = axis_state_raw_mapped;
    set_axis_calibrate(i, axis_state_raw_mapped);
  }

  if (
    special_button_states[SELECT_KEY] == LOW &&
    special_button_states_old[SELECT_KEY] == HIGH
  ) {
    byte count = 0;

    for (i = 0; i < axis_count; i++) {
      preferences.putFloat(axis_min_names[i], axis_min[i]);
      preferences.putFloat(axis_max_names[i], axis_max[i]);

      #ifdef DEBUG
        Serial.print("axis " + String(i) + " min: " + String(axis_min[i]) + ", max: " + String(axis_max[i]) + "\n");
      #endif
    }
    #ifdef DEBUG
      Serial.print("calibration done");
    #endif

    delay(500);
    ESP.restart();
  }
}

void poll () {
  // Bluetooth not connected
  if (!ble_gamepad.isConnected()) {
    if (IS_CONNECTED) { // was previously connected (is disconnected)
      #ifdef DEBUG
        Serial.print("bluetooth disconnected\n");
      #endif
      digitalWrite(LED_PIN, HIGH);
      led_interval.start();
      IS_CONNECTED = false;
    }
    return;
  }

  byte i;

  // Bluetooth connected
  if (!IS_CONNECTED) {
    #ifdef DEBUG
      Serial.print("bluetooth connected\n");
    #endif
    reset_inputs();

    led_interval.stop();
    digitalWrite(LED_PIN, LED_STATE = LOW);

    IS_CONNECTED = true;
  }

  // BUTTONS

  bool changed = false;

  for (i = 0; i < button_count; i++) {
    button_states_old[i] = button_states[i];
    byte button_state = digitalRead(button_pins[i]);
    set_button_state(i, button_state);
  }

  // START, SELECT

  for (i = 0; i < special_button_count; i++) {
    special_button_states_old[i] = special_button_states[i];
    byte pin = special_button_pins[i];
    if (pin == 0) continue;

    byte button_state = digitalRead(pin);
    set_special_button_state(i, button_state);
  }

  // MENU

  handle_soft_menu();

  // FORCE SLEEP

  if (special_button_states[SELECT_KEY] == LOW) {
    if (button_sleep_timeout.state() != RUNNING) button_sleep_timeout.start();
  } else {
    button_sleep_timeout.stop();
  }

  // TOGGLE DIGITAL MODE
  if (
    special_button_states[SELECT_KEY] == LOW &&
    button_states[D_KEY] == LOW &&
    (
      button_states_old[D_KEY] == HIGH ||
      special_button_states_old[SELECT_KEY] == HIGH
    )
  ) {
    DIGITAL_MODE = !DIGITAL_MODE;
    #ifdef DEBUG
      if (DIGITAL_MODE) Serial.print("digital mode enabled\n");
      else Serial.print("digital mode disabled\n");
    #endif
  }

  // AXES

  for (i = 0; i < axis_count; i++) {
    axis_states_old[i] = axis_states[i];
    int16_t axis_state_raw = analogRead(axis_pins[i]);
    float axis_state_raw_mapped = map_range(axis_state_raw, 0.0, 4095.0, -1.0, 1.0);
    axis_states_raw[i] = axis_state_raw_mapped;

    set_axis_state(i, axis_state_raw_mapped);
  }

  // change

  for (i = 0; i < button_count; i++) {
    if (button_states_old[i] != button_states[i]) {
      changed = true;
      #ifdef DEBUG
        Serial.print(
          "button " + String(i) + ": " + button_states_old[i] +
          " -> " + button_states[i] + "\n"
        );
      #else
        break;
      #endif
    }
  }

  for (i = 0; i < special_button_count; i++) {
    if (special_button_states_old[i] != special_button_states[i]) {
      changed = true;
      #ifdef DEBUG
        Serial.print(
          "special button " + String(i) + ": " + String(special_button_states_old[i]) +
          " -> " + String(special_button_states[i]) + "\n"
        );
      #else
        break;
      #endif
    }
  }

  for (i = 0; i < axis_count; i++) {
    if (axis_states_old[i] != axis_states[i]) {
      changed = true;
      #ifdef DEBUG_AXIS
        Serial.print(
          "axis " + String(i) + ": " + String(axis_states_old[i]) +
          " -> " + String(axis_states[i]) +
          ", raw: " + String(axis_states_raw[i]) + "\n"
        );
      #else
        break;
      #endif
    }
  }

  if (changed) {
    sleep_timeout.stop();
    sleep_timeout.start();
  }

  // WRITE
  write_inputs();
}

void loop () {
  update_all_timers();
}
