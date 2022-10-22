#include <Arduino.h>
#include <BleGamepad.h>
#include <TickTwo.h>
#include <Preferences.h>

// config

const uint POLL_RATE = 1000; // micros, 1ms
const float DEADZONE = 0.20;

const uint SLEEP_TIMEOUT = 5; // minutes
const uint SLEEP_BUTTON_TIMEOUT = 5; // seconds

/* pins */

const byte AXIS_X_PIN = 34;
const byte AXIS_Y_PIN = 35;

const byte BUTTON_A_PIN = 19;
const byte BUTTON_B_PIN = 23;
const byte BUTTON_C_PIN = 18;
const byte BUTTON_D_PIN = 5;

const byte BUTTON_START_PIN = 15;
const byte BUTTON_SELECT_PIN = 13;

const byte LED_PIN = 22;

// #define DEBUG 1
// #define DEBUG_AXIS 1

// end config

const uint16_t BLE_AXIS_MIN = 0;
const uint16_t BLE_AXIS_MAX = 4095;

const uint axis_count = 2;

const byte AXIS_X_INDEX = 0;
const byte AXIS_Y_INDEX = 1;

const byte axes[] = { X_AXIS, Y_AXIS };
const char * axis_names[] = { "X", "Y" };
const byte axis_pins[] = { AXIS_X_PIN, AXIS_Y_PIN };
float axis_states[] = { 0.0, 0.0 };

const char * axis_min_names[] = { "x-axis-min", "y-axis-min" };
const char * axis_max_names[] = { "x-axis-max", "y-axis-max" };

float axis_min[] = { -1.0, -1.0 };
float axis_max[] = { 1.0, 1.0 };

const byte BUTTON_A_INDEX = 0;
const byte BUTTON_B_INDEX = 1;
const byte BUTTON_C_INDEX = 2;
const byte BUTTON_D_INDEX = 3;
const byte BUTTON_START_INDEX = 4;
const byte BUTTON_SELECT_INDEX = 5;
const byte BUTTON_MENU_INDEX = 6;

const byte INPUT_VIRTUAL = 255;
const byte BUTTON_TYPE_NORMAL = 0;
const byte BUTTON_TYPE_SPECIAL = 1;

const byte button_count = 6;
const byte buttons[] = { BUTTON_1, BUTTON_2, BUTTON_3, BUTTON_4, START_BUTTON, SELECT_BUTTON };
const char * button_names[] = { "A", "B", "C", "D", "START", "SELECT" };
byte button_states[] = { HIGH, HIGH, HIGH, HIGH, HIGH, HIGH };
const byte button_pins[] = { BUTTON_A_PIN, BUTTON_B_PIN, BUTTON_C_PIN, BUTTON_D_PIN, BUTTON_START_PIN, BUTTON_SELECT_PIN };
const byte button_types[] = { BUTTON_TYPE_NORMAL, BUTTON_TYPE_NORMAL, BUTTON_TYPE_NORMAL, BUTTON_TYPE_NORMAL, BUTTON_TYPE_SPECIAL, BUTTON_TYPE_SPECIAL };
const byte button_input_types[] = { INPUT, INPUT, INPUT, INPUT, INPUT_PULLUP, INPUT_PULLUP };

float axis_states_old[axis_count];
float axis_states_raw[axis_count];
byte button_states_old[button_count];

bool IS_CONNECTED = false;
byte LED_STATE = HIGH;

bool DIGITAL_MODE = false;
bool CALIBRATION_MODE = false;

BleGamepad ble_gamepad("Neo Geo Mini Gamepad", "SNK", 100);
BleGamepadConfiguration ble_gamepad_cfg;

void configure_ble_gamepad () {
  ble_gamepad_cfg.setAutoReport(false);
  ble_gamepad_cfg.setControllerType(CONTROLLER_TYPE_GAMEPAD);

  byte i = 0;

  byte normal_button_count = 0;
  byte special_button_count = 0;

  for (i = 0; i < button_count; i++) {
    if (button_input_types[i] != INPUT_VIRTUAL) pinMode(button_pins[i], button_input_types[i]);
    if (button_types[i] == BUTTON_TYPE_NORMAL) {
      normal_button_count++;
    } else if (button_types[i] == BUTTON_TYPE_SPECIAL) {
      special_button_count++;

      byte button = buttons[i];

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
  }

  #ifdef DEBUG
    Serial.print(String(normal_button_count) + " buttons, " + String(special_button_count) + " special buttons\n");
  #endif

  ble_gamepad_cfg.setButtonCount(normal_button_count);

  ble_gamepad_cfg.setWhichAxes(false, false, false, false, false, false, false, false);

  for (i = 0; i < axis_count; i++) {
    pinMode(axis_pins[i], INPUT);

    byte axis = axes[i];

    switch (axis) {
      case X_AXIS:
        ble_gamepad_cfg.setIncludeXAxis(true);
        break;

      case Y_AXIS:
        ble_gamepad_cfg.setIncludeYAxis(true);
        break;

      case Z_AXIS:
        ble_gamepad_cfg.setIncludeZAxis(true);
        break;

      case RX_AXIS:
        ble_gamepad_cfg.setIncludeRxAxis(true);
        break;

      case RY_AXIS:
        ble_gamepad_cfg.setIncludeRyAxis(true);
        break;

      case RZ_AXIS:
        ble_gamepad_cfg.setIncludeRzAxis(true);
        break;

      case SLIDER1:
        ble_gamepad_cfg.setIncludeSlider1(true);
        break;

      case SLIDER2:
        ble_gamepad_cfg.setIncludeSlider2(true);
        break;
    }
  }

  ble_gamepad_cfg.setAxesMin(BLE_AXIS_MIN); // 0 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal
  ble_gamepad_cfg.setAxesMax(BLE_AXIS_MAX); // 32767 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal

  ble_gamepad_cfg.setHatSwitchCount(1);
}

Preferences preferences;

void poll ();
void poll_calibrate ();
void toggle_led ();
void deep_sleep ();

TickTwo poll_interval(poll, POLL_RATE, 0, MICROS_MICROS);
TickTwo calibrate_interval(poll_calibrate, POLL_RATE, 0, MICROS_MICROS);
TickTwo led_interval(toggle_led, 1000, 0); // 1 second
TickTwo sleep_timeout(deep_sleep, SLEEP_TIMEOUT * 60000, 0);
TickTwo button_sleep_timeout(deep_sleep, SLEEP_BUTTON_TIMEOUT * 1000, 1);

void stop_all_timers () {
  poll_interval.stop();
  calibrate_interval.stop();
  led_interval.stop();
  sleep_timeout.stop();
  button_sleep_timeout.stop();
}

void update_all_timers () {
  poll_interval.update();
  calibrate_interval.update();
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
  const float input_abs = abs(input);
  if (input_abs < DEADZONE) {
    return 0.0;
  }

  const float sign = input / input_abs;

  return sign * map_range(input_abs, DEADZONE, 1.0, 0.0, 1.0);
}

void write_button (byte i, byte state) {
  const byte button = buttons[i];
  if (state == LOW) {
    if (button_types[i] == BUTTON_TYPE_SPECIAL) ble_gamepad.pressSpecialButton(button);
    else ble_gamepad.press(button);
  } else {
    if (button_types[i] == BUTTON_TYPE_SPECIAL) ble_gamepad.releaseSpecialButton(button);
    else ble_gamepad.release(button);
  }
}

void set_button_state (byte i, byte state) {
  button_states[i] = state;
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

  return compute_dpad_value(dpad_up, dpad_right, dpad_down, dpad_left);
}

void write_axis_dpad (float x_state, float y_state) {
  const byte dpad_value = adc(x_state, y_state);
  ble_gamepad.setHat(dpad_value);
}

void write_axis (byte i, float state) {
  const byte axis = axes[i];
  const int16_t state_ble = map_range(state, -1.0, 1.0, BLE_AXIS_MIN, BLE_AXIS_MAX);

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
      Serial.print(String(axis_names[i]) + " new max: " + String(state) + "\n");
    #endif
    axis_max[i] = state;
  }
  else if (state < axis_min[i]) {
    #ifdef DEBUG
      Serial.print(String(axis_names[i]) + " new min: " + String(state) + "\n");
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

  configure_ble_gamepad();

  pinMode(LED_PIN, OUTPUT);

  if (digitalRead(BUTTON_C_PIN) == LOW) {
    CALIBRATION_MODE = true;
    #ifdef DEBUG
      Serial.print("calibration mode\n");
    #endif
  } else if (digitalRead(BUTTON_A_PIN) == LOW) {
    DIGITAL_MODE = false;
  } else if (digitalRead(BUTTON_D_PIN) == LOW) {
    DIGITAL_MODE = true;
  } else {
    DIGITAL_MODE = preferences.getBool("digital-mode", false);
  }

  esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_START_PIN, LOW);

  reset_inputs();

  if (CALIBRATION_MODE) {

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
        Serial.print(String(axis_names[i]) +
          " min: " + String(axis_min[i]) +
          ", max: " + String(axis_max[i]) + "\n"
        );
      #endif
    }

    #ifdef DEBUG
      if (DIGITAL_MODE)
        Serial.print("digital mode\n");
      else
        Serial.print("analog mode\n");
    #endif

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
  if (DIGITAL_MODE) {
    write_axis_dpad(axis_states[AXIS_X_INDEX], axis_states[AXIS_Y_INDEX]);
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

void poll_calibrate () {
  byte i;

  for (i = 0; i < button_count; i++) {
    button_states_old[i] = button_states[i];
    if (button_input_types[i] == INPUT_VIRTUAL) continue;
    byte pin = button_pins[i];

    byte button_state = digitalRead(pin);
    set_button_state(i, button_state);
  }

  for (i = 0; i < axis_count; i++) {
    axis_states_old[i] = axis_states_raw[i];
    int16_t axis_state_raw = analogRead(axis_pins[i]);
    float axis_state_raw_mapped = map_range(axis_state_raw, 0.0, 4095.0, -1.0, 1.0);
    axis_states_raw[i] = axis_state_raw_mapped;
    set_axis_calibrate(i, axis_state_raw_mapped);
  }

  if (
    button_states[BUTTON_SELECT_INDEX] == LOW &&
    button_states_old[BUTTON_SELECT_INDEX] == HIGH
  ) {
    byte count = 0;

    for (i = 0; i < axis_count; i++) {
      preferences.putFloat(axis_min_names[i], axis_min[i]);
      preferences.putFloat(axis_max_names[i], axis_max[i]);

      #ifdef DEBUG
        Serial.print(String(axis_names[i]) + " min: " + String(axis_min[i]) + ", max: " + String(axis_max[i]) + "\n");
      #endif
    }
    #ifdef DEBUG
      Serial.print("calibration done\n");
    #endif

    delay(500);
    ESP.restart();
  }
}

void poll () {
  byte i;

  // BUTTONS

  bool changed = false;

  for (i = 0; i < button_count; i++) {
    button_states_old[i] = button_states[i];
    if (button_input_types[i] == INPUT_VIRTUAL) continue;
    byte button_state = digitalRead(button_pins[i]);
    set_button_state(i, button_state);
  }

  // FORCE SLEEP

  if (button_states[BUTTON_SELECT_INDEX] == LOW) {
    if (button_sleep_timeout.state() != RUNNING) button_sleep_timeout.start();
  } else {
    button_sleep_timeout.stop();
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
          String(button_names[i]) + ": " + button_states_old[i] +
          " -> " + button_states[i] + "\n"
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
          String(axis_names[i]) + ": " + String(axis_states_old[i]) +
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

  if (IS_CONNECTED) write_inputs();
}

void loop () {
  update_all_timers();
}
