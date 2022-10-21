// config

#define POLL_RATE 500 // 0.5ms
#define DEADZONE 0.20

#define SLEEP_TIMEOUT 5 // minutes
#define SLEEP_BUTTON_TIMEOUT 5 // seconds

/* calibration */

#define X_AXIS_MIN -0.8
#define X_AXIS_MAX 0.8
#define Y_AXIS_MIN -0.8
#define Y_AXIS_MAX 0.7

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

/* dev */

// #define DEBUG 1

#define BLE_AXIS_MIN 0
#define BLE_AXIS_MAX 2048

// end config

#include <Arduino.h>
#include <BleGamepad.h>
#include <TickTwo.h>

const uint axis_count = 2;
const uint button_count = 4;
const uint special_button_count = 3;

const byte X_AXIS_KEY = 0;
const byte Y_AXIS_KEY = 1;

const byte axes[] = { X_AXIS, Y_AXIS };
const byte axis_pins[] = { X_AXIS_PIN, Y_AXIS_PIN };
float axis_states[] = { 0.0, 0.0 };

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

bool is_connected = false;
byte led_state = HIGH;

const byte MENU_STATE_WAITING = 0;
const byte MENU_STATE_LOW = 1;
const byte MENU_STATE_HIGH = 2;
const byte MENU_STATE_EXPIRED = 3;
byte MENU_STATE = MENU_STATE_HIGH;

bool DIGITAL_MODE = false;

BleGamepad ble_gamepad("Neo Geo Mini Gamepad", "SNK", 100);
BleGamepadConfiguration ble_gamepad_cfg;

void poll ();
void toggle_led ();
void deep_sleep ();
void delayed_press ();

TickTwo poll_interval(poll, POLL_RATE, 0, MICROS_MICROS);
TickTwo led_interval(toggle_led, 1000, 0); // 1 second
TickTwo sleep_timeout(deep_sleep, SLEEP_TIMEOUT * 60000, 0);
TickTwo delayed_press_timeout(delayed_press, 120, 1); // 120ms
TickTwo button_sleep_timeout(deep_sleep, SLEEP_BUTTON_TIMEOUT * 1000, 1);

void stop_all_timers () {
  poll_interval.stop();
  delayed_press_timeout.stop();
  led_interval.stop();
  sleep_timeout.stop();
  button_sleep_timeout.stop();
}

void update_all_timers () {
  poll_interval.update();
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

void set_axis_state (byte i, float state) {
  byte axis = axes[i];
  float state_new_calibrated;

  switch (axis) {
    case X_AXIS:
      state_new_calibrated = map_range(state, X_AXIS_MIN, X_AXIS_MAX, -1.0, 1.0);
      break;

    case Y_AXIS:
      state_new_calibrated = map_range(state, Y_AXIS_MIN, Y_AXIS_MAX, -1.0, 1.0);
      break;
  }

  float dz_state = dz_scaled_radial(state_new_calibrated);
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

  if (digitalRead(BUTTON_D_PIN) == LOW) {
    DIGITAL_MODE = true;
    #ifdef DEBUG
      Serial.print("DIGITAL MODE SELECTED\n");
    #endif
  }

  ble_gamepad_cfg.setAutoReport(false);
  ble_gamepad_cfg.setControllerType(CONTROLLER_TYPE_GAMEPAD);
  ble_gamepad_cfg.setButtonCount(button_count);

  ble_gamepad_cfg.setAxesMin(BLE_AXIS_MIN); // 0 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal
  ble_gamepad_cfg.setAxesMax(BLE_AXIS_MAX); // 32767 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal

  ble_gamepad_cfg.setHatSwitchCount(1);
  ble_gamepad_cfg.setWhichAxes(true, true, false, false, false, false, false, false);

  ble_gamepad.begin(&ble_gamepad_cfg);

  reset_inputs();

  esp_sleep_enable_ext0_wakeup((gpio_num_t)START_BUTTON_PIN, LOW);

  poll_interval.start();
  led_interval.start();
  sleep_timeout.start();
}

void toggle_led () {
  digitalWrite(LED_PIN, led_state = !led_state);
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

void poll () {
  // Bluetooth not connected
  if (!ble_gamepad.isConnected()) {
    if (is_connected) { // was previously connected (is disconnected)
      #ifdef DEBUG
        Serial.print("bluetooth disconnected\n");
      #endif
      digitalWrite(LED_PIN, HIGH);
      led_interval.start();
      is_connected = false;
    }
    return;
  }

  byte i;

  // Bluetooth connected
  if (!is_connected) {
    #ifdef DEBUG
      Serial.print("bluetooth connected\n");
    #endif
    reset_inputs();

    led_interval.stop();
    digitalWrite(LED_PIN, led_state = LOW);

    is_connected = true;
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
    float axis_state_mapped = map_range(axis_state_raw, 0.0, 4095.0, -1.0, 1.0);
    axis_states_raw[i] = axis_state_mapped;

    set_axis_state(i, axis_state_mapped);
  }

  // change

  for (i = 0; i < button_count; i++) {
    if (button_states_old[i] != button_states[i]) {
      changed = true;
      #ifdef DEBUG
        Serial.print("button ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(button_states_old[i]);
        Serial.print(" -> ");
        Serial.print(button_states[i]);
        Serial.print("\n");
      #else
        break;
      #endif
    }
  }

  for (i = 0; i < special_button_count; i++) {
    if (special_button_states_old[i] != special_button_states[i]) {
      changed = true;
      #ifdef DEBUG
        Serial.print("special button ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(special_button_states_old[i]);
        Serial.print(" -> ");
        Serial.print(special_button_states[i]);
        Serial.print("\n");
      #else
        break;
      #endif
    }
  }

  for (i = 0; i < axis_count; i++) {
    if (axis_states_old[i] != axis_states[i]) {
      changed = true;
      #ifdef DEBUG
        Serial.print("axis ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(axis_states_old[i]);
        Serial.print(" -> ");
        Serial.print(axis_states[i]);
        Serial.print(" raw: ");
        Serial.print(axis_states_raw[i]);
        Serial.print("\n");
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
