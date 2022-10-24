#include <Arduino.h>
#include <TickTwo.h>

#include "joy_util.h"

/* config */

// #define JOY_DEBUG 1

const uint REPORT_RATE = 1000; // micros, 1ms

const float DEADZONE = 0.20;

const uint SLEEP_TIMEOUT = 5; // minutes
const uint SLEEP_BUTTON_TIMEOUT = 5; // seconds

// pins

const byte AXIS_X_PIN = 34;
const byte AXIS_Y_PIN = 35;

const byte BUTTON_A_PIN = 19; // errnoeously marked "B" in the mini controller
const byte BUTTON_B_PIN = 23; // errnoeously marked "D"
const byte BUTTON_C_PIN = 18; // errnoeously marked "A"
const byte BUTTON_D_PIN = 5;  // errnoeously marked "C"

const byte BUTTON_START_PIN = 15;
const byte BUTTON_SELECT_PIN = 13;

const byte LED_PIN = 22;

/* end config */

JoyUtil joy("Neogeo Mini Gamepad", "SNK", DEADZONE);

bool IS_BLE_CONNECTED = false;
bool IS_CLASSIC_CONNECTED = false;

byte LED_STATE = HIGH;

bool CALIBRATION_MODE = false;
bool DIGITAL_MODE = false;

void report ();
void report_calibrate ();
void toggle_led ();
void deep_sleep ();
void ble_conn_check ();

#ifdef JOY_DEBUG
void debug_common ();
TickTwo debug_interval(debug_common, 1000, 0); // 1 second
#endif

TickTwo report_interval(report, REPORT_RATE, 0, MICROS_MICROS);
TickTwo ble_conn_check_interval(ble_conn_check, 1000, 0); // 1 second
TickTwo calibrate_interval(report_calibrate, REPORT_RATE, 0, MICROS_MICROS);
TickTwo nc_led_interval(toggle_led, 1000, 0); // 1 second
TickTwo sleep_timeout(deep_sleep, SLEEP_TIMEOUT * 60000, 0);
TickTwo button_sleep_timeout(deep_sleep, SLEEP_BUTTON_TIMEOUT * 1000, 1);

void stop_all_timers () {
  report_interval.stop();
  ble_conn_check_interval.stop();
  calibrate_interval.stop();
  nc_led_interval.stop();
  sleep_timeout.stop();
  button_sleep_timeout.stop();
}

void update_core1_timers () {
  report_interval.update();
  ble_conn_check_interval.update();
  calibrate_interval.update();
  nc_led_interval.update();
  sleep_timeout.update();
  button_sleep_timeout.update();

  #ifdef JOY_DEBUG
  debug_interval.update();
  #endif
}

float map_axis_value (uint16_t axis_state_raw) {
  return JoyUtil::map_range(axis_state_raw, 0.0, 4095.0, -1.0, 1.0);
}

void read_dpad () {
  joy.set_dpad_analog_state(
    JoyUtil::AXIS_LX, JoyUtil::AXIS_LY,
    map_axis_value(analogRead(AXIS_X_PIN)), map_axis_value(analogRead(AXIS_Y_PIN))
  );
}

void read_buttons () {
  joy.set_button_state(JoyUtil::BUTTON_A, digitalRead(BUTTON_A_PIN));
  joy.set_button_state(JoyUtil::BUTTON_B, digitalRead(BUTTON_B_PIN));
  joy.set_button_state(JoyUtil::BUTTON_X, digitalRead(BUTTON_C_PIN));
  joy.set_button_state(JoyUtil::BUTTON_Y, digitalRead(BUTTON_D_PIN));

  joy.set_button_state(JoyUtil::BUTTON_START, digitalRead(BUTTON_START_PIN));
  joy.set_button_state(JoyUtil::BUTTON_SELECT, digitalRead(BUTTON_SELECT_PIN));
}

void read_axes () {
  joy.set_axis_state(JoyUtil::AXIS_LX, map_axis_value(analogRead(AXIS_X_PIN)));
  joy.set_axis_state(JoyUtil::AXIS_LY, map_axis_value(analogRead(AXIS_Y_PIN)));
}

#ifdef JOY_DEBUG
uint32_t report_runs = 0;
#endif

void loop () {
  update_core1_timers();
}

void start () {
  if (digitalRead(BUTTON_C_PIN) == LOW) {
    CALIBRATION_MODE = true;
    #ifdef JOY_DEBUG
    Serial.print("calibration mode\n");
    #endif
  }

  if (CALIBRATION_MODE) {
    #ifdef JOY_DEBUG
    Serial.print("calibration mode\n");
    Serial.print("put all axes in neutral position then press select\n");
    #endif

    calibrate_interval.start();

  } else {

    if (digitalRead(BUTTON_A_PIN) == LOW) {
      DIGITAL_MODE = false;
    } else if (digitalRead(BUTTON_D_PIN) == LOW) {
      DIGITAL_MODE = true;
    } else {
      DIGITAL_MODE = joy.preferences.getBool("digital-mode", false);
    }

    #ifdef JOY_DEBUG
    if (DIGITAL_MODE)
      Serial.print("digital mode\n");
    else
      Serial.print("analog mode\n");
    #endif

    joy.prefs_read();

    #ifdef JOY_DEBUG
    for (byte axis = 0; axis < JoyUtil::AXIS_COUNT; axis++) {
      Serial.print(String(joy.axis_names[axis]) +
        " min: " + String(joy.get_axis_min(axis)) +
        ", mid: " + String(joy.get_axis_mid(axis)) +
        ", max: " + String(joy.get_axis_max(axis)) + "\n"
      );
    }
    #endif

    joy.connect();

    report_interval.start();
    ble_conn_check_interval.start();
    nc_led_interval.start();
    sleep_timeout.start();
  }
}

void setup () {
  #ifdef JOY_DEBUG
  Serial.begin(115200);
  Serial.print("setup\n");

  debug_interval.start();
  #endif

  joy.prefs_init();

  pinMode(LED_PIN, OUTPUT);

  pinMode(BUTTON_A_PIN, INPUT);
  pinMode(BUTTON_B_PIN, INPUT);
  pinMode(BUTTON_C_PIN, INPUT);
  pinMode(BUTTON_D_PIN, INPUT);

  pinMode(BUTTON_START_PIN, INPUT_PULLUP);
  pinMode(BUTTON_SELECT_PIN, INPUT_PULLUP);

  pinMode(AXIS_X_PIN, INPUT);
  pinMode(AXIS_Y_PIN, INPUT);

  esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_START_PIN, LOW);

  start();
}

void toggle_led () {
  digitalWrite(LED_PIN, LED_STATE = !LED_STATE);
}

void deep_sleep () {
  stop_all_timers();

  #ifdef JOY_DEBUG
  Serial.print("entering sleep mode\n");
  #endif

  joy.raise_inputs();
  joy.report();

  joy.preferences.putBool("digital-mode", DIGITAL_MODE);

  // delay needed otherwise buttons remain pressed briefly on disconnect (?)
  delay(120);
  esp_deep_sleep_start();
}

float axes_min[2] = { 0.0, 0.0 };
float axes_max[2] = { 0.0, 0.0 };

byte CALIBRATION_STATE = 0;
byte special_btn_state_old = HIGH;

void report_calibrate () {
  read_buttons();
  read_axes();

  byte special_btn_state_new = joy.get_button_state(JoyUtil::BUTTON_SELECT);
  bool special_pressed = special_btn_state_old == HIGH && special_btn_state_new == LOW;
  special_btn_state_old = special_btn_state_new;

  if (CALIBRATION_STATE == 1) {
    for (byte axis = 0; axis < 2; axis++) { // first 2 axes, LX and LY
      float state = joy.get_axis_state_raw(axis);
      if (state < axes_min[axis]) {
        axes_min[axis] = state;
        joy.set_axis_min(axis, state);
      } else if (state > axes_max[axis]) {
        axes_max[axis] = state;
        joy.set_axis_max(axis, state);
      }
    }
  } else if (CALIBRATION_STATE == 0) {
    for (byte axis = 0; axis < 2; axis++) { // first 2 axes, LX and LY
      joy.set_axis_mid(axis, joy.get_axis_state_raw(axis));
    }
  }

  if (special_pressed) {
    if (CALIBRATION_STATE == 0) {
      CALIBRATION_STATE = 1;
      #ifdef JOY_DEBUG
      Serial.print("now move all axes full range then press select\n");
      #endif
    } else {
      joy.prefs_write();

      #ifdef JOY_DEBUG
      for (byte axis = 0; axis < JoyUtil::AXIS_COUNT; axis++) {
        Serial.print(
          String(joy.axis_names[axis]) +
          " min: " + String(joy.get_axis_min(axis)) +
          ", mid: " + String(joy.get_axis_mid(axis)) +
          ", max: " + String(joy.get_axis_max(axis)) + "\n"
        );
      }

      Serial.print("calibration done\n");
      #endif

      calibrate_interval.stop();

      delay(500);
      ESP.restart();
    }
  }
}

void ble_conn_check () {
  if (!joy.is_connected()) { // Bluetooth not connected
    if (IS_BLE_CONNECTED) { // was previously connected (is disconnected)
      #ifdef JOY_DEBUG
      Serial.print("bluetooth disconnected\n");
      #endif
      digitalWrite(LED_PIN, HIGH);
      nc_led_interval.start();
      IS_BLE_CONNECTED = false;
    }
    return;
  } else { // Bluetooth connected
    if (!IS_BLE_CONNECTED) {
      #ifdef JOY_DEBUG
      Serial.print("bluetooth connected\n");
      #endif

      joy.raise_inputs();

      nc_led_interval.stop();
      digitalWrite(LED_PIN, LED_STATE = LOW);

      IS_BLE_CONNECTED = true;
    }
  }
}

void report () {
  #ifdef JOY_DEBUG
  report_runs++;
  #endif

  read_buttons();

  if (DIGITAL_MODE) {
    read_dpad();
  } else {
    read_axes();
  }

  // FORCE SLEEP

  if (joy.get_button_state(JoyUtil::BUTTON_SELECT) == LOW) {
    if (button_sleep_timeout.state() != RUNNING) button_sleep_timeout.start();
  } else {
    button_sleep_timeout.stop();
  }

  if (joy.is_any_pressed()) {
    sleep_timeout.stop();
    sleep_timeout.start();
  }

  joy.report();
}

#ifdef JOY_DEBUG
void debug_common () {
  for (byte btn = 0; btn < JoyUtil::BUTTON_COUNT; btn++) {
    const std::string spacer = (btn < JoyUtil::BUTTON_COUNT - 1) ? ", " : "";
    Serial.print(
      String(joy.button_names[btn]) +
      ": " + String(joy.get_button_state(btn)) + spacer.c_str()
    );
  }
  Serial.print(" :: ");
  for (byte axis = 0; axis < JoyUtil::AXIS_COUNT; axis++) {
    const std::string spacer = (axis < JoyUtil::AXIS_COUNT - 1) ? ", " : "";
    Serial.print(
      String(joy.axis_names[axis]) +
      ": " + String(joy.get_axis_state(axis)) + "(" + String(joy.get_axis_state_raw(axis)) + ")" + spacer.c_str()
    );
  }
  Serial.print(":: DP: " + String(joy.get_dpad_state()) + "\n");
  Serial.print("report runs: " + String(report_runs) + "\n");

  report_runs = 0;
}
#endif
