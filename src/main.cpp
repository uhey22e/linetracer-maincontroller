#include <Arduino.h>
#include "FS.h"
#include "SPIFFS.h"

#define configLOGGING_MESSAGE_LEN 64
#define configNUM_PHOTO_REFLECTORS 5
#define configCALIBRATION_LENGTH 50

// photo reflectors
int pr_values[configNUM_PHOTO_REFLECTORS];
const uint8_t PR_PINS[] = {36, 39, 34, 35, 32};

const String pr_calibration_file = "/pr_calibration.dat";
float calibration_values[configNUM_PHOTO_REFLECTORS];

// motor control
#define MODE_PIN 2
#define MOTOR_L1_PIN 4
#define MOTOR_L2_PIN 16
#define MOTOR_R1_PIN 17
#define MOTOR_R2_PIN 5
#define MOTOR_L_PHASE_PIN MOTOR_L1_PIN
#define MOTOR_L_ENABLE_PIN MOTOR_L2_PIN
#define MOTOR_R_PHASE_PIN MOTOR_R1_PIN
#define MOTOR_R_ENABLE_PIN MOTOR_R2_PIN
#define MOTOR_L1_PWM_CH 0
#define MOTOR_L2_PWM_CH 1
#define MOTOR_R1_PWM_CH 2
#define MOTOR_R2_PWM_CH 3
#define MOTOR_L_ENABLE_PWM_CH MOTOR_L2_PWM_CH
#define MOTOR_R_ENABLE_PWM_CH MOTOR_R2_PWM_CH
const double MOTOR_PWM_FREQ_HZ = 40000;
const uint8_t MOTOR_PWM_RESOLUTION_BITS = 8;

// others
const uint8_t USER_SW_PIN = 19;

void load_params();
void calibration();
void setup_motor_with_phase_enable_mode();

void setup()
{
  Serial.begin(115200);
  Serial.println("Start");

  // calibration mode
  pinMode(USER_SW_PIN, INPUT_PULLUP);

  delay(1);
  if (digitalRead(USER_SW_PIN) == LOW)
  {
    calibration();
  }
  else
  {
    load_params();
  }

  // motor setup
  setup_motor_with_phase_enable_mode();

  Serial.println("Complete setup sequence");
}

void loop()
{
  float normalized[configNUM_PHOTO_REFLECTORS];
  for (int n_pr = 0; n_pr < configNUM_PHOTO_REFLECTORS; n_pr++)
  {
    int val = analogRead(PR_PINS[n_pr]);
    normalized[n_pr] = val / calibration_values[n_pr];
  }

  // motor control
  if (normalized[1] < 0.3) {
    ledcWrite(MOTOR_L_ENABLE_PWM_CH, 80);
    ledcWrite(MOTOR_R_ENABLE_PWM_CH, 150);
  } else if (normalized[configNUM_PHOTO_REFLECTORS - 2] < 0.3) {
    ledcWrite(MOTOR_L_ENABLE_PWM_CH, 150);
    ledcWrite(MOTOR_R_ENABLE_PWM_CH, 80);
  } else if (normalized[0] < 0.3) {
    ledcWrite(MOTOR_L_ENABLE_PWM_CH, 0);
    ledcWrite(MOTOR_R_ENABLE_PWM_CH, 150);
  } else if (normalized[configNUM_PHOTO_REFLECTORS - 1] < 0.3) {
    ledcWrite(MOTOR_L_ENABLE_PWM_CH, 150);
    ledcWrite(MOTOR_R_ENABLE_PWM_CH, 0);
  } else if (normalized[configNUM_PHOTO_REFLECTORS / 2] < 0.3) {
    ledcWrite(MOTOR_L_ENABLE_PWM_CH, 150);
    ledcWrite(MOTOR_R_ENABLE_PWM_CH, 150);
  }

  delay(5);
}

void calibration()
{
  int vals[configNUM_PHOTO_REFLECTORS][configCALIBRATION_LENGTH];

  Serial.println("Start calibration");

  for (int i = 0; i < configCALIBRATION_LENGTH; i++)
  {
    for (int n_pr = 0; n_pr < configNUM_PHOTO_REFLECTORS; n_pr++)
    {
      vals[n_pr][i] = analogRead(PR_PINS[n_pr]);
    }
    delay(10);
  }

  Serial.println("Average");

  float average[configNUM_PHOTO_REFLECTORS];
  for (int n_pr = 0; n_pr < configNUM_PHOTO_REFLECTORS; n_pr++)
  {
    for (int i = 0; i < configCALIBRATION_LENGTH; i++)
    {
      average[n_pr] += float(vals[n_pr][i] / 50.0);
    }
    Serial.println((int)average[n_pr]);
  }

  Serial.println("Saving calibration data to file...");

  if (!SPIFFS.begin(true)) {
    Serial.println("- Failed to mount file system");
    while (1);
  }

  File fptr = SPIFFS.open(pr_calibration_file, FILE_WRITE);
  if (!fptr) {
    Serial.println("- Failed to open file for writing");
    while (1);
  }
  for (int i = 0; i < sizeof(float) * configNUM_PHOTO_REFLECTORS; i++) {
    fptr.write(((uint8_t *)average)[i]);
  }
  fptr.close();

  Serial.println("End calibration");
  Serial.println("Press reset switch");
  while (1)
    ;
}

void setup_motor_with_phase_enable_mode()
{
  // motor setup
  pinMode(MODE_PIN, OUTPUT);
  digitalWrite(MODE_PIN, HIGH);

  pinMode(MOTOR_L_PHASE_PIN, OUTPUT);
  ledcSetup(MOTOR_L_ENABLE_PWM_CH, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS);
  ledcAttachPin(MOTOR_L_ENABLE_PIN, MOTOR_L_ENABLE_PWM_CH);
  pinMode(MOTOR_R_PHASE_PIN, OUTPUT);
  ledcSetup(MOTOR_R_ENABLE_PWM_CH, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS);
  ledcAttachPin(MOTOR_R_ENABLE_PIN, MOTOR_R_ENABLE_PWM_CH);
}

void load_params()
{
  if (!SPIFFS.begin(true))
  {
    Serial.println("SPIFFS Mount failed");
    while (1)
      ;
  }
  Serial.println("SPIFFS Mount success");

  File fptr = SPIFFS.open(pr_calibration_file, FILE_READ);
  if (!fptr)
  {
    Serial.println("- Failed to open file for reading");
    while (1)
      ;
  }

  // load calibration data
  for (int i = 0; i < sizeof(float) * configNUM_PHOTO_REFLECTORS; i++)
  {
    fptr.read((uint8_t *)calibration_values, sizeof(float) * configNUM_PHOTO_REFLECTORS);
  }
  fptr.close();

  Serial.println("Success");
  for (int i = 0; i < configNUM_PHOTO_REFLECTORS; i++)
  {
    Serial.println((int)calibration_values[i]);
  }

  SPIFFS.end();
}