#include <Arduino.h>

#include "init.h"
#include "config.h"
#include "board_mapping.h"
#include "interfaces.h"
#include "secrets.h"

// ***************  WIFI  *************** //
#if WIFI_ACTIVE == 1
#include <WiFi.h>
#include <WiFiUdp.h>
#endif

#if EAP_ACTIVE == 1
#include "esp_wpa2.h"
#endif

#if OTA_ACTIVE == 1
#include <ArduinoOTA.h>
#include "esp_wpa2.h"
#endif

#if WEBSERIAL_ACTIVE == 1
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
#endif

// ***************  LED  *************** //

#if NEOPIXEL_ACTIVE == 1
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
int pixel = 0;
#endif

// ***************  I2C  *************** //
#if I2C_ACTIVE == 1
#include <Wire.h>
#endif

// ***************  SPI  *************** //

#if SPI_ACTIVE == 1
#include <SPI.h>
#endif

#if SPI_ACTIVE == 1 && MXC6655_ACTIVE == 1
#include "MXC6655.h"
MXC6655 accel;
#endif

// ***************  MOTEURS  *************** //

#if MOTORS_ACTIVE == 1
#include <StepperNB.h>
StepperNB moteur_droit(GPIO_DIR_D, GPIO_STEP_D, GPIO_MS1_D, GPIO_MS2_D, GPIO_MS3_D, 200, true);
hw_timer_t *Timer3_Cfg = NULL; // Moteur Droit
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Fonction d'interruption pour envoyer les impulsions au moteur droit
void IRAM_ATTR Timer3_MoteurD_ISR()
{
  portENTER_CRITICAL(&timerMux);
  noInterrupts();

  moteur_droit.computeSpeed();

  uint64_t delay = moteur_droit.getTimerPeriod();
  if (delay > 100000)
  {
    delay = 100000;
  }
  if (delay < 100000)
  {
    digitalWrite(GPIO_STEP_D, HIGH);
    delayMicroseconds(2);
    digitalWrite(GPIO_STEP_D, LOW);

    if (moteur_droit.getDirection() == 1) {
      moteur_droit.setPositionSteps(moteur_droit.getPositionSteps() - 16/moteur_droit.getRatio());
    }
    else {
      moteur_droit.setPositionSteps(moteur_droit.getPositionSteps() + 16/moteur_droit.getRatio());
    }
  }

  timerAlarmWrite(Timer3_Cfg, delay, true);
  timerAlarmEnable(Timer3_Cfg);

  interrupts();
  portEXIT_CRITICAL(&timerMux);
}
#endif

// ***************  CAPTEUR  *************** //

#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// ***************  CONTROL  *************** //
unsigned long previousMillisControlLoop;

#define KP -1.20
#define KI -0.82
#define KD -0.1

float dt = 5;
uint8_t anti_windup = 0;
/********************************************/

// ***************  SETUP  *************** //
void setup()
{
  int initilisation_reussie = 0;
  initilisation_reussie += initialisationsNeoPixel(pixels);
  initilisation_reussie += initialisationSerie();
  initilisation_reussie += initialisationBroches();
  initilisation_reussie += initialisationI2C();
  initilisation_reussie += initialisationSPI();

#if MOTORS_ACTIVE == 1
  moteur_droit.setSpeed(0);
  moteur_droit.setRatio(16);

  Timer3_Cfg = timerBegin(3, 80, true); // timer incrémente toutes les 1us
  timerAttachInterrupt(Timer3_Cfg, &Timer3_MoteurD_ISR, true);
  timerAlarmWrite(Timer3_Cfg, 1000000, true); // delai d'une seconde au démarrage
  timerAlarmEnable(Timer3_Cfg);

  // Enable motors
  pinMode(GPIO_ENABLE_MOTEURS, OUTPUT);
  digitalWrite(GPIO_ENABLE_MOTEURS, LOW);
#else
  // Disable motors
  digitalWrite(GPIO_ENABLE_MOTEURS, HIGH);
#endif

  while (!lox.begin())
  {
    Serial.println(F("Failed to boot VL53L0X"));
    delay(1000);
  }
  lox.startRangeContinuous();

  // Éteindre toutes les LEDs
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.setPixelColor(1, pixels.Color(0, 0, 0));
  pixels.setPixelColor(2, pixels.Color(0, 0, 0));
  pixels.setPixelColor(3, pixels.Color(0, 0, 0));
  pixels.setPixelColor(4, pixels.Color(0, 0, 0));
  pixels.show();

  
}

int green = 0;

// ***************  LOOP  *************** //
void loop()
{
#if OTA_ACTIVE == 1
  ArduinoOTA.handle();
#endif

  // Boucle de controle de la vitesse horizontale
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillisControlLoop >= dt * 1000)
  {
    previousMillisControlLoop = currentMillis;

    if (moteur_droit.getPositionDegrees() > 85 && moteur_droit.getPositionDegrees() < 95)
      moteur_droit.setTargetPositionDegrees(0);
    else 
      moteur_droit.setTargetPositionDegrees(90);

    green += 10;
    green = green % 255;

    /*
    // Section pour tester le capteur
    if (lox.isRangeComplete())
    {
      Serial.print("Distance in mm: ");
      Serial.println(lox.readRange());
    }
    */

    Serial.println(moteur_droit.getPositionDegrees());

  }

}
