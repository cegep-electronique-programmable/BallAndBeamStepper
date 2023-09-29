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

    if (moteur_droit.getDirection() == 1)
    {
      moteur_droit.setPositionSteps(moteur_droit.getPositionSteps() - 16 / moteur_droit.getRatio());
    }
    else
    {
      moteur_droit.setPositionSteps(moteur_droit.getPositionSteps() + 16 / moteur_droit.getRatio());
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

#define POSITION_CENTRALE_MM 470
#define KP 0.05
#define KI 0.008
#define KD 0.0025
#define ANGLE_OFFSET 28

float dt = 0.01;
float error_sum = 0;
float error_previous = 0;
uint8_t anti_windup = 0;
/********************************************/

// ***************  DISPLAY  *************** //
unsigned long previousMillisDisplayLoop;
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

  pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.setPixelColor(1, pixels.Color(0, 0, 0));
  pixels.setPixelColor(2, pixels.Color(0, 0, 0));
  pixels.setPixelColor(3, pixels.Color(0, 0, 0));
  pixels.setPixelColor(4, pixels.Color(0, 0, 0));
  pixels.show();

  while (!lox.begin())
  {
    Serial.println(F("Failed to boot VL53L0X"));
    delay(1000);
  }
  lox.startRangeContinuous();

#if MOTORS_ACTIVE == 1
  pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.setPixelColor(1, pixels.Color(255, 0, 0));
  pixels.setPixelColor(2, pixels.Color(0, 0, 0));
  pixels.setPixelColor(3, pixels.Color(0, 0, 0));
  pixels.setPixelColor(4, pixels.Color(0, 0, 0));
  pixels.show();

  moteur_droit.setSpeed(0);
  moteur_droit.setRatio(16);

  Timer3_Cfg = timerBegin(3, 80, true); // timer incrémente toutes les 1us
  timerAttachInterrupt(Timer3_Cfg, &Timer3_MoteurD_ISR, true);
  timerAlarmWrite(Timer3_Cfg, 1000000, true); // delai d'une seconde au démarrage
  timerAlarmEnable(Timer3_Cfg);

  // Enable motors
  pinMode(GPIO_ENABLE_MOTEURS, OUTPUT);
  digitalWrite(GPIO_ENABLE_MOTEURS, HIGH);

  delay(2000);

  pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.setPixelColor(1, pixels.Color(255, 0, 0));
  pixels.setPixelColor(2, pixels.Color(255, 0, 0));
  pixels.setPixelColor(3, pixels.Color(0, 0, 0));
  pixels.setPixelColor(4, pixels.Color(0, 0, 0));
  pixels.show();
  digitalWrite(GPIO_ENABLE_MOTEURS, LOW);

  // Se deplacer a l'horizontal avant de commencer
  moteur_droit.setTargetPositionDegrees(ANGLE_OFFSET);

  unsigned long startMillis = millis();
  while (millis() - startMillis < 2000)
  {
    moteur_droit.computeSpeed();
    delay(100);
  }

  pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  pixels.setPixelColor(1, pixels.Color(0, 255, 0));
  pixels.setPixelColor(2, pixels.Color(0, 255, 0));
  pixels.setPixelColor(3, pixels.Color(0, 255, 0));
  pixels.setPixelColor(4, pixels.Color(0, 255, 0));
  pixels.show();

  delay(1000);
#else
  // Disable motors
  digitalWrite(GPIO_ENABLE_MOTEURS, HIGH);
#endif

  // Éteindre toutes les LEDs
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.setPixelColor(1, pixels.Color(0, 0, 0));
  pixels.setPixelColor(2, pixels.Color(0, 0, 0));
  pixels.setPixelColor(3, pixels.Color(0, 0, 0));
  pixels.setPixelColor(4, pixels.Color(0, 0, 0));
  pixels.show();
}

// ***************  LOOP  *************** //
void loop()
{
#if OTA_ACTIVE == 1
  ArduinoOTA.handle();
#endif

  float distance_mm = 0;
  float error = 0;
  float error_delta = 0;
  float position = 0;
  float propotionnal = 0;
  float integral = 0;
  float derivative = 0;

  // Boucle de controle de la vitesse horizontale
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillisControlLoop >= dt * 1000)
  {
    previousMillisControlLoop = currentMillis;
    
    while (!lox.isRangeComplete());

    distance_mm = lox.readRange();
    error = POSITION_CENTRALE_MM - distance_mm;
    error_sum += error * dt;
    error_delta = (error - error_previous) / dt;
    error_previous = error;

    propotionnal = KP * error;
    integral = KI * error_sum;
    derivative = KD * error_delta;

    position = propotionnal + integral + derivative;

    if (position < -25) {
      position = -25;
      error_sum = 0;
    }
    else if (position > 45) {
      position = 45;
      error_sum = 0;
    }
    
    
    moteur_droit.setTargetPositionDegrees(position + ANGLE_OFFSET);
    moteur_droit.computeSpeed();
  }

  currentMillis = millis();

  if (currentMillis - previousMillisDisplayLoop >= 100)
  {
    previousMillisDisplayLoop = currentMillis;

    // Affiche propotionnal, integral et derivative avec un printf
    printf("%5.1f, %5.1f, %5.1f\n", propotionnal, integral, derivative);


    // Afficher la distance, l'erreur et la position
    /*
    Serial.print(distance_mm);
    Serial.print(" ");
    Serial.print(error);
    Serial.print(" ");
    Serial.print(position);
    Serial.print(" ");
    

    
    Serial.print(moteur_droit.getTargetPositionDegrees());
    Serial.print(" ");
    Serial.print(moteur_droit.getPositionDegrees());
    Serial.print(" ");
    Serial.println(moteur_droit.getSpeed());
    */

    // Afficher les valeurs de P, I et D avec la fonction printf
    //printf("%5.1f, %5.1f, %5.1f, %5.1f, %5.1f, %5.1f\n", distance_mm, error, P, I, D, position);
    //printf("P @ %d = %8.2f, D @ %d = %8.2f", &P, P, &D, D);



    /*
    Serial.print(position);
    Serial.print(" ");
    Serial.print(P);
    Serial.print(" ");
    Serial.print(I);
    Serial.print(" ");
    Serial.println(D);
    */

    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1, pixels.Color(0, 0, 0));
    pixels.setPixelColor(2, pixels.Color(0, 0, 0));
    pixels.setPixelColor(3, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4, pixels.Color(0, 0, 0));

    if (abs(error) < 25)
    {
      pixels.setPixelColor(2, pixels.Color(0, 128, 0));
    }
    else if (error >= 25 && error <= 200)
    {
      pixels.setPixelColor(1, pixels.Color(255, 165, 0));
    }
    else if (error <= -25 && error >= -200)
    {
      pixels.setPixelColor(3, pixels.Color(255, 165, 0));
    }
    else if (error > 200)
    {
      pixels.setPixelColor(0, pixels.Color(128, 0, 0));
    }
    else if (error < -200)
    {
      pixels.setPixelColor(4, pixels.Color(128, 0, 0));
    }

    pixels.show();
  }
}
