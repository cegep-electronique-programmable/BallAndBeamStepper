#include "Arduino.h"
#include "StepperNB.h"

#define MAX_SPEED_DEG_SEC 3600.0
#define MAX_ACCELERATION_DEG_SEC2 500.0

StepperNB::StepperNB(int pin_direction, int pin_step, int pin_ms1, int pin_ms2, int pin_ms3, int number_of_steps, bool direction_inverted)
{
    this->pin_direction = pin_direction;
    this->pin_step = pin_step;
    this->pin_ms1 = pin_ms1;
    this->pin_ms2 = pin_ms2;
    this->pin_ms3 = pin_ms3;
    this->number_of_steps = number_of_steps;
    this->direction_inverted = direction_inverted;

    this->direction = 0;
    this->ratio = 1;

    this->position_steps = 0;
    this->position_degrees = 0;

    this->target_speed_degrees_per_second = 0;
    this->target_position_degrees = 0;

    this->position_error = 0;
    this->position_error_sum = 0;
    this->position_error_previous = 0;
    this->position_error_derivative = 0;
    this->position_error_dt = 0;
    this->position_error_dt_previous = micros();
    this->position_kp = 6.5;
    this->position_ki = 0.5;
    this->position_kd = 0.01;
}

float StepperNB::getSpeed(void) {
    return this->target_speed_degrees_per_second;
}

void StepperNB::computeSpeed(void) {
    // Compute dt
    this->position_error_dt = (micros() - this->position_error_dt_previous) / 1000000.0;
    this->position_error_dt_previous = micros();

    // Compute error
    this->position_error = this->target_position_degrees - this->position_degrees;

    if (abs(position_error) < 0.5) {
        this->position_error = 0;
    }

    // Compute integral
    this->position_error_sum += position_error * this->position_error_dt;

    // Compute derivative
    this->position_error_derivative = (this->position_error - this->position_error_previous) / this->position_error_dt;

    // Compute PID
    float target_speed = this->position_kp * position_error + this->position_ki * this->position_error_sum + this->position_kd * this->position_error_derivative;

    // Saturation
    if (target_speed > this->getSpeed() + MAX_ACCELERATION_DEG_SEC2) {
        target_speed = this->getSpeed() + MAX_ACCELERATION_DEG_SEC2;
        this->position_error_sum = 0;
    }
    else if (target_speed < this->getSpeed() - MAX_ACCELERATION_DEG_SEC2) {
        target_speed = this->getSpeed() - MAX_ACCELERATION_DEG_SEC2;
        this->position_error_sum = 0;
    }

    
    if (target_speed > MAX_SPEED_DEG_SEC) {
        target_speed = MAX_SPEED_DEG_SEC;
        this->position_error_sum = 0;
    }
    else if (target_speed < -MAX_SPEED_DEG_SEC) {
        target_speed = -MAX_SPEED_DEG_SEC;
        this->position_error_sum = 0;
    }
    
    // Vitesse minimale
    /*
    if (target_speed > 0 && target_speed < 5)
        target_speed = 5;
    if (target_speed < 0 && target_speed > -5)
        target_speed = -5;
    */
    this->setSpeed(target_speed);
}

void StepperNB::setSpeed(float target_speed_degrees_per_second)
{
    this->target_speed_degrees_per_second = target_speed_degrees_per_second;

    // Adujst direction
    if (this->target_speed_degrees_per_second > 0)
    {
        this->setDirection(0);
    }
    else
    {
        this->setDirection(1);
    }

    // TODO: adjust ratio according to speed range
    if (target_speed_degrees_per_second < 45 && target_speed_degrees_per_second > -45) {
        this->setRatio(16);
    }
    else if (target_speed_degrees_per_second < 90 && target_speed_degrees_per_second > -90) {
        this->setRatio(8);
    }
    else if (target_speed_degrees_per_second < 180 && target_speed_degrees_per_second > -180) {
        this->setRatio(4);
    }
    else if (target_speed_degrees_per_second < 360 && target_speed_degrees_per_second > -360) {
        this->setRatio(2);
    }
    else {
        this->setRatio(1);
    }

    // Compute timer period
    float step_per_second = abs(this->target_speed_degrees_per_second) / 360.0 * this->number_of_steps * this->ratio;
    int timer_period = 1000000.0 / step_per_second; // Nombre de micro secondes entre deux impulsions
    if (timer_period < 100) // saturation at 100 us
    {
        timer_period = 100;
    }

    
    
    this->timer_period = timer_period;
}

int StepperNB::getDirection(void)
{
    return this->direction;
}

void StepperNB::setDirection(int direction)
{
    this->direction = direction;
    if (this->direction_inverted)
    {
        digitalWrite(this->pin_direction, !direction);
    }
    else 
    {
        digitalWrite(this->pin_direction, direction);  
    }
    
    
}

void StepperNB::setRatio(int ratio)
{
    switch (ratio)
    {
    case 1:
        this->ratio = ratio;
        digitalWrite(this->pin_ms1, LOW);
        digitalWrite(this->pin_ms2, LOW);
        digitalWrite(this->pin_ms3, LOW);
        break;
    case 2:
        this->ratio = ratio;
        digitalWrite(this->pin_ms1, HIGH);
        digitalWrite(this->pin_ms2, LOW);
        digitalWrite(this->pin_ms3, LOW);
        break;
    case 4:
        this->ratio = ratio;
        digitalWrite(this->pin_ms1, LOW);
        digitalWrite(this->pin_ms2, HIGH);
        digitalWrite(this->pin_ms3, LOW);
        break;
    case 8:
        this->ratio = ratio;
        digitalWrite(this->pin_ms1, HIGH);
        digitalWrite(this->pin_ms2, HIGH);
        digitalWrite(this->pin_ms3, LOW);
        break;
    case 16:
        this->ratio = ratio;
        digitalWrite(this->pin_ms1, HIGH);
        digitalWrite(this->pin_ms2, HIGH);
        digitalWrite(this->pin_ms3, HIGH);
        break;
    default:
        break;
    }
}

int StepperNB::getRatio(void)
{
    return this->ratio;
}

uint64_t StepperNB::getTimerPeriod(void)
{
    
    return this->timer_period;
}

int StepperNB::version(void)
{
    return 0;
}

void StepperNB::setPositionSteps(int position_steps)
{
    this->position_steps = position_steps;
    this->position_degrees = this->position_steps / 16.0 * 360.0 / this->number_of_steps;
}

int StepperNB::getPositionSteps(void)
{
    return this->position_steps;
}

float StepperNB::getPositionDegrees(void)
{
    return this->position_degrees;
}

void StepperNB::setTargetPositionDegrees(float target_position_degrees)
{
    this->target_position_degrees = target_position_degrees;
    this->position_error_sum = 0;
}

float StepperNB::getTargetPositionDegrees()
{
    return this->target_position_degrees;
}