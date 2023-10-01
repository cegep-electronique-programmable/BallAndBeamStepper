#ifndef StepperNB_h
#define StepperNB_h

// library interface description
class StepperNB
{
public:
    // constructors:
    StepperNB(int pin_direction, int pin_step, int pin_ms1, int pin_ms2, int pin_ms3, int number_of_steps, bool direction_inverted);

    float getSpeed(void);
    void setSpeed(float target_speed_degrees_per_second);
    int getDirection(void);
    void setDirection(int direction);
    void setRatio(int ratio);

    int getRatio(void);
    uint64_t getTimerPeriod(void);

    int version(void);

    int getPositionSteps(void);
    void setPositionSteps(int position_steps);

    float getPositionDegrees(void);

    void setTargetPositionDegrees(float target_position_degrees);
    float getTargetPositionDegrees();
    
    void computeSpeed(void);

private:
    int pin_direction;
    int pin_step;
    int pin_ms1;
    int pin_ms2;
    int pin_ms3;
    int number_of_steps;
    int direction_inverted;

    int direction;
    int ratio;

    int position_steps; // microsteps = 16 x steps
    float position_degrees;

    float target_speed_degrees_per_second;
    float target_position_degrees;

    int timer_period;

    float position_error;
    float position_error_sum;
    float position_error_previous;
    float position_error_derivative;
    float position_error_dt;
    float position_error_dt_previous;
    float position_kp;
    float position_ki;
    float position_kd;
    
};

#endif
