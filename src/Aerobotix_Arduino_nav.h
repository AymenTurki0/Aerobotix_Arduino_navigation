#ifndef AEROBOTIX_ARDUINO_NAV_H
#define AEROBOTIX_ARDUINO_NAV_H

#include <Arduino.h>
#include <TimerOne.h>

class Aerobotix_Arduino_nav {
public:
    // ===== CONSTRUCTOR =====
    Aerobotix_Arduino_nav();

    // ===== INITIALIZATION =====
    void begin();

    // ===== MAIN NAVIGATION FUNCTIONS =====
    void moveDistance(float distance, float speed);
    void dour(float angle, float speed, bool stop = false);
    void rotate(float angle, float speed);
    void go(float targetX, float targetY, float speed);
    void stopmotors();
    void updateOdometrie();

    // ===== UTILITY FUNCTIONS =====
    float RadToDeg(float radians);
    float DegToRad(float degrees);
    float calculDistance(long deltaLeftCount, long deltaRightCount, float wheel_radius, int nb_ticks);
    void speed_calcul();

    // ===== GETTER FUNCTIONS =====
    // Motor pins
    uint8_t getIN1() { return _IN1; }
    uint8_t getIN2() { return _IN2; }
    uint8_t getIN3() { return _IN3; }
    uint8_t getIN4() { return _IN4; }

    // Encoder pins
    uint8_t getInterruptPinRA() { return _interruptPinRA; }
    uint8_t getInterruptPinRB() { return _interruptPinRB; }
    uint8_t getInterruptPinLA() { return _interruptPinLA; }
    uint8_t getInterruptPinLB() { return _interruptPinLB; }

    // Physical parameters
    float getWheelRadius() { return _wheel_radius; }
    float getEntreaxe() { return _entreaxe; }
    int getNbTicks() { return _nb_ticks; }

    // Ticks conversion
    float getTickcmR() { return _tickcmR; }
    float getTickcmL() { return _tickcmL; }
    int getTickZR_P() { return _tickZR_P; }
    int getTickZL_N() { return _tickZL_N; }
    int getTickZL_P() { return _tickZL_P; }
    int getTickZR_N() { return _tickZR_N; }

    // Control parameters
    int getMaxSpeed() { return _maxSpeed; }
    int getMinSpeed() { return _minSpeed; }
    int getMaxAcc() { return _maxAcc; }
    float getPI() { return _PI; }

    // PID parameters
    float getKp() { return _kp; }
    float getKi() { return _ki; }
    float getKTheta() { return _kTheta; }
    float getKpDour() { return _kp_dour; }
    float getKPosition() { return _k_position; }

    // PWM limits
    float getPWMMin() { return _PWM_MIN; }
    float getPWMMax() { return _PWM_MAX; }
    float getPWMMinDoura() { return _PWM_MIN_DOURA; }
    float getPWMMaxDoura() { return _PWM_MAX_DOURA; }

    // Speed calculation
    int getSpeedEch() { return _speed_ech; }

    // Navigation state
    float getCurrentVelocityRight() { return _currentvelocityRight; }
    float getCurrentVelocityLeft() { return _currentvelocityLeft; }
    long getEncoderLeftCount() { return _encoderLeftCount; }
    long getEncoderRightCount() { return _encoderRightCount; }
    float getTheta() { return _theta; }
    float getDSTotal() { return _dS_total; }
    float getPWM_R() { return _PWM_R; }
    float getPWM_L() { return _PWM_L; }
    int getSens() { return _sens; }

    // ===== SETTER FUNCTIONS =====
    // Motor pins
    void setIN1(uint8_t pin) { _IN1 = pin; }
    void setIN2(uint8_t pin) { _IN2 = pin; }
    void setIN3(uint8_t pin) { _IN3 = pin; }
    void setIN4(uint8_t pin) { _IN4 = pin; }

    // Encoder pins
    void setInterruptPinRA(uint8_t pin) { _interruptPinRA = pin; }
    void setInterruptPinRB(uint8_t pin) { _interruptPinRB = pin; }
    void setInterruptPinLA(uint8_t pin) { _interruptPinLA = pin; }
    void setInterruptPinLB(uint8_t pin) { _interruptPinLB = pin; }

    // Physical parameters
    void setWheelRadius(float radius) { _wheel_radius = radius; }
    void setEntreaxe(float distance) { _entreaxe = distance; }
    void setNbTicks(int ticks) { _nb_ticks = ticks; }

    // Ticks conversion
    void setTickcmR(float ticks) { _tickcmR = ticks; }
    void setTickcmL(float ticks) { _tickcmL = ticks; }
    void setTickZR_P(int ticks) { _tickZR_P = ticks; }
    void setTickZL_N(int ticks) { _tickZL_N = ticks; }
    void setTickZL_P(int ticks) { _tickZL_P = ticks; }
    void setTickZR_N(int ticks) { _tickZR_N = ticks; }

    // Control parameters
    void setMaxSpeed(int speed) { _maxSpeed = speed; }
    void setMinSpeed(int speed) { _minSpeed = speed; }
    void setMaxAcc(int acc) { _maxAcc = acc; }
    void setPI(float pi) { _PI = pi; }

    // PID parameters
    void setKp(float kp) { _kp = kp; }
    void setKi(float ki) { _ki = ki; }
    void setKTheta(float ktheta) { _kTheta = ktheta; }
    void setKpDour(float kp_dour) { _kp_dour = kp_dour; }
    void setKPosition(float k_position) { _k_position = k_position; }

    // PWM limits
    void setPWMMin(float min) { _PWM_MIN = min; }
    void setPWMMax(float max) { _PWM_MAX = max; }
    void setPWMMinDoura(float min) { _PWM_MIN_DOURA = min; }
    void setPWMMaxDoura(float max) { _PWM_MAX_DOURA = max; }

    // Speed calculation
    void setSpeedEch(int ech) { _speed_ech = ech; }

    // Navigation state (use with caution)
    void setCurrentVelocityRight(float vel) { _currentvelocityRight = vel; }
    void setCurrentVelocityLeft(float vel) { _currentvelocityLeft = vel; }
    void setEncoderLeftCount(long count) { _encoderLeftCount = count; }
    void setEncoderRightCount(long count) { _encoderRightCount = count; }
    void setTheta(float theta) { _theta = theta; }
    void setDSTotal(float dist) { _dS_total = dist; }
    void setPWM_R(float pwm) { _PWM_R = pwm; }
    void setPWM_L(float pwm) { _PWM_L = pwm; }
    void setSens(int sens) { _sens = sens; }

private:
    // ===== PRIVATE VARIABLES =====
    // Motor control pins
    uint8_t _IN1 = 3;
    uint8_t _IN2 = 2;
    uint8_t _IN3 = 4;
    uint8_t _IN4 = 5;

    // Encoder pins
    uint8_t _interruptPinRA = 18;
    uint8_t _interruptPinRB = 19;
    uint8_t _interruptPinLA = 20;
    uint8_t _interruptPinLB = 21;

    // Robot physical parameters
    float _wheel_radius = 39.55;
    float _entreaxe = 305;
    int _nb_ticks = 800;

    // Ticks per unit conversions
    float _tickcmR = 58.6;
    float _tickcmL = 58.9;
    int _tickZR_P = 882;
    int _tickZL_N = 886;
    int _tickZL_P = 887;
    int _tickZR_N = 882;

    // Control parameters
    int _maxSpeed = 120;
    int _minSpeed = 10;
    int _maxAcc = 10;
    float _PI = 3.14159265358979323846;

    // PID parameters
    float _kp = 0.1;
    float _ki = 0.05;
    float _kTheta = 2;
    float _kp_dour = 0.001;
    float _k_position = 0.5;

    // PWM limits
    float _PWM_MIN = 70;
    float _PWM_MAX = 180;
    float _PWM_MIN_DOURA = 85;
    float _PWM_MAX_DOURA = 150;

    // Navigation variables
    float _currentvelocityRight = 0.0;
    float _currentvelocityLeft = 0.0;
    long _encoderLeftCount = 0;
    long _encoderRightCount = 0;
    float _theta = 0.0;
    float _dS_total = 0.0;

    // Control variables
    float _PWM_R = 0.0;
    float _PWM_L = 0.0;
    int _sens = 1;
    int _speed_ech = 10;

    // Error terms
    float _right_erreur = 0.0;
    float _left_erreur = 0.0;
    float _i_right_erreur = 0.0;
    float _i_left_erreur = 0.0;
    float _orientation_erreur = 0.0;
    float _i_orientation_erreur = 0.0;
    float _Theta_correction = 0.0;
    float _position_erreur = 0.0;

    // Odometry variables
    long _lastEncoderLeftCount = 0;
    long _lastEncoderRightCount = 0;
    float _totalL = 0.0;
    float _totalR = 0.0;

    // Timing variables
    unsigned long _previousMillis = 0;
    unsigned long _chrono = 0;
    long _t = 0;
    float _total_ech_l = 0.0;
    float _total_ech_r = 0.0;

    // Distance calculations
    float _dsR = 0.0;
    float _dsL = 0.0;
    float _dS = 0.0;
    float _dTheta = 0.0;

    // ===== PRIVATE METHODS =====
    void interruptR();
    void interruptL();
    void applyMotorCommand(float cmdPwmRight, float cmdPwmLeft);
    void run();
    void iniiit();
    float erreur(float PWM, float min, float max);
    float acceleration(float speed, float distance, float accel, float decel);
    float acceleration_dour(float speed, float distance, float accel, float decel);
    int constraint(float a, int min, int max);
    float getcurrentVelocity(float dist, float t);
    float angleToDistance(float angleRad, float radius);
    float ramp(int time);
};

// External instance for ISR compatibility
extern Aerobotix_Arduino_nav aerobotix_arduino_nav;

#endif // AEROBOTIX_ARDUINO_NAV_H
