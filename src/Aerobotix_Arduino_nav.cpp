#include "Aerobotix_Arduino_nav.h"
#include <TimerOne.h>

// Create global instance
Aerobotix_Arduino_nav aerobotix_arduino_nav;

// ===== CONSTRUCTOR =====
Aerobotix_Arduino_nav::Aerobotix_Arduino_nav() {
    // All initialization is done in the header with default values
}

// ===== INITIALIZATION =====
void Aerobotix_Arduino_nav::begin() {
    Serial.begin(9600);

    // Set pin modes for motor control
    pinMode(_IN1, OUTPUT);
    pinMode(_IN2, OUTPUT);
    pinMode(_IN3, OUTPUT);
    pinMode(_IN4, OUTPUT);

    // Set pin modes for encoders
    pinMode(_interruptPinRA, INPUT_PULLUP);
    pinMode(_interruptPinRB, INPUT_PULLUP);
    pinMode(_interruptPinLA, INPUT_PULLUP);
    pinMode(_interruptPinLB, INPUT_PULLUP);

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(_interruptPinRA), interruptR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(_interruptPinLA), interruptL, CHANGE);

    // Initialize Timer
    Timer1.initialize(5000);  // 5ms
    Timer1.attachInterrupt(updateOdometrie);

    _previousMillis = millis();
    
    Serial.println("Aerobotix_Arduino_nav initialized");
}

// ===== INTERRUPT HANDLERS =====
void Aerobotix_Arduino_nav::interruptR() {
    if (digitalRead(aerobotix_arduino_nav._interruptPinRA) == digitalRead(aerobotix_arduino_nav._interruptPinRB)) {
        aerobotix_arduino_nav._encoderRightCount--;
    } else {
        aerobotix_arduino_nav._encoderRightCount++;
    }
}

void Aerobotix_Arduino_nav::interruptL() {
    if (digitalRead(aerobotix_arduino_nav._interruptPinLA) == digitalRead(aerobotix_arduino_nav._interruptPinLB)) {
        aerobotix_arduino_nav._encoderLeftCount++;
    } else {
        aerobotix_arduino_nav._encoderLeftCount--;
    }
}

// ===== MAIN NAVIGATION FUNCTIONS =====
void Aerobotix_Arduino_nav::moveDistance(float distance, float speed) {
    iniiit();
    float accel = 0.25 * distance;
    float decel = 0.5 * distance;

    while (abs(_dS_total - distance) > 5) {
        if ((_dS_total - distance) < 0)
            _sens = 1;
        else
            _sens = -1;

        float current_speed = _sens * acceleration(speed, abs(distance), abs(accel), abs(decel));
        
        // Right PID
        _right_erreur = current_speed - _currentvelocityRight;
        _i_right_erreur += _right_erreur;
        _PWM_R = _kp * _right_erreur + _ki * _i_right_erreur;

        if (_sens == 1) {
            _PWM_R = erreur(_PWM_R, _PWM_MIN, _PWM_MAX);
        } else {
            _PWM_R = erreur(_PWM_R, -_PWM_MAX, -_PWM_MIN);
        }

        // Left PID
        _left_erreur = current_speed - _currentvelocityLeft;
        _i_left_erreur += _left_erreur;
        _PWM_L = _kp * _left_erreur + _ki * _i_left_erreur;

        if (_sens == 1) {
            _PWM_L = erreur(_PWM_L, _PWM_MIN, _PWM_MAX);
        } else {
            _PWM_L = erreur(_PWM_L, -_PWM_MAX, -_PWM_MIN);
        }

        // Orientation correction
        float orientation_erreur = _totalR - _totalL;
        float Theta_correction = _kTheta * orientation_erreur;
        
        _PWM_R -= Theta_correction;
        _PWM_L += Theta_correction;
        
        if (_sens == 1) {
            _PWM_R = erreur(_PWM_R, _PWM_MIN, _PWM_MAX);
            _PWM_L = erreur(_PWM_L, _PWM_MIN, _PWM_MAX);
        } else {
            _PWM_R = erreur(_PWM_R, -_PWM_MAX, -_PWM_MIN);
            _PWM_L = erreur(_PWM_L, -_PWM_MAX, -_PWM_MIN);
        }

        // Swap for motor direction if needed
        float PWM_test = _PWM_R;
        _PWM_R = _PWM_L;
        _PWM_L = PWM_test;
        
        run();
        
        // Debug output
        Serial.print("Distance: ");
        Serial.print(_dS_total);
        Serial.print(" / ");
        Serial.print(distance);
        Serial.print(" PWM_R: ");
        Serial.print(_PWM_R);
        Serial.print(" PWM_L: ");
        Serial.println(_PWM_L);
        
        delay(10); // Small delay to prevent overwhelming the serial
    }
    stopmotors();
    Serial.print("Move completed. Total distance: ");
    Serial.println(_dS_total);
}

void Aerobotix_Arduino_nav::dour(float angle, float speed, bool stop) {
    iniiit();
    float distance = angle * _PI * _entreaxe / 180.0;
    float accel = 0.25 * distance;
    float decel = 0.5 * distance;

    while (abs((_theta * 180.0 / _PI) - angle) > 2.0) {
        if (((_totalR - _totalL) - distance) < 0) {
            _sens = 1;
        } else {
            if (stop) break;
            _sens = -1;
        }

        float current_speed = _sens * acceleration_dour(speed, abs(distance), abs(accel), abs(decel));

        // Right PID
        _right_erreur = current_speed - _currentvelocityRight;
        _i_right_erreur += _right_erreur;
        _PWM_R = _kp_dour * _right_erreur;

        if (_sens == 1) {
            _PWM_R = erreur(_PWM_R, _PWM_MIN_DOURA, _PWM_MAX_DOURA);
        } else {
            _PWM_R = erreur(_PWM_R, -_PWM_MAX_DOURA, -_PWM_MIN_DOURA);
        }

        // Left PID
        _left_erreur = -current_speed - _currentvelocityLeft;
        _i_left_erreur += _left_erreur;
        _PWM_L = _kp_dour * _left_erreur;

        if (_sens == 1) {
            _PWM_L = erreur(_PWM_L, -_PWM_MAX_DOURA, -_PWM_MIN_DOURA);
        } else {
            _PWM_L = erreur(_PWM_L, _PWM_MIN_DOURA, _PWM_MAX_DOURA);
        }

        // Position correction
        _position_erreur = _k_position * (_totalR + _totalL);
        _PWM_R += _position_erreur;
        _PWM_L -= _position_erreur;

        if (_sens == 1) {
            _PWM_L = erreur(_PWM_L, -_PWM_MAX_DOURA, -_PWM_MIN_DOURA);
            _PWM_R = erreur(_PWM_R, _PWM_MIN_DOURA, _PWM_MAX_DOURA);
        } else {
            _PWM_L = erreur(_PWM_L, _PWM_MIN_DOURA, _PWM_MAX_DOURA);
            _PWM_R = erreur(_PWM_R, -_PWM_MAX_DOURA, -_PWM_MIN_DOURA);
        }

        // Swap motors if needed
        float PWM_test = _PWM_R;
        _PWM_R = _PWM_L;
        _PWM_L = PWM_test;
        
        run();
        
        Serial.print("Angle: ");
        Serial.print(_theta * 180.0 / _PI);
        Serial.print(" / ");
        Serial.print(angle);
        Serial.print(" PWM_R: ");
        Serial.print(_PWM_R);
        Serial.print(" PWM_L: ");
        Serial.println(_PWM_L);
        
        delay(10);
    }
    stopmotors();
}

void Aerobotix_Arduino_nav::rotate(float angle, float speed) {
    // Simple rotate using dour function
    dour(angle, speed, true);
}

void Aerobotix_Arduino_nav::go(float targetX, float targetY, float speed) {
    float deltaX = targetX - (_dS_total * cos(_theta));
    float deltaY = targetY - (_dS_total * sin(_theta));
    float targetAngle = atan2(deltaY, deltaX);
    float distance = sqrt(deltaX * deltaX + deltaY * deltaY);
    
    targetAngle = RadToDeg(targetAngle);
    while (targetAngle < 0) targetAngle += 360;
    while (targetAngle >= 360) targetAngle -= 360;
    
    rotate(targetAngle, speed * 0.8);
    delay(500);
    moveDistance(distance, speed);
}

void Aerobotix_Arduino_nav::stopmotors() {
    digitalWrite(_IN1, LOW);
    digitalWrite(_IN2, LOW);
    digitalWrite(_IN3, LOW);
    digitalWrite(_IN4, LOW);
}

// ===== ODOMETRY AND SPEED CALCULATION =====
void Aerobotix_Arduino_nav::updateOdometrie() {
    _t++;

    long deltaLeftCount = aerobotix_arduino_nav._encoderLeftCount - aerobotix_arduino_nav._lastEncoderLeftCount;
    long deltaRightCount = aerobotix_arduino_nav._encoderRightCount - aerobotix_arduino_nav._lastEncoderRightCount;

    aerobotix_arduino_nav._lastEncoderLeftCount = aerobotix_arduino_nav._encoderLeftCount;
    aerobotix_arduino_nav._lastEncoderRightCount = aerobotix_arduino_nav._encoderRightCount;

    _dS = calculDistance(deltaLeftCount, deltaRightCount, _wheel_radius, _nb_ticks);
    _totalL += _dsL;
    _totalR += _dsR;
    _dS_total += _dS;

    _total_ech_l += _dsL;
    _total_ech_r += _dsR;

    _dTheta = (_dsR - _dsL) / _entreaxe;
    _theta += _dTheta;

    if (_theta > _PI) _theta -= 2.0 * _PI;
    else if (_theta < -_PI) _theta += 2.0 * _PI;

    if (_t % _speed_ech == 0) {
        speed_calcul();
        _total_ech_l = 0;
        _total_ech_r = 0;
    }

    _dsR = 0;
    _dsL = 0;
}

void Aerobotix_Arduino_nav::speed_calcul() {
    float right_encoder_speed = float(_total_ech_r / (float(_speed_ech) * 15.0 / 1000.0));
    float left_encoder_speed = float(_total_ech_l / (float(_speed_ech) * 15.0 / 1000.0));
    float alpha_speed = (float(_total_ech_r - _total_ech_l)) / (float(_speed_ech) * 15.0 / 1000.0);

    _currentvelocityRight = (right_encoder_speed + left_encoder_speed) / 2.0 + alpha_speed * _wheel_radius / 2.0;
    _currentvelocityLeft = (right_encoder_speed + left_encoder_speed) / 2.0 - alpha_speed * _wheel_radius / 2.0;
}

// ===== UTILITY FUNCTIONS =====
float Aerobotix_Arduino_nav::RadToDeg(float radians) {
    return radians * (180.0 / _PI);
}

float Aerobotix_Arduino_nav::DegToRad(float degrees) {
    return degrees * (_PI / 180.0);
}

float Aerobotix_Arduino_nav::calculDistance(long deltaLeftCount, long deltaRightCount, float wheel_radius, int nb_ticks) {
    _dsL = (deltaLeftCount / (float)nb_ticks) * 2.0 * _PI * wheel_radius;
    _dsR = (deltaRightCount / (float)nb_ticks) * 2.0 * _PI * wheel_radius;
    return (_dsL + _dsR) / 2.0;
}

// ===== PRIVATE CONTROL FUNCTIONS =====
void Aerobotix_Arduino_nav::applyMotorCommand(float cmdPwmRight, float cmdPwmLeft) {
    digitalWrite(_IN1, cmdPwmRight);
    digitalWrite(_IN2, cmdPwmRight);
    digitalWrite(_IN3, cmdPwmLeft);
    digitalWrite(_IN4, cmdPwmLeft);
}

void Aerobotix_Arduino_nav::run() {
    if (_PWM_R > 0) {
        analogWrite(_IN1, _PWM_R);
        analogWrite(_IN2, 0);
    } else {
        analogWrite(_IN1, 0);
        analogWrite(_IN2, -_PWM_R);
    }
    
    if (_PWM_L > 0) {
        analogWrite(_IN3, _PWM_L);
        analogWrite(_IN4, 0);
    } else {
        analogWrite(_IN3, 0);
        analogWrite(_IN4, -_PWM_L);
    }
}

void Aerobotix_Arduino_nav::iniiit() {
    _totalR = 0;
    _totalL = 0;
    _dS_total = 0;
    _i_right_erreur = 0;
    _i_left_erreur = 0;
    _right_erreur = 0;
    _left_erreur = 0;
    _position_erreur = 0;
    _orientation_erreur = 0;
}

float Aerobotix_Arduino_nav::erreur(float PWM, float min, float max) {
    if (PWM < min) return min;
    if (PWM > max) return max;
    return PWM;
}

float Aerobotix_Arduino_nav::acceleration(float speed, float distance, float accel, float decel) {
    float current_speed;
    if (abs(_dS_total) < accel) {
        current_speed = (speed / accel) * abs(_dS_total);
    } else if (distance - abs(_dS_total) < decel) {
        current_speed = (speed / -decel) * abs(_dS_total) + speed - ((distance - decel) * (speed / -decel));
    } else {
        current_speed = speed;
    }
    return current_speed;
}

float Aerobotix_Arduino_nav::acceleration_dour(float speed, float distance, float accel, float decel) {
    float current_speed;
    if ((_totalR - _totalL) < accel) {
        current_speed = (speed / accel) * (_totalR - _totalL);
    } else if (distance - (_totalR - _totalL) < decel) {
        current_speed = (speed / -decel) * (_totalR - _totalL) + speed - ((distance - decel) * (speed / -decel));
    } else {
        current_speed = speed;
    }
    current_speed = erreur(current_speed, _PWM_MIN, speed);
    return current_speed;
}

int Aerobotix_Arduino_nav::constraint(float a, int min, int max) {
    if (a < min) return min;
    if (a > max) return max;
    return a;
}

float Aerobotix_Arduino_nav::getcurrentVelocity(float dist, float t) {
    return dist / t;
}

float Aerobotix_Arduino_nav::angleToDistance(float angleRad, float radius) {
    float angleDeg = RadToDeg(angleRad);
    return radius * angleDeg;
}

float Aerobotix_Arduino_nav::ramp(int time) {
    return time * 0.6;
}

// ===== ARDUINO LOOP COMPATIBILITY =====
void loop() {
    Serial.print("Position: ");
    Serial.print(aerobotix_arduino_nav.getDSTotal());
    Serial.print(" cm, Angle: ");
    Serial.print(aerobotix_arduino_nav.getTheta() * 180.0 / 3.14159);
    Serial.println(" deg");
    
    delay(100);
}
