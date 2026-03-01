
#include "control.hpp"

esp_err_t Control::Init(Servo servo)
{
    _servo = servo;
    return ESP_OK;
}

esp_err_t Control::ControlLoop(const xAxisIMU::IMUAttitude imuAttitude)
{
    _imuAttitude = imuAttitude;

    // calculate


    // output

    _servo.SetAngle(Servo::LF0, _servoTarget.LF);
    _servo.SetAngle(Servo::RF0, _servoTarget.RF);
    _servo.SetAngle(Servo::LB0, _servoTarget.LB);
    _servo.SetAngle(Servo::LB1, _servoTarget.LB1);
    _servo.SetAngle(Servo::RB0, _servoTarget.RB);
    _servo.SetAngle(Servo::RB1, _servoTarget.RB1);
    _servo.SetAngle(Servo::FAN, _servoTarget.FAN);
    return ESP_OK;
}
