#ifndef STEPPER_MOTORS_H
#define STEPPER_MOTORS_H

#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

void rack_motor_deploy();
void rack_motor_retract();
void lift_platform_up();
void lift_platform_down();

extern AccelStepper stepX;
extern AccelStepper stepY;
extern AccelStepper stepZ;
extern MultiStepper XY_steppers;

extern const long STEPS_PER_REV;
extern const float XY_SPEED_SLOW;
extern const float XY_ACCEL_SLOW;
extern const float XY_SPEED_FAST;
extern const float XY_ACCEL_FAST;
extern const float Z_SPEED_NORMAL;
extern const float Z_ACCEL_NORMAL;
extern const float Z_SPEED_FAST;
extern const float Z_ACCEL_FAST;

void setupSteppers();

void test_xy();
void test_x();
void test_y();
void test_z();

#endif  // STEPPER_MOTORS_H
