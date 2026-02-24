#include "stepper_motors.h"

AccelStepper stepX(AccelStepper::DRIVER, 2, 5);
AccelStepper stepY(AccelStepper::DRIVER, 3, 6);
AccelStepper stepZ(AccelStepper::DRIVER, 4, 7);
MultiStepper XY_steppers;

const long   STEPS_PER_REV   = 200;   // 360° = 200 pas
const float  XY_SPEED_SLOW   = 300.0;
const float  XY_ACCEL_SLOW   = 150.0;
const float  XY_SPEED_FAST   = 800.0;
const float  XY_ACCEL_FAST   = 400.0;
const float  Z_SPEED_NORMAL  = 600.0;
const float  Z_ACCEL_NORMAL  = 300.0;
const float  Z_SPEED_FAST    = 1000.0;
const float  Z_ACCEL_FAST    = 600.0;

void setupSteppers() {
  stepX.setMaxSpeed(XY_SPEED_SLOW);
  stepX.setAcceleration(XY_ACCEL_SLOW);
  stepY.setMaxSpeed(XY_SPEED_SLOW);
  stepY.setAcceleration(XY_ACCEL_SLOW);

  stepZ.setMaxSpeed(Z_SPEED_NORMAL);
  stepZ.setAcceleration(Z_ACCEL_NORMAL);

  XY_steppers.addStepper(stepX);
  XY_steppers.addStepper(stepY);
}

void rack_motor_deploy() {
  stepZ.moveTo(stepZ.currentPosition() + STEPS_PER_REV);
  while (stepZ.distanceToGo() != 0) stepZ.run();
}
void rack_motor_retract() {
  stepZ.moveTo(stepZ.currentPosition() - STEPS_PER_REV);
  while (stepZ.distanceToGo() != 0) stepZ.run();
}

void lift_platform_up() {
  long targets[2] = {
    stepX.currentPosition() + STEPS_PER_REV,
    stepY.currentPosition() + STEPS_PER_REV
  };
  XY_steppers.moveTo(targets);
  XY_steppers.runSpeedToPosition();
}
void lift_platform_down() {
  long targets[2] = {
    stepX.currentPosition() - STEPS_PER_REV,
    stepY.currentPosition() - STEPS_PER_REV
  };
  XY_steppers.moveTo(targets);
  XY_steppers.runSpeedToPosition();
}

void test_xy() {
  stepX.setMaxSpeed(XY_SPEED_FAST);
  stepX.setAcceleration(XY_ACCEL_FAST);
  stepY.setMaxSpeed(XY_SPEED_FAST);
  stepY.setAcceleration(XY_ACCEL_FAST);

  lift_platform_up();
  delay(100);
  lift_platform_down();

  stepX.setMaxSpeed(XY_SPEED_SLOW);
  stepX.setAcceleration(XY_ACCEL_SLOW);
  stepY.setMaxSpeed(XY_SPEED_SLOW);
  stepY.setAcceleration(XY_ACCEL_SLOW);
}

void test_x() {
  stepX.setMaxSpeed(XY_SPEED_FAST);
  stepX.setAcceleration(XY_ACCEL_FAST);
  stepX.moveTo(stepX.currentPosition() + STEPS_PER_REV);
  while (stepX.distanceToGo() != 0) stepX.run();
  delay(100);
  stepX.moveTo(stepX.currentPosition() - STEPS_PER_REV);
  while (stepX.distanceToGo() != 0) stepX.run();
  stepX.setMaxSpeed(XY_SPEED_SLOW);
  stepX.setAcceleration(XY_ACCEL_SLOW);
}

void test_y() {
  stepY.setMaxSpeed(XY_SPEED_FAST);
  stepY.setAcceleration(XY_ACCEL_FAST);
  stepY.moveTo(stepY.currentPosition() + STEPS_PER_REV);
  while (stepY.distanceToGo() != 0) stepY.run();
  delay(100);
  stepY.moveTo(stepY.currentPosition() - STEPS_PER_REV);
  while (stepY.distanceToGo() != 0) stepY.run();
  stepY.setMaxSpeed(XY_SPEED_SLOW);
  stepY.setAcceleration(XY_ACCEL_SLOW);
}

void test_z() {
  stepZ.setMaxSpeed(Z_SPEED_FAST);
  stepZ.setAcceleration(Z_ACCEL_FAST);
  rack_motor_deploy();
  delay(100);
  rack_motor_retract();
  stepZ.setMaxSpeed(Z_SPEED_NORMAL);
  stepZ.setAcceleration(Z_ACCEL_NORMAL);
}
