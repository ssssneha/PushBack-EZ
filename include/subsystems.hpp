#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

inline pros::Motor intake1(10, pros::MotorGearset::blue);
inline pros::Motor intake2(12, pros::MotorGearset::blue);

inline pros::Optical detector(13);
inline pros::ADIDigitalOut height('B');
inline pros::ADIDigitalOut doinker('C');
inline pros::ADIDigitalOut pod('D');
inline pros::ADIDigitalOut wing('A');
inline pros::ADIDigitalOut dblPark('E');

void toggleHeight();
void toggleDoinker();
void togglePod();
void toggleWings();
void togglePark();
void intaking(double speed);
void intakingStore(double speed);
void intakeStop();