#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/adi.hpp"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

inline pros::Motor intake1(-1, pros::MotorGearset::blue);
inline pros::Motor intake2(-10, pros::MotorGearset::blue);

inline pros::Optical detector(2);
inline pros::ADIDigitalOut height('A');
inline pros::ADIDigitalOut doinker('E');
//inline pros::ADIDigitalOut pod('-');
inline pros::ADIDigitalOut wing('B');
inline pros::ADIDigitalOut dblPark('C');
inline pros::ADIDigitalOut aligner('D');

void toggleHeight();
void toggleDoinker();
void togglePod();
void toggleWings();
void togglePark();
void intaking(double speed);
void intakingStore(double speed);
void intakeStop();