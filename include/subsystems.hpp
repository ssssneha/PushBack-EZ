#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

inline pros::Motor intake1(1, pros::MotorGearset::blue); //bottom
inline pros::Motor intake2(10, pros::MotorGearset::blue); //hood

inline pros::Optical detector(13);
inline pros::ADIDigitalOut height('A');
inline pros::ADIDigitalOut doinker('E');
//inline pros::ADIDigitalOut pod('D');
inline pros::ADIDigitalOut wing('B');
inline pros::ADIDigitalOut dblPark50('C');
inline pros::ADIDigitalOut dblPark100('D');

void toggleHeight();
void toggleDoinker();
void toggleWings();
void togglePark100();
void togglePark50();
void intaking(double speed);
void intakingStore(double speed);
void intakeStop();