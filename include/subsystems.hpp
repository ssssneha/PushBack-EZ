#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

inline pros::Motor intake1(-1, pros::MotorGearset::blue); //bottom
inline pros::Motor intake2(10, pros::MotorGearset::blue); //hood

inline pros::Optical detector(13);
inline pros::ADIDigitalOut height('A');
inline pros::ADIDigitalOut doinker('E');
inline pros::ADIDigitalOut wing('B');
inline pros::ADIDigitalOut dblPark50('C');
inline pros::ADIDigitalOut dblPark100('D');
inline pros::Distance distL(3);
inline pros::Distance distR(8);

void toggleHeight();
void toggleDoinker();
void toggleWings();
void togglePark50();
void togglePark100();
void intaking(double speed);
void intakingStore(double speed);
void intakeStop();
void intakeMid(double speed);