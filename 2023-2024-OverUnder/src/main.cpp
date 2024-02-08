#include "Drive.h"

vex::brain Brain;

vex::motor lb = vex::motor(vex::PORT20, vex::ratio18_1);
vex::motor lm = vex::motor(vex::PORT15, vex::ratio18_1, true);
vex::motor lf  = vex::motor(vex::PORT16, vex::ratio18_1);

vex::motor rb = vex::motor(vex::PORT14, vex::ratio18_1, true);
vex::motor rm = vex::motor(vex::PORT10, vex::ratio18_1);
vex::motor rf = vex::motor(vex::PORT12, vex::ratio18_1, true);

vex::motor cata = vex::motor(vex::PORT11, vex::ratio36_1);

vex::digital_out wings = vex::digital_out(Brain.ThreeWirePort.E);
vex::digital_out block = vex::digital_out(Brain.ThreeWirePort.D);

vex::inertial i = vex::inertial(vex::PORT13);
vex::controller c = vex::controller(vex::primary);

TankDrive drive = TankDrive(4.0, 10.0, vex::distanceUnits::in, 1, &i, lb, rb, lm, rm, lf, rf);

vex::motor_group l = vex::motor_group(lb, lm, lf);
vex::motor_group r = vex::motor_group(rb, rm, rf);

//vex::smartdrive d = vex::smartdrive(l, r, i, 3.25*M_PI, 13.0, 13.0, vex::distanceUnits::in, 2.0/3.0);
vex::smartdrive Drivetrain = vex::smartdrive(l, r, i, 299.24, 320, 40, vex::mm, 1.5);

vex::competition Comp;

bool blockState = false, wingState = false;

void auton15s() {
    double debug = 1;
    r.setStopping(vex::brake);
    l.setStopping(vex::brake);

    r.spin(vex::forward, 10, vex::volt);
    vex::wait(0.35, vex::seconds);
    r.spin(vex::forward, 0, vex::volt);

    vex::wait(debug, vex::seconds);

    wings.set(true);
    vex::wait(0.3, vex::seconds);

    l.spin(vex::forward, 10, vex::volt);
    r.spin(vex::forward, 10, vex::volt);
    vex::wait(0.25, vex::seconds);
    l.spin(vex::forward, 0, vex::volt);
    r.spin(vex::forward, 0, vex::volt);

    vex::wait(debug, vex::seconds);

    l.spin(vex::forward, 10, vex::volt);
    vex::wait(0.2, vex::seconds);
    l.spin(vex::forward, 0, vex::volt);
    
    vex::wait(1, vex::seconds);
    wings.set(false);
    vex::wait(1, vex::seconds);

    l.spin(vex::forward, 12, vex::volt);
    r.spin(vex::forward, 12, vex::volt);
    vex::wait(0.5, vex::seconds);
    l.spin(vex::forward, 0, vex::volt);
    r.spin(vex::forward, 0, vex::volt);

    vex::wait(debug, vex::seconds);

    l.spin(vex::forward, -12, vex::volt);
    r.spin(vex::forward, -12, vex::volt);
    vex::wait(0.3, vex::seconds);
    l.spin(vex::forward, 0, vex::volt);
    r.spin(vex::forward, 0, vex::volt);

    vex::wait(debug, vex::seconds);

    l.spin(vex::forward, 12, vex::volt);
    r.spin(vex::forward, 12, vex::volt);
    vex::wait(0.5, vex::seconds);
    l.spin(vex::forward, 0, vex::volt);
    r.spin(vex::forward, 0, vex::volt);

    vex::wait(debug, vex::seconds);

    l.spin(vex::forward, -10, vex::volt);
    r.spin(vex::forward, -10, vex::volt);
    vex::wait(0.2, vex::seconds);
    l.spin(vex::forward, 0, vex::volt);
    r.spin(vex::forward, 0, vex::volt);

    vex::wait(debug, vex::seconds);

    l.spin(vex::forward, 10, vex::volt);
    r.spin(vex::forward, -10, vex::volt);
    vex::wait(0.2, vex::seconds);
    l.spin(vex::forward, 0, vex::volt);
    r.spin(vex::forward, 0, vex::volt);

    return;
}

void autonWin() {
    // double debug = 1;
    // r.setStopping(vex::brake);
    // l.setStopping(vex::brake);

    // l.spin(vex::forward, 10, vex::volt);
    // vex::wait(0.25, vex::seconds);
    // l.spin(vex::forward, 0, vex::volt);

    // vex::wait(debug, vex::seconds);

    // l.spin(vex::forward, 10, vex::volt);
    // r.spin(vex::forward, 10, vex::volt);
    // vex::wait(0.75, vex::seconds);
    // l.spin(vex::forward, 0, vex::volt);
    // r.spin(vex::forward, 0, vex::volt);

    // vex::wait(debug, vex::seconds);

    // l.spin(vex::forward, -10, vex::volt);
    // r.spin(vex::forward, 10, vex::volt);
    // vex::wait(0.75, vex::seconds);
    // l.spin(vex::forward, 0, vex::volt);
    // r.spin(vex::forward, 0, vex::volt);

    // wings.set(true);

    // vex::wait(debug, vex::seconds);

    // l.spin(vex::forward, 10, vex::volt);
    // r.spin(vex::forward, 10, vex::volt);
    // vex::wait(0.75, vex::seconds);
    // l.spin(vex::forward, 0, vex::volt);
    // r.spin(vex::forward, 0, vex::volt);

    // return;
}

void R2() {blockState = !blockState;}

void L2() {wingState = !wingState;}

void teleOp() {
    cata.setStopping(vex::brakeType::hold);
    c.ButtonR2.pressed(R2);
    c.ButtonL2.pressed(L2);
    cata.spin(vex::forward);
    while (Comp.isDriverControl() && Comp.isEnabled()) {
        drive.driveTeleOp(c, 0.75, JoystickFilter::CUBIC, 5);
        wings.set(wingState);
        block.set(blockState);
        cata.setVelocity(c.ButtonL1.pressing() ? 100 : 0, vex::percent);
    }
}

int main() {
    vex::competition::bStopTasksBetweenModes = false;
    Comp.autonomous(autonWin);
    Comp.drivercontrol(teleOp);

    return 0;
}

