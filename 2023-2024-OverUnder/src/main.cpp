// #include "Drive.h"

// vex::brain Brain;

// vex::motor lb = vex::motor(vex::PORT20, vex::ratio18_1);
// vex::motor lm = vex::motor(vex::PORT15, vex::ratio18_1, true);
// vex::motor lf  = vex::motor(vex::PORT16, vex::ratio18_1);

// vex::motor rb = vex::motor(vex::PORT14, vex::ratio18_1, true);
// vex::motor rm = vex::motor(vex::PORT10, vex::ratio18_1);
// vex::motor rf = vex::motor(vex::PORT12, vex::ratio18_1, true);

// vex::motor cata = vex::motor(vex::PORT11, vex::ratio36_1);

// vex::digital_out wings = vex::digital_out(Brain.ThreeWirePort.E);
// vex::digital_out block = vex::digital_out(Brain.ThreeWirePort.D);

// vex::inertial i = vex::inertial(vex::PORT13);
// vex::controller c = vex::controller(vex::primary);

// TankDrive drive = TankDrive(4.0, 10.0, vex::distanceUnits::in, 1, &i, lb, rb, lm, rm, lf, rf);

// vex::motor_group l = vex::motor_group(lb, lm, lf);
// vex::motor_group r = vex::motor_group(rb, rm, rf);

// //vex::smartdrive d = vex::smartdrive(l, r, i, 3.25*M_PI, 13.0, 13.0, vex::distanceUnits::in, 2.0/3.0);
// vex::smartdrive Drivetrain = vex::smartdrive(l, r, i, 299.24, 320, 40, vex::mm, 1.5);

// vex::competition Comp;

// bool blockState = false, wingState = false;

// void auton15s() {
//     // drive.driveWithGeometry(-12, 25);
//     Drivetrain.setDriveVelocity(25, vex::percent);
//     Drivetrain.drive(vex::reverse);
//     vex::wait(5, vex::seconds);
//     Drivetrain.driveFor(vex::forward, 200, vex::mm);
//     // Drivetrain.turn(vex::right);
//     // vex::wait(0.1, vex::seconds);
//     // wings.set(true);
//     // Drivetrain.drive(vex::forward);
//     // vex::wait(5, vex::seconds);
//     // wings.set(false);
//     // Drivetrain.turn(vex::turnType::right);
//     // vex::wait(0.5, vex::seconds);
//     // Drivetrain.drive(vex::forward);
//     // vex::wait(3, vex::seconds);
//     // Drivetrain.stop();
//     // d.driveFor(1/0.0, vex::distanceUnits::in, 25.0, vex::velocityUnits::pct, true);
//     // d.driveFor(-10.0, vex::distanceUnits::in, 25.0, vex::velocityUnits::pct, true);
//     // wings.set(true);
//     // d.turnToHeading(135.0, vex::rotationUnits::deg, 25.0, vex::velocityUnits::pct, true);
//     // d.driveFor(36.0, vex::distanceUnits::in, 25.0, vex::velocityUnits::pct, true);
// }

// void R2() {blockState = !blockState;}

// void L2() {wingState = !wingState;}

// void teleOp() {
//     cata.setStopping(vex::brakeType::hold);
//     c.ButtonR2.pressed(R2);
//     c.ButtonL2.pressed(L2);
//     cata.spin(vex::forward);
//     while (Comp.isDriverControl() && Comp.isEnabled()) {
//         drive.driveTeleOp(c, 0.75, JoystickFilter::CUBIC, 5);
//         wings.set(wingState);
//         block.set(blockState);
//         cata.setVelocity(c.ButtonL1.pressing() ? 100 : 0, vex::percent);
//     }
// }

// int main() {
//     vex::competition::bStopTasksBetweenModes = false;
//     Comp.autonomous(auton15s);
//     Comp.drivercontrol(teleOp);

//     return 0;
// }

#include "Drive.h"

vex::brain Brain;
vex::competition Comp;

vex::motor l1(vex::PORT2, vex::ratio18_1, false);
vex::motor r1(vex::PORT7, vex::ratio18_1, true);
vex::motor l2(vex::PORT12, vex::ratio18_1, false);
vex::motor r2(vex::PORT13, vex::ratio18_1, true);

vex::inertial i(vex::PORT11);
vex::controller c(vex::primary);

TankDrive drive(4, 10, vex::inches, 6/4, &i, l1, r1, l2, r2);
vex::motor_group l(l1, l2);
vex::motor_group r(r1, r2);
vex::smartdrive sdrive(l, r, i, M_PI*4, 10, 13, vex::inches, 3/2);

void auton() {
    drive.turn(12, vex::voltageUnits::volt, true);
    while (true) {
        drive.update();
    }
    
}

int main() {
    Comp.autonomous(auton);
    return 0;
}