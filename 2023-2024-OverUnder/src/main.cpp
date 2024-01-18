/* #include "vex.h"

// using namespace vex;

// brain Brain;

// motor leftMotorP = motor(PORT8, ratio6_1, true);
// motor leftMotorA = motor(PORT10, ratio6_1, false);
// motor leftMotorB = motor(PORT6, ratio6_1, false);
// motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB, leftMotorP);

// motor rightMotorP = motor(PORT7, ratio6_1, false);
// motor rightMotorA = motor(PORT9, ratio6_1, true);
// motor rightMotorB = motor(PORT5, ratio6_1, true);
// motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB, rightMotorP);

// inertial DrivetrainInertial = inertial(PORT20);
// smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, DrivetrainInertial,
//                                    299.24, 320, 40, mm, 1.5);

// motor Intake = motor(PORT18, ratio6_1, false);
// motor Catapult = motor(PORT15, ratio36_1, true);

// bumper BumperA = bumper(Brain.ThreeWirePort.A);
// controller Controller1 = controller(primary);

// competition Competition;

// void auto_0() {
//     Drivetrain.setDriveVelocity(25,percent);
//     Drivetrain.driveFor(reverse, 2300, mm);
//     Drivetrain.driveFor(forward, 500, mm);
// }

// void teleOp() {
//     double leftSpeed, rightSpeed, turnSpeed = 0.5;
//     RightDriveSmart.spin(forward);
//     LeftDriveSmart.spin(forward);
//     while (Competition.isDriverControl() && Competition.isEnabled()) {

//         leftSpeed = Controller1.Axis3.position() - Controller1.Axis1.position() * turnSpeed;
//         rightSpeed = Controller1.Axis3.position() + Controller1.Axis1.position() * turnSpeed ;

        RightDriveSmart.setVelocity(rightSpeed, percent);
//         LeftDriveSmart.setVelocity(leftSpeed, percent);

//         Drivetrain.setStopping((Controller1.ButtonY.pressing() ? hold : coast));

//         if (BumperA.pressing()) {
//             Catapult.stop();
//             if (Controller1.ButtonL1.pressing()) {
//                 Catapult.setVelocity(95, percent);
//                 Catapult.spin(reverse);
//             }
//         }
//         else {
//             Catapult.setVelocity(95, percent);
//             Catapult.spin(reverse);
//         }

//         Intake.setVelocity(95,percent);
//         if (Controller1.ButtonR1.pressing()) {
//             Intake.spin(forward);
//         }
//         else if (Controller1.ButtonR2.pressing()) {
//             Intake.spin(reverse);
//         }
//         else {
//             Intake.stop();
//         }
//     }
// }

// int main() {
//     vex::competition::bStopTasksBetweenModes = false;
//     Competition.autonomous(auto_0);
//     Competition.drivercontrol(teleOp);

//     DrivetrainInertial.calibrate();
//     while (DrivetrainInertial.isCalibrating()) {}

//     return 0;
// }
*/

#include "Drive.h"

vex::brain Brain;

vex::motor lb = vex::motor(vex::PORT14, vex::ratio18_1, true);
vex::motor lm = vex::motor(vex::PORT15, vex::ratio18_1);
vex::motor lf  = vex::motor(vex::PORT16, vex::ratio18_1, true);

vex::motor rb = vex::motor(vex::PORT11, vex::ratio18_1);
vex::motor rm = vex::motor(vex::PORT12, vex::ratio18_1);
vex::motor rf = vex::motor(vex::PORT13, vex::ratio18_1);

vex::motor cata = vex::motor(vex::PORT20, vex::ratio36_1);

vex::digital_out wings = vex::digital_out(Brain.ThreeWirePort.H);
vex::digital_out block = vex::digital_out(Brain.ThreeWirePort.A);

vex::inertial i = vex::inertial(vex::PORT17);
vex::controller c = vex::controller(vex::primary);

TankDrive drive = TankDrive(4.0, 10.0, vex::distanceUnits::in, i, lb, rb, lm, rm, lf, rf);

vex::motor_group l = vex::motor_group(lb, lm, lf);
vex::motor_group r = vex::motor_group(rb, rm, rf);

vex::smartdrive d = vex::smartdrive(l, r, i, 3.25*M_PI, 13.0, 13.0, vex::distanceUnits::in, 2.0/3.0);

vex::competition Comp;

bool blockState = false, wingState = false;

void auton15s() {
    d.driveFor(10.0, vex::distanceUnits::in, 25.0, vex::velocityUnits::pct, true);
    d.driveFor(-10.0, vex::distanceUnits::in, 25.0, vex::velocityUnits::pct, true);
    wings.set(true);
    d.turnToHeading(135.0, vex::rotationUnits::deg, 25.0, vex::velocityUnits::pct, true);
    d.driveFor(36.0, vex::distanceUnits::in, 25.0, vex::velocityUnits::pct, true);
}

void R2() {blockState = !blockState;}

void L2() {wingState = !wingState;}

void teleOp() {
    c.ButtonR2.pressed(R2);
    c.ButtonL2.pressed(L2);
    cata.spin(vex::forward);
    while (Comp.isDriverControl() && Comp.isEnabled()) {
        drive.driveTeleOp(c, 1.0, JoystickFilter::CUBIC, 5);
        wings.set(wingState);
        block.set(blockState);
        cata.setVelocity(c.ButtonL1.pressing() ? 100 : 0, vex::percent);
    }
}

int main() {
    vex::competition::bStopTasksBetweenModes = false;
    Comp.autonomous(auton15s);
    Comp.drivercontrol(teleOp);

    return 0;
}
