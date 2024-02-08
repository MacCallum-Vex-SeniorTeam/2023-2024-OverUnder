// #include "Drive.h"

// vex::brain Brain;
// vex::competition Comp;

// vex::motor l1(vex::PORT2, vex::ratio18_1, false);
// vex::motor r1(vex::PORT7, vex::ratio18_1, true);
// vex::motor l2(vex::PORT12, vex::ratio18_1, false);
// vex::motor r2(vex::PORT13, vex::ratio18_1, true);

// vex::inertial i(vex::PORT11);
// vex::controller c(vex::primary);

// TankDrive drive(4, 10, vex::inches, 6/4, &i, l1, r1, l2, r2);
// vex::motor_group l(l1, l2);
// vex::motor_group r(r1, r2);
// vex::smartdrive sdrive(l, r, i, M_PI*4, 10, 13, vex::inches, 3/2);

// void auton() {
//     drive.turn(12, vex::voltageUnits::volt, true);
//     while (true) {
//         Brain.Screen.print(drive.slewControl.targetVoltage[0]);
//         Brain.Screen.newLine();
//         Brain.Screen.print(drive.slewControl.targetVoltage[1]);
//         Brain.Screen.newLine();
//         Brain.Screen.print(drive.slewControl.targetVoltage[2]);
//         Brain.Screen.newLine();
//         Brain.Screen.print(drive.slewControl.targetVoltage[3]);
//         Brain.Screen.newLine();
//         drive.update();
//     }
    
// }

// int main() {
//     Comp.autonomous(auton);
//     return 0;
// }