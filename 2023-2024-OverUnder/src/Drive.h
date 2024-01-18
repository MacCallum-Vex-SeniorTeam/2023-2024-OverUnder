#include "common.h"

class TankDrive {
    public:
        std::vector<vex::motor> motors;
        double wheelCirc, trackWidth;
        SlewController SlewController;
        vex::inertial *inertialSensor;
        template <typename... Args>
        TankDrive(double wheelDiameter, double trackWidth, vex::distanceUnits units, vex::inertial &inertialSensor, vex::motor m, Args... m2) : motors(), SlewController(0.044, 0.001) {
            _AddMotor(m, m2...);
            double conversionFactor = 1.0;
            wheelCirc = M_PI * convert(wheelDiameter, units, vex::distanceUnits::in);
            this->trackWidth = convert(trackWidth, units, vex::distanceUnits::in);
            this->inertialSensor = inertialSensor;
        }
        void driveTeleOp(vex::controller c, double turnSpeed, vex::brain b, JoystickFilter filter = JoystickFilter::NONE, int deadZone = 0) {
            double axis3 = abs(c.Axis3.position()) < deadZone ? 0 : c.Axis3.position();
            double axis1 = abs(c.Axis1.position()) < deadZone ? 0 : c.Axis1.position();
            double leftSpeed = axis3 + axis1 * turnSpeed;
            double rightSpeed = axis3 - axis1 * turnSpeed;
            if (filter == JoystickFilter::CUBIC) {
                leftSpeed = pow(leftSpeed/100, 3.0)*100;
                rightSpeed = pow(rightSpeed/100, 3.0)*100;
            }
            int i = 0;
            for (vex::motor m: motors) {
                m.spin(vex::forward);
                m.setVelocity(i % 2 == 0 ? leftSpeed : rightSpeed, vex::percent);
                m.setStopping(c.ButtonY.pressing() ? vex::hold : vex::coast);
                b.Screen.print(m.velocity(vex::percent));
                b.Screen.newLine();
                i++;
            }
            b.Screen.newLine();
            b.Screen.print(leftSpeed);
            b.Screen.newLine();
            b.Screen.print(rightSpeed);
            b.Screen.newLine();
        }
        void drive(double voltage, vex::voltageUnits voltageUnit = vex::voltageUnits::volt, bool slew = true) {
            if (voltageUnit == vex::voltageUnits::mV) voltage /= 1000;
            voltage = clip(voltage, volt_min, volt_max);
            if (slew) {
                for (vex::motor motor: motors) SlewController.addTask(motor, voltage);
            } else {
                for (vex::motor motor: motors) motor.spin(vex::forward, voltage, vex::voltageUnits::volt);
            }
        }
        void turn(double voltage, vex::voltageUnits voltageUnit, bool slew) {
            if (voltageUnit == vex::voltageUnits::mV) voltage /= 1000;
            voltage = clip(voltage, volt_min, volt_max);
            if (slew) {
                int i = 0;
                for (vex::motor motor: motors) {
                    SlewController.addTask(motor, voltage*(i % 2 == 0 ? 1 : -1));
                    i++;
                }
            } else {
                int i = 0;
                for (vex::motor motor: motors) {
                    motor.spin(vex::forward, voltage*(i % 2 == 0 ? 1 : -1), vex::voltageUnits::volt);
                    i++;
                }
            }
        }
        void update() {
            SlewController.update();
        }
        
        void turnWithGyroPID(double angle, AngleUnits angleUnits, double p, double i, double d, double i_max, double i_min) {     
        }
        void turnWithGeometry(double angle, double velocity, AngleUnits angleUnits = AngleUnits::DEGREE, vex::velocityUnits velUnits = vex::velocityUnits::pct) {
        }
        void driveWithGeometry(double distance, double velocity, vex::distanceUnits distanceUnits = vex::distanceUnits::in, vex::velocityUnits vUnits = vex::velocityUnits::pct, double accuracy = 0.01) {
            distance = convert(distance, distanceUnits, vex::distanceUnits::in);
            distance /= wheelCirc;
            double target[motors.size()];
            int i = 0;
            for (vex::motor motor: motors) {
                target[i] = motor.position(vex::rotationUnits::rev) + distance;
                i++;
            }
            _SetMotorVelocity(velocity, vUnits);
            waitUntilCondition([&motors, &target]() -> bool {
                int i = 0;
                for (vex::motor motor: motors) if (target[i] - motor.position(vex::rotationUnits::rev) < accuracy) return true;
                return false;
            });
        }
        void driveWithInertialPID(double distance, vex::distanceUnits distanceUnits, double p, double i, double d, double i_max, double i_min) {
        }
    private:
        template <typename... Args>
        void _AddMotor(vex::motor m1, Args... m2) {
            motors.push_back(m1);
            _AddMotor(m2...);
        }
        void _AddMotor(vex::motor m1) {
            motors.push_back(m1);
        }
        void _SetMotorVelocity(double velocity, vex::velocityUnits velocityUnit = vex::velocityUnits::pct) {
            if (velocityUnit == vex::velocityUnits::pct) velocity = clip(velocity, 0, 100);
            for (vex::motor motor: motors) {
                motor.spin(vex::forward, velocity, velocityUnit);
            }
        }
        void _SetMotorVoltage(double voltage, vex::voltageUnits voltageUnits = vex::voltageUnits::volt) {
            if (voltageUnits == vex::voltageUnits::mV) voltage /= 1000;
            voltage = clip(voltage, volt_min, volt_max);
            for (vex::motor motor: motors) {
                motor.spin(vex::forward, voltage, vex::voltageUnits::volt);
            }
        }
};