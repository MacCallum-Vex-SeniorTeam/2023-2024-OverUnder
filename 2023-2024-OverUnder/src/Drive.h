#include "common.h"

class TankDrive {
    public:
        std::vector<vex::motor> motors;
        double wheelCirc, trackWidth;
        SlewController SlewController;
        vex::inertial inertial;
        template <typename... Args>
        TankDrive(double wheelDiameter, double trackWidth, vex::distanceUnits units, vex::inertial intertial, vex::motor m, Args... m2) : motors(), SlewController(0.044, 0.001) {
            _AddMotor(m, m2...);
            double conversionFactor = 1.0;
            switch (units) {
                case vex::distanceUnits::cm:
                    conversionFactor /= 2.54;
                    break;
                case vex::distanceUnits::mm:
                    conversionFactor /= 25.4;
                    break;
                default:
                    break;
            }
            wheelCirc = M_PI * wheelDiameter * conversionFactor;
            this->trackWidth = trackWidth * conversionFactor;
            this->inertial = inertial;
        }
        void driveTeleOp(vex::controller c, double turnSpeed, JoystickFilter filter = JoystickFilter::NONE, int deadZone = 0) {
            double axis3 = abs(c.Axis3.position()) < deadZone ? 0 : c.Axis3.position();
            double axis1 = abs(c.Axis1.position()) < deadZone ? 0 : c.Axis1.position();
            double leftSpeed = axis3 - axis1 * turnSpeed;
            double rightSpeed = axis3 + axis1 * turnSpeed;
            if (filter == JoystickFilter::CUBIC) {
                leftSpeed = pow(leftSpeed/100, 3.0)*100;
                rightSpeed = pow(rightSpeed/100, 3.0)*100;
            }
            int i = 0;
            for (vex::motor m: motors) {
                m.spin(vex::forward);
                m.setVelocity(i % 2 == 0 ? leftSpeed : rightSpeed, vex::percent);
                m.setStopping(c.ButtonY.pressing() ? vex::hold : vex::coast);
                i++;
            }
        }
        void drive(double voltage, vex::voltageUnits voltageUnit, bool slew) {
            if (voltageUnit == vex::voltageUnits::mV) {
                voltage /= 1000;
            }
            voltage = std::max(volt_min, std::min(volt_max, voltage));
            if (slew) {
                for (vex::motor motor: motors) {
                    SlewController.addTask(motor, voltage);
                }
            } else {
                for (vex::motor motor: motors) {
                    motor.spin(vex::directionType::fwd, voltage, vex::voltageUnits::volt);
                }
            }
        }
        void turn(double voltage, vex::voltageUnits voltageUnit, bool slew) {
            if (voltageUnit == vex::voltageUnits::mV) {
                voltage /= 1000;
            }
            voltage = std::max(volt_min, std::min(volt_max, voltage));
            if (slew) {
                int i = 0;
                for (vex::motor motor: motors) {
                    SlewController.addTask(motor, voltage*(i % 2 == 0 ? 1 : -1));
                    i++;
                }
            }
        }
        void update() {
            SlewController.update();
        }
        
        void turnWithGyroPID(double angle, AngleUnits angleUnits, double p, double i, double d, double i_max, double i_min) {     
        }
        void turnWithGeometry(double time, vex::timeUnits timeUnits, double angle, AngleUnits angleUnits) {
        }
        void driveWithGeometry(double time, vex::timeUnits timeUnits, double distance, vex::distanceUnits distanceUnits) {
            switch (distanceUnits) {
                case vex::distanceUnits::cm:
                    break;
                case vex::distanceUnits::mm:
                    break;
                default:
                    break;
            }
            
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
        void _SetMotorVelocity(double velocity, vex::velocityUnits velocityUnit) {
            for (vex::motor motor: motors) {
                motor.setVelocity(velocity, velocityUnit);
            }
        }
        void _SetReversed(bool reversed) {
            for (vex::motor motor : motors) {
                motor.setReversed(reversed);
            }
        }
};