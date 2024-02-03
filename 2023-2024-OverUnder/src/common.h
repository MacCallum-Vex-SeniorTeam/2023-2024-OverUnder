#include <cmath>
#include <vector>
#include "vex.h"

#define clip(x, min, max) fmax(min, fmin(max, x));

template <typename F, typename A> 
void enumerate(std::vector<F> l, A func) {
    F obj;
    for (int i = 0; l.size(); i++) {
        obj = l[i];
        func(&i, &obj);
    }
}

template <typename F> 
void waitUntilCondition(F cond) { 
    while (1) {
        if (cond()) {
            return;
        }
    } 
}

double convert(double dist1, vex::distanceUnits initialUnits, vex::distanceUnits targetUnits) {
    if (initialUnits == targetUnits) return dist1;
    switch (targetUnits) {
        case vex::distanceUnits::cm:
            switch (initialUnits) {
                case vex::distanceUnits::mm:
                    return dist1*10.0;
                case vex::distanceUnits::in:
                    return dist1/2.54;
                default:
                    return 0.0;
            }
        case vex::distanceUnits::mm:
            switch (initialUnits) {
                case vex::distanceUnits::cm:
                    return dist1/10.0;
                case vex::distanceUnits::in:
                    return dist1/25.4;
                default:
                    return 0.0;
            }
        case vex::distanceUnits::in:
            switch (initialUnits) {
                case vex::distanceUnits::cm:
                    return dist1*2.54;
                case vex::distanceUnits::mm:
                    return dist1*25.4;
                default:
                    return 0.0;
            }
        default:
            return 0.0;
    }
    return 0.0;
}

static const double volt_max = 12, volt_min = -12;

class SlewController {
    public:
        std::vector<vex::motor> motors;
        std::vector<double> targetVoltage;
        double slewRate, required_accuracy;
        SlewController(double slewRate, double required_accuracy) {
            this->slewRate = slewRate;
            this->required_accuracy = required_accuracy;
        }
        void addTask(vex::motor motor, double targetVoltage) {
            targetVoltage = fmax(volt_min, fmin(volt_max, targetVoltage));
            motors.push_back(motor);
            this->targetVoltage.push_back(targetVoltage);
        }
        void update() {
            enumerate(motors,
                [this] (int *i, vex::motor *motor) -> void {
                    double voltage = motor->voltage();
                    double error = targetVoltage[*i] - voltage;
                    if (fabs(error) < required_accuracy) {
                        motors.erase(motors.begin()+*i);
                        targetVoltage.erase(targetVoltage.begin() + *i);
                        *i--;
                    } else {
                        if (fabs(error) > slewRate) error = copysign(slewRate, error);
                        motor->spin(vex::forward, voltage+error, vex::volt);
                    }
                }
            );
            // int i = 0;
            // for (vex::motor motor : motors) {
            //     double voltage = motor.voltage();
            //     if (fabs(voltage - targetVoltage[i]) < required_accuracy) {
            //         motors.erase(motors.begin()+i);
            //         targetVoltage.erase(targetVoltage.begin()+i);
            //         i--;
            //         continue;
            //     }
            //     double error = targetVoltage[i] - voltage;
            //     if (fabs(error) > slewRate) error = copysign(slewRate, error);
            //     motor.spin(vex::forward, voltage+error, vex::volt);
            //     i++;
            // }
            // double voltage, error;
            // for (int i = 0; i < motors.size(); i++) {
            //     voltage = motors[i].voltage();
            //     if (fabs(voltage - targetVoltage[i]) < required_accuracy) {
            //         motors.erase(motors.begin()+i);
            //         targetVoltage.erase(targetVoltage.begin()+i);
            //         i--;
            //         continue;
            //     }
            //     error = targetVoltage[i] - voltage;
            //     if (fabs(error) > slewRate) error = copysign(slewRate, error);
            //     motors[i].spin(vex::directionType::fwd, voltage+error, vex::voltageUnits::volt);
            // }
        }
};

class PIDController {
    public:
        double p, i, d, i_max, i_min, i_state, prev_state, target;
        PIDController(double p, double i, double d, double i_max, double i_min, double target) {
            this->p = p;
            this->i = i;
            this->d = d;
            this->i_max = i_max;
            this->i_min = i_min;
            this->target = target;
            i_state = 0;
            prev_state = 0;
        }
        double getValue(double input) {
            double error = target - input;
            double pTerm = p * error;
            i_state += error;
            i_state = clip(i_state, i_min, i_max);
            double iTerm = i_state * i;
            double dTerm = d * (prev_state - input);
            prev_state = input;
            return pTerm + iTerm + dTerm;
        }
        
};

enum JoystickFilter { NONE, CUBIC };
enum AngleUnits { DEGREE, RADIAN };