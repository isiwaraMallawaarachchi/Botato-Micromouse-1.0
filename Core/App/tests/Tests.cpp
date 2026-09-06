#include "Tests.hpp"
#include "Robot.hpp"
#include "Config.h"
extern "C" {
#include "tim.h"
}

void Test_Motor_Init() {
    robot.control().enable(false);
}

void Test_Motor_Update() {
    const int16_t DUTY = 1500;
    for (int p = 0; p <= DUTY; p += 75) { robot.drive().setPwm(p, 0); HAL_Delay(20); }
    HAL_Delay(1000);
    for (int p = DUTY; p >= 0; p -= 75) { robot.drive().setPwm(p, 0); HAL_Delay(20); }
    robot.drive().stop(); HAL_Delay(500);
    for (int p = 0; p >= -DUTY; p -= 75) { robot.drive().setPwm(p, 0); HAL_Delay(20); }
    HAL_Delay(1000);
    for (int p = -DUTY; p <= 0; p += 75) { robot.drive().setPwm(p, 0); HAL_Delay(20); }
    robot.drive().stop(); HAL_Delay(1000);
}

void Test_Drive_Init() {
    robot.control().resetControllers();   // zeroes gyro angle
    robot.control().enable(true);
    robot.control().holdHeading(0.0f);    // hold the just-zeroed heading
}
