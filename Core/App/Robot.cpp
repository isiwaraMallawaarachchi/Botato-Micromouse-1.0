#include "Robot.hpp"
extern "C" {
#include "tim.h"
#include "i2c.h"
}

Robot robot;

void Robot::initCore() {
    led_.init();
    led_.set(Indicator::BLINK);          // visible during the blocking ToF boot
    btn_.init();

    encL_.init(&htim5, false);           // left  = TIM5
    encR_.init(&htim2, true);            // right = TIM2, negated (#18)
    drive_.init();

    gyro_.init(&hi2c2);                  // I2C2, dedicated (#16)
    walls_.init(&hi2c1);                 // I2C1, XSHUT sequence (~1s)

    ctrl_.init(&gyro_, &encL_, &encR_, &drive_);
    ctrl_.enable(false);

    coreReady_ = true;
    HAL_TIM_Base_Start_IT(&htim3);       // 1kHz tick starts last
    gyro_.beginCalibration();            // ISR takes it from here
}

void Robot::init() {
    initCore();
    navigator_.init(&ctrl_, &walls_, &encL_, &encR_);
    modes_.init(&btn_, &ctrl_, &gyro_, &navigator_, &led_);
}

void Robot::onControlTick() {
    if (!coreReady_) return;
    gyro_.update();                      // fresh sensors BEFORE the controller
    encL_.update();
    encR_.update();
    ctrl_.tick();
}

void Robot::serviceCore() {
    walls_.poll();
    btn_.update();
    led_.update();
}

void Robot::onMainLoop() {
    walls_.poll();
    btn_.update();
    modes_.update();
    led_.update();
}
