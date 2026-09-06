#include "Robot.hpp"
#include "Config.h"

extern "C" {
    #include "tim.h"
    #include "i2c.h"
}

Robot robot;   // the one global instance

void Robot::init() {
    led_.init();
    led_.set(Indicator::FAST_BLINK);   // busy: calibrating

    // Encoders: left = TIM5 (not inverted), right = TIM2 (inverted).
    encL_.init(&htim5, false);
    encR_.init(&htim2, true);

    // Motors + drive.
    drive_.init();

    // Gyro on the dedicated I2C2 bus; calibrate once at boot (keep still).
    gyro_.init(&hi2c2);
    gyro_.calibrate();

    led_.flash(3);                     // 3 flashes = calibration done
    led_.set(Indicator::SLOW_BLINK);   // idle, waiting for button

    // ToF array on I2C1.
    walls_.init(&hi2c1);

    // Control loop wiring.
    ctrl_.init(&gyro_, &encL_, &encR_, &drive_);

    navigator_.init(&ctrl_, &walls_, &encL_, &encR_);

    // Buttons + mode state machine.
    btn_.init();
    modes_.init(&btn_, &ctrl_, &gyro_, &navigator_);

    // Start the 1kHz control tick last, once everything is ready.
    HAL_TIM_Base_Start_IT(&htim3);
}

void Robot::onControlTick() {
    // Order matters: fresh sensor data BEFORE the control loop consumes it.
    gyro_.update();     // oversampled read + yaw integrate
    encL_.update();
    encR_.update();
    ctrl_.tick();       // fuse -> PID -> motors
}

void Robot::onMainLoop() {
    walls_.poll();      // non-blocking ToF (I2C in main loop, not ISR)
    btn_.update();
    modes_.update();
    led_.update();
}
