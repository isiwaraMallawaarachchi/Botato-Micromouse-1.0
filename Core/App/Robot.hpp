#ifndef APP_ROBOT_HPP
#define APP_ROBOT_HPP

#include "Indicator.hpp"
#include "Encoder.hpp"
#include "Gyro.hpp"
#include "WallSensorArray.hpp"
#include "DifferentialDrive.hpp"
#include "ControlLoop.hpp"
#include "ButtonManager.hpp"
#include "ModeController.hpp"
#include "Navigator.hpp"

/*
 * Robot — owns every subsystem and wires them together.
 *
 *   init()          competition: initCore() + navigator + mode controller
 *   initCore()      hardware + control only; used by the robot-based tests.
 *                   Starts TIM3 and the gyro calibration (runs in the ISR).
 *   onControlTick() TIM3 ISR, 1kHz: gyro, encoders, control loop
 *   onMainLoop()    competition main loop
 *   serviceCore()   ToF poll, buttons, LED — the part tests also need
 */
class Robot {
public:
    void init();
    void initCore();
    void onControlTick();
    void onMainLoop();
    void serviceCore();

    Encoder&           encoderL()  { return encL_; }
    Encoder&           encoderR()  { return encR_; }
    Gyro&              gyro()      { return gyro_; }
    WallSensorArray&   walls()     { return walls_; }
    DifferentialDrive& drive()     { return drive_; }
    ControlLoop&       control()   { return ctrl_; }
    ButtonManager&     buttons()   { return btn_; }
    ModeController&    modes()     { return modes_; }
    Navigator&         navigator() { return navigator_; }
    Indicator&         led()       { return led_; }

private:
    Encoder           encL_;
    Encoder           encR_;
    Gyro              gyro_;
    WallSensorArray   walls_;
    DifferentialDrive drive_;
    ControlLoop       ctrl_;
    ButtonManager     btn_;
    Indicator         led_;
    Navigator         navigator_;
    ModeController    modes_;
    volatile bool     coreReady_ = false;
};

extern Robot robot;   // the single instance; no dynamic allocation

#endif // APP_ROBOT_HPP
