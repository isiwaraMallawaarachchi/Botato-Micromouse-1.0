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
#include "telemetry/Telemetry.hpp"

/*
 * Robot — top-level facade. Owns every subsystem, wires them together, and
 * exposes two entry points that main.cpp calls:
 *   onControlTick()  — from the TIM3 1kHz ISR (via HAL callback). Fast, fixed.
 *   onMainLoop()     — from while(1). Non-blocking: sensors, buttons, modes.
 *
 * Keeping this facade means main.cpp stays ~10 lines. All wiring lives here.
 */
class Robot {
public:
    void init();          // full bring-up of every subsystem
    void onControlTick(); // 1kHz: gyro + encoders + control loop
    void onMainLoop();    // background: ToF poll, buttons, mode state machine

    // Accessors so test harnesses / debugger can reach the subsystems.
    Encoder&           encoderL()  { return encL_; }
    Encoder&           encoderR()  { return encR_; }
    Gyro&              gyro()      { return gyro_; }
    WallSensorArray&   walls()     { return walls_; }
    DifferentialDrive& drive()     { return drive_; }
    ControlLoop&       control()   { return ctrl_; }
    ButtonManager&     buttons()   { return btn_; }
    ModeController&    modes()     { return modes_; }
    Navigator& 		   navigator() { return navigator_; }
    Indicator& 		   led() 	   { return led_; }
    Telemetry&         telem()     { return telemetry; }


private:
    Navigator 	      navigator_;
    Encoder           encL_;
    Encoder           encR_;
    Gyro              gyro_;
    WallSensorArray   walls_;
    DifferentialDrive drive_;
    ControlLoop       ctrl_;
    ButtonManager     btn_;
    ModeController    modes_;
    Indicator 		  led_;
};

// Single global instance (defined in Robot.cpp). No dynamic allocation.
extern Robot robot;

#endif // APP_ROBOT_HPP
