#ifndef APP_MOTORCONFIG_H
#define APP_MOTORCONFIG_H

namespace motorcfg {

// Minimum PWM that actually turns a wheel against static friction. Any
// non-zero command is scaled into [DEADBAND, PWM_MAX]. Tune with TEST_MOTOR:
// raise it if a wheel stalls at low command, lower it if the robot jerks.
constexpr int DEADBAND = 500;

} // namespace motorcfg

#endif // APP_MOTORCONFIG_H
