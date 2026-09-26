#ifndef APP_ENCODERCONFIG_H
#define APP_ENCODERCONFIG_H

namespace enccfg {

// Speed is averaged over this many 1ms ticks. One tick of one encoder count is
// ~99mm/s at 1ms, so a single-tick speed is mostly quantisation noise that the
// speed PID amplifies. 4 ticks = ~25mm/s resolution for 2ms of extra lag.
constexpr int SPEED_WINDOW = 4;

} // namespace enccfg

#endif // APP_ENCODERCONFIG_H
