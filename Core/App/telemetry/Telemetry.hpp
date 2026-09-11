#ifndef APP_TELEMETRY_HPP
#define APP_TELEMETRY_HPP

#include <cstdint>

extern "C" {
#include "usart.h"
}

/*
 * Telemetry — non-blocking JSON-lines emitter over USART6 + DMA.
 *
 * Currently emits one message kind: k:"t", the 50 Hz ToF sample.
 * Events, the maze map and the hello message come later (TELEMETRY.md §4).
 *
 * The three rules from TELEMETRY.md §3.3, which this class exists to enforce:
 *   1. Format in the main loop, NEVER in the ISR. snprintf is thousands of
 *      cycles with variable duration; it has no business in a 1kHz interrupt.
 *   2. DMA, and DROP the frame if DMA is still busy. A blocking transmit of
 *      ~90 bytes at 460800 is ~2ms of a main loop that should be polling ToF.
 *   3. Snapshot atomically. TIM3 (priority 0) preempts the main loop at any
 *      instruction boundary; formatting from live members yields torn frames.
 *
 * Nothing here blocks. With USART6 disconnected every call is a no-op costing
 * a few hundred cycles.
 *
 * NOTE ON FLOATS: this project links --specs=nano.specs WITHOUT
 * -u _printf_float, so snprintf("%f") produces NOTHING. Distances are
 * therefore carried as integer millimetres. Do not introduce %f here unless
 * you also enable float printf in the project settings.
 */

// Snapshot written by the control ISR, drained by the main loop.
// Plain scalars only — the ISR must do nothing but copy.

// Discrete events. Queued and never dropped — they carry the robot's reasoning.
enum TelemEvent : uint8_t {
    EV_BOOT = 0,      // firmware started
    EV_MODE,          // ModeController changed.  a=new state
    EV_SENSE,         // walls sensed.  a=mask(L|F<<1|R<<2)  b=x  c=y
    EV_DECIDE,        // flood-fill chose.  a=Dir  b=Turn
    EV_TURN_BEGIN,    // a=Turn
    EV_TURN_END,      // a=1 settled / 0 TIMED OUT   b=heading err, tenths deg
    EV_DRIVE_BEGIN,   // a=speed mm/s
    EV_DRIVE_END,     // a=distance travelled mm
    EV_ARRIVE,        // a=x  b=y  c=Dir
    EV_GOAL,          // a=state
    EV_STATE,         // Navigator state changed.  a=new state
    EV_ABORT,
    EV_ERROR,         // a=code
};

struct TelemSnapshot {
    uint32_t ms;          // HAL_GetTick() at sample time
    int16_t  tofMm[5];    // calibrated distance, mm. Index = cfg::ToFIndex
    uint8_t  tofOk;       // bitmask, bit i = walls_.ok(i)
    uint8_t  wallMask;    // bitmask, bit i = walls_.wallPresent(i)
    uint8_t navState, navPhase, modeState;
};

class Telemetry {
public:
    void init(UART_HandleTypeDef* huart);

    // From the 1 kHz control ISR. Decimates internally to 50 Hz.
    // Cost: a 16-byte struct copy. Never formats, never blocks.
    void onControlTick();

    // From the main loop, every pass. Formats and DMAs at most ONE message.
    void update();
    // Push a discrete event. Main-loop context only, never from an ISR.
    void event(TelemEvent kind, int a = 0, int b = 0, int c = 0);

    // From HAL_UART_TxCpltCallback. Without this txBusy_ never clears and
    // telemetry stops after exactly one message.
    void onTxComplete() { txBusy_ = false; }

    // Diagnostics — add these to Live Expressions.
    uint32_t sent_    = 0;   // messages handed to DMA successfully
    uint32_t dropped_ = 0;   // frames discarded because DMA was still busy
    uint32_t evLost_  = 0;   // events lost to ring overflow — should stay 0

private:
    UART_HandleTypeDef* huart_ = nullptr;

    volatile bool txBusy_ = false;
    char          txBuf_[160];

    TelemSnapshot snap_{};
    volatile bool snapReady_ = false;
    uint16_t      tickDiv_   = 0;

    struct Ev {
        uint32_t   ms;
        uint32_t   seq;
        TelemEvent kind;
        int16_t    a, b, c;
    };
    static constexpr int EV_RING = 16;

    Ev               evRing_[EV_RING]{};
    volatile uint8_t evHead_ = 0, evTail_ = 0;
    uint32_t         evSeq_  = 0;
    bool sendBuf_(int len);              // hand txBuf_ to DMA; false if busy
    int  formatTelem_(char* b, int n);
    int  formatEvent_(char* b, int n, const Ev& e);
};

extern Telemetry telemetry;

#endif // APP_TELEMETRY_HPP
