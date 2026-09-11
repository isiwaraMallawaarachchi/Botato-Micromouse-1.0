#include "telemetry/Telemetry.hpp"
#include "Robot.hpp"
#include "Config.h"
#include <cstdio>

Telemetry telemetry;

void Telemetry::init(UART_HandleTypeDef* huart) {
    huart_     = huart;
    txBusy_    = false;
    snapReady_ = false;
    tickDiv_   = 0;
    sent_      = 0;
    dropped_   = 0;
}

// ── ISR context (TIM3, priority 0) ── scalar copies only ──────────────
void Telemetry::onControlTick() {
    if (++tickDiv_ < 20) return;     // 1000 Hz / 20 = 50 Hz
    tickDiv_ = 0;

    // Main loop hasn't drained the last one yet. Leave it alone rather than
    // tearing it — a sample skipped here costs nothing, another is 20ms away.
    if (snapReady_) return;

    snap_.ms = HAL_GetTick();

    uint8_t okMask   = 0;
    uint8_t wallMask = 0;
    for (int i = 0; i < 5; ++i) {
        snap_.tofMm[i] = (int16_t)robot.walls().distanceMm(i);
        if (robot.walls().ok(i))          okMask   |= (uint8_t)(1u << i);
        if (robot.walls().wallPresent(i)) wallMask |= (uint8_t)(1u << i);
    }
    snap_.tofOk    = okMask;
    snap_.wallMask = wallMask;

    snap_.navState  = (uint8_t)robot.navigator().state();
    snap_.navPhase  = (uint8_t)robot.navigator().phase();
    snap_.modeState = (uint8_t)robot.modes().state();
    snapReady_ = true;
}

// ── main-loop context ─────────────────────────────────────────────────
bool Telemetry::sendBuf_(int len) {
    if (txBusy_ || len <= 0 || len >= (int)sizeof(txBuf_)) return false;
    txBusy_ = true;
    if (HAL_UART_Transmit_DMA(huart_, (uint8_t*)txBuf_, (uint16_t)len) != HAL_OK) {
        txBusy_ = false;
        return false;
    }
    ++sent_;
    return true;
}

void Telemetry::update() {
    if (txBusy_) return;                  // DMA still running — do nothing

    // Priority 1: events. Never dropped; they are the record of what happened.
    if (evHead_ != evTail_) {
        if (sendBuf_(formatEvent_(txBuf_, sizeof(txBuf_), evRing_[evTail_])))
            evTail_ = (uint8_t)((evTail_ + 1) % EV_RING);
        return;
    }

    // Priority 2: the 50 Hz sample. Droppable by design.
    if (!snapReady_) return;
    int n = formatTelem_(txBuf_, sizeof(txBuf_));
    snapReady_ = false;                   // release the ISR either way
    if (!sendBuf_(n)) ++dropped_;
}

/*
 * One line of JSON, newline-terminated. ~85 bytes.
 *   d  = calibrated distance in mm, order L, LF, F, RF, R
 *   ok = bit i set -> sensor i initialised
 *   wm = bit i set -> the FIRMWARE BELIEVES a wall is there
 *
 * wm is not redundant with d. wallPresent() also requires ok_[i], so a dead
 * sensor reads "no wall" in firmware while a frontend comparing d against the
 * threshold would read "wall". Sending the belief next to the evidence is what
 * makes the disagreement — the actual bug — visible. (TELEMETRY.md §4.3)
 *
 * Integer mm only: see the float note in Telemetry.hpp.
 */
int Telemetry::formatTelem_(char* b, int n) {
    const TelemSnapshot& s = snap_;
    return snprintf(b, n,
        "{\"k\":\"t\",\"ms\":%lu,\"d\":[%d,%d,%d,%d,%d],\"ok\":%u,\"wm\":%u,"
        "\"ns\":%u,\"np\":%u,\"md\":%u}\n",
        (unsigned long)s.ms,
        s.tofMm[0], s.tofMm[1], s.tofMm[2], s.tofMm[3], s.tofMm[4],
        (unsigned)s.tofOk, (unsigned)s.wallMask,
        (unsigned)s.navState, (unsigned)s.navPhase, (unsigned)s.modeState);
}


// Ring buffer push. If the ring is full we count the loss rather than
// overwrite — a silently lost event is worse than a known-missing one.
void Telemetry::event(TelemEvent kind, int a, int b, int c) {
    uint8_t next = (uint8_t)((evHead_ + 1) % EV_RING);
    if (next == evTail_) { ++evLost_; return; }
    Ev& e = evRing_[evHead_];
    e.ms   = HAL_GetTick();
    e.seq  = ++evSeq_;
    e.kind = kind;
    e.a = (int16_t)a; e.b = (int16_t)b; e.c = (int16_t)c;
    evHead_ = next;
}

// "n" is a monotonic sequence number. A gap in it means events were lost,
// which is the frontend's cue that its picture may be stale.
int Telemetry::formatEvent_(char* b, int n, const Ev& e) {
    return snprintf(b, n,
        "{\"k\":\"e\",\"ms\":%lu,\"n\":%lu,\"t\":%u,\"a\":%d,\"b\":%d,\"c\":%d}\n",
        (unsigned long)e.ms, (unsigned long)e.seq, (unsigned)e.kind,
        e.a, e.b, e.c);
}
