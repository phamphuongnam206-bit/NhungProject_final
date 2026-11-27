#pragma once
#include <stdint.h>
#ifndef STATIC_ROUTE_HOST_TEST
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#endif
namespace RangeSensors {
constexpr unsigned Count = 4;
constexpr uint16_t NoTarget = 0xFFFF;
constexpr uint16_t MaxUsefulMm = 1200;
constexpr uint32_t StaleMs = 100;
// Beams: 0 left 35.43 deg, 1 left straight, 2 right straight,
// 3 right 35.43 deg.
struct Reading {
    uint16_t mm;
    uint32_t at;
    uint16_t errors;
};
namespace {
Reading readings[Count] = {
    {NoTarget, 0, 0}, {NoTarget, 0, 0},
    {NoTarget, 0, 0}, {NoTarget, 0, 0}
};
uint8_t readyBits = 0;
uint8_t sampledBits = 0;  // bit i: sensor i has stored a sample (a timestamp of 0 is valid)
#ifndef STATIC_ROUTE_HOST_TEST
const uint32_t XshutPins[Count] = {PA2, PA3, PA4, PA5};
const uint8_t Addresses[Count] = {0x30, 0x31, 0x32, 0x33};
TwoWire sensorWire(PB11, PB10);
VL53L0X sensors[Count];
uint8_t phases[Count] = {}; // 0: check ready, 1: read result, 2: clear/commit
uint16_t pendingMm[Count] = {};
uint32_t pendingAt[Count] = {};
uint8_t pollIndex = 0;
uint32_t lastPollUs = 0;
inline void holdLow(unsigned i) {
    digitalWrite(XshutPins[i], LOW);
    pinMode(XshutPins[i], OUTPUT);
}
inline bool probe(uint8_t address) {
    sensorWire.beginTransmission(address);
    return sensorWire.endTransmission() == 0;
}
inline bool readRegisters(unsigned i, uint8_t reg, uint8_t *data,
                          uint8_t count) {
    sensorWire.beginTransmission(Addresses[i]);
    sensorWire.write(reg);
    if (sensorWire.endTransmission(false) != 0 ||
        sensorWire.requestFrom(Addresses[i], count) != count) return false;
    for (uint8_t j = 0; j < count; ++j) {
        const int value = sensorWire.read();
        if (value < 0) return false;
        data[j] = static_cast<uint8_t>(value);
    }
    return true;
}
inline bool clearInterrupt(unsigned i) {
    sensorWire.beginTransmission(Addresses[i]);
    sensorWire.write(static_cast<uint8_t>(VL53L0X::SYSTEM_INTERRUPT_CLEAR));
    sensorWire.write(static_cast<uint8_t>(1));
    return sensorWire.endTransmission() == 0;
}
inline void noteError(unsigned i) {
    if (readings[i].errors != 0xFFFFU) ++readings[i].errors;
}
#endif
} // namespace
#ifdef STATIC_ROUTE_HOST_TEST
inline bool begin() {
    readyBits = static_cast<uint8_t>((1U << Count) - 1U);
    sampledBits = 0;
    for (unsigned i = 0; i < Count; ++i) readings[i] = {NoTarget, 0, 0};
    return true;
}
inline void poll(uint32_t) {}
inline void inject(unsigned i, uint16_t mm, uint32_t at) {
    if (i >= Count) return;
    readings[i] = {mm, at, readings[i].errors};
    sampledBits = static_cast<uint8_t>(sampledBits | (1U << i));
}
#else
inline bool begin() {
    readyBits = sampledBits = pollIndex = 0;
    lastPollUs = 0;
    for (unsigned i = 0; i < Count; ++i) {
        readings[i] = {NoTarget, 0, 0};
        phases[i] = 0;
        holdLow(i);
    }
    delay(40);
    sensorWire.begin();
    sensorWire.setClock(100000);
    for (unsigned i = 0; i < Count; ++i) {
        pinMode(XshutPins[i], INPUT);
        delay(40);
        sensors[i] = VL53L0X{};
        sensors[i].setBus(&sensorWire);
        sensors[i].setTimeout(40);
        bool ok = probe(0x29) && sensors[i].init();
        if (ok) {
            sensors[i].setAddress(Addresses[i]);
            ok = sensors[i].last_status == 0 && probe(Addresses[i]) &&
                 sensors[i].setMeasurementTimingBudget(20000);
        }
        if (ok) {
            sensors[i].startContinuous(); // Back-to-back mode.
            ok = sensors[i].last_status == 0 && probe(Addresses[i]);
        }
        if (ok) readyBits |= static_cast<uint8_t>(1U << i);
        else holdLow(i);
    }
    return readyBits == static_cast<uint8_t>((1U << Count) - 1U);
}
inline void poll(uint32_t nowMs) {
    const uint32_t nowUs = micros();
    if (nowUs - lastPollUs < 1000U) return;
    lastPollUs = nowUs;
    unsigned i = pollIndex;
    for (unsigned n = 0; n < Count; ++n) {
        i = pollIndex;
        pollIndex = static_cast<uint8_t>((pollIndex + 1U) % Count);
        if (readyBits & (1U << i)) break;
    }
    if (!(readyBits & (1U << i))) return;
    if (phases[i] == 0) {
        uint8_t interruptStatus = 0;
        if (!readRegisters(i, VL53L0X::RESULT_INTERRUPT_STATUS,
                           &interruptStatus, 1)) {
            noteError(i);
        } else if (interruptStatus & 7U) {
            phases[i] = 1;
        }
        return;
    }
    if (phases[i] == 1) {
        uint8_t raw[12];
        if (!readRegisters(i, VL53L0X::RESULT_RANGE_STATUS, raw, 12)) {
            noteError(i);
            phases[i] = 0;
            return;
        }
        const uint8_t status = static_cast<uint8_t>((raw[0] >> 3) & 0x0F);
        const uint16_t mm = static_cast<uint16_t>(raw[10]) << 8 | raw[11];
        pendingMm[i] = (status == 11 && mm != 0 && mm <= MaxUsefulMm)
                           ? mm : NoTarget;
        pendingAt[i] = nowMs;
        phases[i] = 2;
        return;
    }
    if (!clearInterrupt(i)) {
        noteError(i);
        return; // Retry until the sampled interrupt is acknowledged.
    }
    readings[i].mm = pendingMm[i];
    readings[i].at = pendingAt[i];
    sampledBits = static_cast<uint8_t>(sampledBits | (1U << i));
    phases[i] = 0;
}
#endif
inline const Reading &get(unsigned i) {
    static const Reading invalid = {NoTarget, 0, 0};
    return i < Count ? readings[i] : invalid;
}
inline bool fresh(unsigned i, uint32_t nowMs) {
    return i < Count && (readyBits & (1U << i)) && (sampledBits & (1U << i)) &&
           nowMs - readings[i].at <= StaleMs;
}
inline bool allFresh(uint32_t nowMs) {
    for (unsigned i = 0; i < Count; ++i) if (!fresh(i, nowMs)) return false;
    return true;
}
inline uint8_t readyMask() { return readyBits; }
} // namespace RangeSensors
