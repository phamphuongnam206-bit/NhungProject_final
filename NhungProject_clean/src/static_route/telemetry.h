#pragma once
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

namespace StaticRoute {
// A frame is queued whole. The UART drains bounded chunks after motor service.
struct TxQueue {
    static constexpr unsigned Capacity = 3072, EventReserve = 1024;
    char data[Capacity];
    unsigned head = 0, count = 0;
    uint32_t droppedSamples = 0, droppedEvents = 0;

    void push(const char *line, bool sample = false) {
        const unsigned n = strlen(line);
        const unsigned reserve = sample ? EventReserve : 0;
        if (n + reserve > Capacity - count) {
            if (sample) ++droppedSamples;
            else ++droppedEvents;
            return;
        }
        unsigned tail = (head + count) % Capacity;
        unsigned first = n < Capacity - tail ? n : Capacity - tail;
        memcpy(data + tail, line, first);
        memcpy(data, line + first, n - first);
        count += n;
    }
    unsigned contiguous() const { return count < Capacity-head ? count : Capacity-head; }
    void consume(unsigned n) { head = (head + n) % Capacity; count -= n; }
};

// newlib-nano does not enable floating-point printf on the Blue Pill.
struct Decimal {
    char text[15];
    explicit Decimal(float value) {
        const int32_t scaled = int32_t(lroundf(value * 10.0f));
        const uint32_t magnitude = scaled < 0 ? 0U-uint32_t(scaled) : uint32_t(scaled);
        snprintf(text, sizeof(text), "%s%lu.%u", scaled < 0 ? "-" : "",
                 (unsigned long)(magnitude/10), unsigned(magnitude%10));
    }
};
// About 5 significant digits, trailing zeros trimmed: 400, 0.027, 0.00028.
struct Number {
    char text[24];
    explicit Number(float value) {
        static const uint32_t Pow10[] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000};
        float m = fabsf(value);
        unsigned d = 0;
        while (d < 7 && m < 10000.0f) { m *= 10.0f; ++d; }
        const uint32_t n = uint32_t(lroundf(m));
        int len = snprintf(text, sizeof(text), "%s%lu", value < 0 && n ? "-" : "",
                           (unsigned long)(n/Pow10[d]));
        if (d && n%Pow10[d]) {
            len += snprintf(text+len, sizeof(text)-len, ".%0*lu", int(d), (unsigned long)(n%Pow10[d]));
            while (text[len-1] == '0') text[--len] = 0;
        }
    }
};
constexpr char CsvHeader[] =
    "TIME_MS,X_MM,Y_MM,YAW_DEG,WAYPOINT,LEFT_SPEED,RIGHT_SPEED,HEADING_ERROR,CROSS_TRACK_ERROR\r\n";
}
