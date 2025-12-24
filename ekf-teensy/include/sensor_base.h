#ifndef SENSOR_BASE_H
#define SENSOR_BASE_H

#include <Arduino.h>

class SensorBase {
protected:
    const char* name_;
    uint32_t interval_ms_;
    uint32_t last_update_time_ = 0;
    uint32_t last_exec_us_ = 0;
    uint32_t max_exec_us_ = 0;
    uint32_t timing_start_ = 0;

    void startTiming() { timing_start_ = micros(); }
    
    void endTiming() {
        last_exec_us_ = micros() - timing_start_;
        if (last_exec_us_ > max_exec_us_) max_exec_us_ = last_exec_us_;
    }

public:
    SensorBase(const char* name, uint32_t interval_ms) 
        : name_(name), interval_ms_(interval_ms) {}
    
    virtual ~SensorBase() = default;
    virtual bool initialize() = 0;
    virtual void update() = 0;
    
    bool shouldUpdate(uint32_t now) const {
        return (now - last_update_time_) >= interval_ms_;
    }
    
    void markUpdated(uint32_t now) { last_update_time_ = now; }
    
    const char* name() const { return name_; }
    uint32_t lastExecUs() const { return last_exec_us_; }
    uint32_t maxExecUs() const { return max_exec_us_; }
    uint32_t intervalMs() const { return interval_ms_; }
};

#endif