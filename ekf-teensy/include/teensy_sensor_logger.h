#ifndef TEENSY_SENSOR_LOGGER_H
#define TEENSY_SENSOR_LOGGER_H

#include <sensor_base.h>
#include <Arduino.h>
#include <SD.h>
#include <string.h>

#ifndef SENSORBASE_SAVE_MAX_BYTES
#define SENSORBASE_SAVE_MAX_BYTES 32
#endif

#ifndef SENSORBASE_LOG_PATH_MAX
#define SENSORBASE_LOG_PATH_MAX 64
#endif

#ifndef SENSORBASE_SD_DEFAULT_CS
#ifdef BUILTIN_SDCARD
#define SENSORBASE_SD_DEFAULT_CS BUILTIN_SDCARD
#else
#define SENSORBASE_SD_DEFAULT_CS 10
#endif
#endif

namespace sensors {

/**
 * Teensy-specific sensor logger base class.
 * Provides SD card logging functionality for Teensy platform sensors.
 * This is a mixin class that adds logging capabilities without inheriting from Sensor.
 */
class TeensySensorLogger {
protected:
    const char* name_ = nullptr;
    uint32_t interval_us_ = 0;
    uint32_t next_due_us_ = 0;
    uint32_t last_update_us_ = 0;
    bool has_next_due_ = false;
    
    uint32_t last_exec_us_ = 0;
    uint32_t max_exec_us_ = 0;
    uint32_t timing_start_ = 0;
    uint32_t updates_since_report_ = 0;

    File log_file_;
    bool log_prepared_ = false;
    uint32_t last_sd_file_flush_ms_ = 0;

    static constexpr size_t LOG_BUFFER_BYTES = 512;
    uint8_t log_buf_[LOG_BUFFER_BYTES] = {0};
    size_t log_buf_len_ = 0;

    inline static bool sd_initialized_ = false;
    inline static bool sd_ok_ = false;
    inline static uint8_t sd_cs_pin_ = SENSORBASE_SD_DEFAULT_CS;

    void startTiming() { timing_start_ = micros(); }

    void endTiming() {
        last_exec_us_ = micros() - timing_start_;
        if (last_exec_us_ > max_exec_us_) max_exec_us_ = last_exec_us_;
    }

    void buildLogPath(char* out, size_t out_len) const {
        if (out == nullptr || out_len == 0) return;
        out[0] = '\0';

        if (name_ == nullptr || name_[0] == '\0') {
            snprintf(out, out_len, "/sensor.bin");
            return;
        }

        if (name_[0] == '/') {
            // Treat name_ as explicit path.
            snprintf(out, out_len, "%s", name_);
            return;
        }

        // Default: per-sensor file at root with .bin extension.
        snprintf(out, out_len, "/%s.bin", name_);
    }

    bool ensureLogFileOpen() {
        if (!save_to_sd) return false;
        if (!sd_initialized_ || !sd_ok_) return false;
        if (log_file_) return true;

        char path[SENSORBASE_LOG_PATH_MAX];
        buildLogPath(path, sizeof(path));

        if (!log_prepared_) {
            log_prepared_ = true;
            if (overwrite_on_start) {
                SD.remove(path);
            }
        }

        log_file_ = SD.open(path, FILE_WRITE);
        last_sd_file_flush_ms_ = millis();
        return static_cast<bool>(log_file_);
    }

    void flushLogBuffer(bool force_file_flush) {
        if (!log_file_ || log_buf_len_ == 0) {
            if (force_file_flush && log_file_) {
                log_file_.flush();
                last_sd_file_flush_ms_ = millis();
            }
            return;
        }

        log_file_.write(log_buf_, log_buf_len_);
        log_buf_len_ = 0;

        if (force_file_flush) {
            log_file_.flush();
            last_sd_file_flush_ms_ = millis();
        }
    }

    void saveBytesIfEnabled(uint32_t now_ms, const void* data, size_t len) {
        if (!save_to_sd) return;
        if (!ensureLogFileOpen()) return;

        struct RecordHeader {
            uint32_t t_ms;
            uint16_t len;
            uint8_t truncated;
            uint8_t reserved;
        } hdr;

        const bool truncated = len > SENSORBASE_SAVE_MAX_BYTES;
        const uint16_t write_len = static_cast<uint16_t>(truncated ? SENSORBASE_SAVE_MAX_BYTES : len);

        hdr.t_ms = now_ms;
        hdr.len = write_len;
        hdr.truncated = truncated ? 1 : 0;
        hdr.reserved = 0;

        const size_t record_bytes = sizeof(hdr) + write_len;

        // If this record won't fit, flush buffered bytes first.
        if (record_bytes > LOG_BUFFER_BYTES) {
            flushLogBuffer(true);
            log_file_.write(reinterpret_cast<const uint8_t*>(&hdr), sizeof(hdr));
            if (write_len > 0 && data != nullptr) {
                log_file_.write(reinterpret_cast<const uint8_t*>(data), write_len);
            }
            if (flush_each_record) {
                log_file_.flush();
                last_sd_file_flush_ms_ = millis();
            }
            return;
        }

        if (log_buf_len_ + record_bytes > LOG_BUFFER_BYTES) {
            flushLogBuffer(false);
        }

        memcpy(log_buf_ + log_buf_len_, &hdr, sizeof(hdr));
        log_buf_len_ += sizeof(hdr);
        if (write_len > 0 && data != nullptr) {
            memcpy(log_buf_ + log_buf_len_, data, write_len);
            log_buf_len_ += write_len;
        }

        const uint32_t now_flush_ms = millis();
        const bool time_to_flush = (flush_interval_ms > 0) && 
                                   ((now_flush_ms - last_sd_file_flush_ms_) >= flush_interval_ms);
        if (flush_each_record || time_to_flush) {
            flushLogBuffer(true);
        }
    }

    template <typename T>
    void saveValueIfEnabled(uint32_t now_ms, const T& value) {
        saveBytesIfEnabled(now_ms, &value, sizeof(T));
    }

    void markUpdated(uint32_t now_us) {
        last_update_us_ = now_us;

        if (!has_next_due_) {
            has_next_due_ = true;
            next_due_us_ = now_us + interval_us_;
        } else {
            // Keep a stable cadence based on the prior deadline.
            // If we missed deadlines, skip ahead until the next one is in the future.
            do {
                next_due_us_ += interval_us_;
            } while (static_cast<int32_t>(now_us - next_due_us_) >= 0);
        }

        updates_since_report_++;
    }

public:
    bool save_to_sd = false;
    bool flush_each_record = false;
    uint32_t flush_interval_ms = 1000;
    bool overwrite_on_start = true;

    TeensySensorLogger(const char* name, uint32_t interval_us)
        : name_(name), interval_us_(interval_us), next_due_us_(0) {}

    virtual ~TeensySensorLogger() = default;

    static bool initSd(uint8_t cs_pin = SENSORBASE_SD_DEFAULT_CS) {
        sd_cs_pin_ = cs_pin;
        sd_initialized_ = true;
        sd_ok_ = SD.begin(sd_cs_pin_);
        return sd_ok_;
    }

    static bool sdOk() { return sd_ok_; }

    void closeLogFile() {
        if (log_file_) {
            flushLogBuffer(true);
            log_file_.flush();
            log_file_.close();
        }
    }

    void forceFlushLog() {
        if (!save_to_sd) return;
        if (!ensureLogFileOpen()) return;
        flushLogBuffer(true);
    }

    uint32_t consumeUpdatesSinceReport() {
        const uint32_t n = updates_since_report_;
        updates_since_report_ = 0;
        return n;
    }

    bool is_due(uint32_t current_time_us) {
        if (!has_next_due_) return true;
        return static_cast<int32_t>(current_time_us - next_due_us_) >= 0;
    }

    uint32_t lastExecUs() const { return last_exec_us_; }
    uint32_t maxExecUs() const { return max_exec_us_; }
    
    // Convert from microseconds to milliseconds for compatibility
    uint32_t intervalMs() const { return interval_us_ / 1000; }
};

}  // namespace sensors

#endif  // TEENSY_SENSOR_LOGGER_H
