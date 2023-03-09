#pragma once

#include "mavsdk_time.h"
#include "mavlink_include.h"

namespace mavsdk {

class SystemImpl;

class Timesync {
public:
    explicit Timesync(SystemImpl& parent);
    ~Timesync();

    void enable();
    void do_work();

    bool is_converged() {
        return _sequence >= CONVERGENCE_WINDOW;
    }

    Timesync(const Timesync&) = delete;
    Timesync& operator=(const Timesync&) = delete;

private:
    SystemImpl& _parent;

    void process_timesync(const mavlink_message_t& message);
    void send_timesync(uint64_t tc1, uint64_t ts1);
    void set_timesync_offset(int64_t offset_ns, uint64_t start_transfer_local_time_ns);

    void add_sample(int64_t offset_ns);
    void reset_filter();

    static constexpr double TIMESYNC_SEND_INTERVAL_S = 0.1;
    SteadyTimePoint _last_time{};

    static constexpr uint64_t MAX_CONS_HIGH_RTT = 5;
    static constexpr uint64_t MAX_RTT_SAMPLE_MS = 10;
    static constexpr uint64_t MAX_DEVIATION_SAMPLE_MS = 100;
    static constexpr uint64_t CONVERGENCE_WINDOW = 100;
    static constexpr uint32_t MAX_CONSECUTIVE_HIGH_RTT = 5;
    static constexpr uint32_t MAX_CONSECUTIVE_HIGH_DEVIATION = 5;

    // Filter gains
    //
    // Alpha : Used to smooth the overall clock offset estimate. Smaller values will lead
    // to a smoother estimate, but track time drift more slowly, introducing a bias
    // in the estimate. Larger values will cause low-amplitude oscillations.
    //
    // Beta : Used to smooth the clock skew estimate. Smaller values will lead to a
    // tighter estimation of the skew (derivative), but will negatively affect how fast the
    // filter reacts to clock skewing (e.g cause by temperature changes to the oscillator).
    // Larger values will cause large-amplitude oscillations.
    static constexpr double ALPHA_GAIN_INITIAL = 0.05;
    static constexpr double BETA_GAIN_INITIAL = 0.05;
    static constexpr double ALPHA_GAIN_FINAL = 0.003;
    static constexpr double BETA_GAIN_FINAL = 0.003;

    uint64_t _high_rtt_count{};
    uint64_t _high_deviation_count{};
    // bool _autopilot_timesync_acquired{false};
    uint64_t _sequence{};

    double _time_offset{};
    double _time_skew{};

    // Filter parameters
	double _filter_alpha{ALPHA_GAIN_INITIAL};
	double _filter_beta{BETA_GAIN_INITIAL};

    bool _is_enabled{false};
};
} // namespace mavsdk
