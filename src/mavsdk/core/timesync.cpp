#include "timesync.h"
#include "log.h"
#include "system_impl.h"

// Partially based on: https://github.com/mavlink/mavros/blob/master/mavros/src/plugins/sys_time.cpp

namespace mavsdk {

Timesync::Timesync(SystemImpl& parent) : _parent(parent) {}

Timesync::~Timesync()
{
    _parent.unregister_all_mavlink_message_handlers(this);
}

void Timesync::enable()
{
    _is_enabled = true;
    _parent.register_mavlink_message_handler(
        MAVLINK_MSG_ID_TIMESYNC,
        [this](const mavlink_message_t& message) { process_timesync(message); },
        this);
}

void Timesync::do_work()
{
    if (!_is_enabled) {
        return;
    }

    if (_parent.get_time().elapsed_since_s(_last_time) >= TIMESYNC_SEND_INTERVAL_S) {
        if (_parent.is_connected()) {
            // uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            //                       _parent.get_autopilot_time().now().time_since_epoch())
            //                       .count();

            // we initiate timesync using monotonic clock
            uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(_parent.get_time().steady_time()
                .time_since_epoch()).count();
            send_timesync(0, now_ns);
        } else {
            _sequence = 0;
        }
        _last_time = _parent.get_time().steady_time();
    }
}

void Timesync::process_timesync(const mavlink_message_t& message)
{
    mavlink_timesync_t timesync{};

    mavlink_msg_timesync_decode(&message, &timesync);

    // int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    //                      _parent.get_autopilot_time().now().time_since_epoch())
    //                      .count();

    // we send monotonic clock back to FC
    int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(_parent.get_time().steady_time()
                .time_since_epoch()).count();

    if (timesync.tc1 == 0) {
        // Send synced time to remote system
        send_timesync(now_ns, timesync.ts1);
    } else if (timesync.tc1 > 0) {
        // Time offset between this system and the remote system is calculated assuming RTT for
        // the timesync packet is roughly equal both ways.
        set_timesync_offset((timesync.tc1 * 2 - (timesync.ts1 + now_ns)) / 2, timesync.ts1);
    }
}

void Timesync::send_timesync(uint64_t tc1, uint64_t ts1)
{
    mavlink_message_t message;

    mavlink_msg_timesync_pack(
        _parent.get_own_system_id(),
        _parent.get_own_component_id(),
        &message,
        static_cast<int64_t>(tc1),
        static_cast<int64_t>(ts1),
        0,
        0);
    _parent.send_message(message);
}

void Timesync::set_timesync_offset(int64_t offset_ns, uint64_t start_transfer_local_time_ns)
{
    // uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    //                       _parent.get_autopilot_time().now().time_since_epoch())
    //                       .count();
    uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                          _parent.get_time().steady_time().time_since_epoch())
                          .count();

    // Calculate the round trip time (RTT) it took the timesync packet to bounce back to us from
    // remote system
    uint64_t rtt_ns = now_ns - start_transfer_local_time_ns;

    uint64_t deviation_ns = std::abs((int64_t)_time_offset - offset_ns);


    if (rtt_ns < MAX_RTT_SAMPLE_MS * 1000000ULL) { // Only use samples with low RTT


        if (is_converged() && (deviation_ns / 1e6 > MAX_DEVIATION_SAMPLE_MS ) ) {

            _high_deviation_count++;

            if (_high_deviation_count > MAX_CONSECUTIVE_HIGH_DEVIATION) {
                // Issue a warning to the user if the RTT is constantly high
                LogWarn() << "Time jump detected. Resetting time sync";
                
                reset_filter();
                // Reset counter
                _high_deviation_count = 0;
            }

        } else {

            if (is_converged()) {
                // Interpolate with a sigmoid function
                double progress = (double)_sequence / (double)CONVERGENCE_WINDOW;
                double p = 1.0 - exp(0.5 * (1.0 - 1.0 / (1.0 - progress)));
                _filter_alpha = p * ALPHA_GAIN_FINAL + (1.0 - p) * ALPHA_GAIN_INITIAL;
                _filter_beta = p * BETA_GAIN_FINAL + (1.0 - p) * BETA_GAIN_INITIAL;

            } else {
                _filter_alpha = ALPHA_GAIN_FINAL;
                _filter_beta = BETA_GAIN_FINAL;
            }

            // Increment sequence counter after filter update
            _sequence++;

            if (_sequence == CONVERGENCE_WINDOW)
                LogInfo() << "Time Sync complete with offset of " << _time_offset / 1e6 << " ms";

            // Reset high RTT count after filter update
            _high_deviation_count = 0;

            _high_rtt_count = 0;

        }

        add_sample(offset_ns);

        // Save time offset for other components to use
        // _parent.get_autopilot_time().shift_time_by(std::chrono::nanoseconds(offset_ns));
        // _autopilot_timesync_acquired = true;
        
    } else {
        // Increment counter if round trip time is too high for accurate timesync
        _high_rtt_count++;

        if (_high_rtt_count > MAX_CONS_HIGH_RTT) {
            // Issue a warning to the user if the RTT is constantly high
            LogWarn() << "RTT too high for timesync: " << static_cast<double>(rtt_ns) / 1000000.0
                      << " ms.";

            // Reset counter
            _high_rtt_count = 0;
        }
    }
}

void Timesync::add_sample(int64_t offset_ns)
{
	// Online exponential smoothing filter. The derivative of the estimate is also
	// estimated in order to produce an estimate without steady state lag:
	// https://en.wikipedia.org/wiki/Exponential_smoothing#Double_exponential_smoothing
    std::lock_guard<std::mutex> lock(_mutex_time_offset);
	double time_offset_prev = _time_offset;

	if (_sequence == 0) {
		// First offset sample
		_time_offset = offset_ns;

	} else {
		// Update the clock offset estimate
		_time_offset = _filter_alpha * offset_ns + (1.0 - _filter_alpha) * (_time_offset + _time_skew);

		// Update the clock skew estimate
		_time_skew = _filter_beta * (_time_offset - time_offset_prev) + (1.0 - _filter_beta) * _time_skew;
	}
}

void Timesync::reset_filter()
{
    std::lock_guard<std::mutex> lock(_mutex_time_offset);
    _sequence = 0;
	_time_offset = 0.0;
	_time_skew = 0.0;
	_filter_alpha = ALPHA_GAIN_INITIAL;
	_filter_beta = BETA_GAIN_INITIAL;
	_high_deviation_count = 0;
	_high_rtt_count = 0;

}

} // namespace mavsdk
