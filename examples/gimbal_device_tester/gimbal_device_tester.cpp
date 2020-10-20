// Test against a MAVLink gimbal device according to the gimbal protocol v2.
//
// More info: https://mavlink.io/en/services/gimbal_v2.html
//
// Author: Julian Oes <julian@oes.ch>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <chrono>
#include <cstdint>
#include <future>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>

using namespace mavsdk;

static constexpr auto test_prefix = "[TEST] ";

float degrees(float radians)
{
    if (std::isfinite(radians)) {
        return radians * 180.0f / static_cast<float>(M_PI);
    } else {
        return radians;
    }
}

float radians(float degrees)
{
    if (std::isfinite(degrees)) {
        return degrees / 180.0f * static_cast<float>(M_PI);
    } else {
        return degrees;
    }
}

class AttitudeData {
public:
    AttitudeData() = default;
    ~AttitudeData() = default;

    struct GimbalAttitude {
        float roll_deg{NAN};
        float pitch_deg{NAN};
        float yaw_deg{NAN};
    };

    struct GimbalLimits {
        float roll_min_deg{0.0f};
        float roll_max_deg{0.0f};
        float pitch_min_deg{0.0f};
        float pitch_max_deg{0.0f};
        float yaw_min_deg{0.0f};
        float yaw_max_deg{0.0f};
    };

    struct VehicleAttitude {
        float roll_deg{0.0f};
        float pitch_deg{0.0f};
        float yaw_deg{0.0f};
    };

    enum class Mode { Follow, Lock };

    struct AttitudeSetpoint {
        Mode mode{Mode::Follow};
        float roll_deg{NAN};
        float pitch_deg{NAN};
        float yaw_deg{NAN};
        float roll_rate_deg{NAN};
        float pitch_rate_deg{NAN};
        float yaw_rate_deg{NAN};
    };

    GimbalAttitude gimbal_attitude() const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _gimbal_attitude;
    }

    GimbalLimits gimbal_limits() const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _gimbal_limits;
    }

    VehicleAttitude vehicle_attitude() const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _vehicle_attitude;
    }

    AttitudeSetpoint attitude_setpoint() const
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return _attitude_setpoint;
    }

    void change_gimbal_attitude(
        const std::function<void(GimbalAttitude& gimbal_attitude)>& change_function)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        change_function(_gimbal_attitude);
    }

    void
    change_gimbal_limits(const std::function<void(GimbalLimits& gimbal_limits)>& change_function)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        change_function(_gimbal_limits);
    }

    void change_vehicle_attitude(
        const std::function<void(VehicleAttitude& vehicle_attitude)>& change_function)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        change_function(_vehicle_attitude);
    }

    void change_attitude_setpoint(
        const std::function<void(AttitudeSetpoint& attitude_setpoint)>& change_function)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        change_function(_attitude_setpoint);
    }

private:
    mutable std::mutex _mutex{};
    GimbalAttitude _gimbal_attitude{};
    VehicleAttitude _vehicle_attitude{};
    AttitudeSetpoint _attitude_setpoint{};
    GimbalLimits _gimbal_limits{};
};

class Sender {
public:
    explicit Sender(MavlinkPassthrough& mavlink_passthrough, AttitudeData& attitude_data) :
        _mavlink_passthrough(mavlink_passthrough),
        _attitude_data(attitude_data),
        _thread(&Sender::run, this)
    {}

    ~Sender()
    {
        _should_exit = true;
        _thread.join();
    }

    void run()
    {
        while (!_should_exit) {
            send_autopilot_state_for_gimbal_device();
            send_gimbal_device_set_attitude();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

private:
    void send_autopilot_state_for_gimbal_device()
    {
        const auto vehicle_attitude = _attitude_data.vehicle_attitude();

        float q[4];
        mavlink_euler_to_quaternion(
            radians(vehicle_attitude.roll_deg),
            radians(vehicle_attitude.pitch_deg),
            radians(vehicle_attitude.yaw_deg),
            q);

        const uint16_t estimator_status =
            ESTIMATOR_ATTITUDE | ESTIMATOR_VELOCITY_HORIZ | ESTIMATOR_VELOCITY_VERT |
            ESTIMATOR_POS_HORIZ_REL | ESTIMATOR_POS_HORIZ_ABS | ESTIMATOR_POS_VERT_ABS |
            ESTIMATOR_POS_VERT_AGL | ESTIMATOR_PRED_POS_HORIZ_REL | ESTIMATOR_PRED_POS_HORIZ_ABS;

        mavlink_message_t message;
        mavlink_msg_autopilot_state_for_gimbal_device_pack(
            _mavlink_passthrough.get_our_sysid(),
            _mavlink_passthrough.get_our_compid(),
            &message,
            0, // broadcast
            0, // broadcast
            0, // FIXME: time us
            q,
            0, // q estimated delay
            0.0f, // vx
            0.0f, // vy
            0.0f, // vz
            0, // estimated delay
            0.0f, // feed forward angular velocity z
            estimator_status,
            MAV_LANDED_STATE_IN_AIR);
        _mavlink_passthrough.send_message(message);
    }

    void send_gimbal_device_set_attitude()
    {
        uint16_t flags = GIMBAL_DEVICE_FLAGS_ROLL_LOCK | GIMBAL_DEVICE_FLAGS_PITCH_LOCK;

        const auto attitude_setpoint = _attitude_data.attitude_setpoint();

        float q[4];

        mavlink_euler_to_quaternion(
            radians(attitude_setpoint.roll_deg),
            radians(attitude_setpoint.pitch_deg),
            radians(attitude_setpoint.yaw_deg),
            q);

        if (attitude_setpoint.mode == AttitudeData::Mode::Lock) {
            flags |= GIMBAL_DEVICE_FLAGS_YAW_LOCK;
        }

        mavlink_message_t message;
        mavlink_msg_gimbal_device_set_attitude_pack(
            _mavlink_passthrough.get_our_sysid(),
            _mavlink_passthrough.get_our_compid(),
            &message,
            0, // broadcast
            0, // broadcast
            flags,
            q,
            radians(attitude_setpoint.roll_rate_deg),
            radians(attitude_setpoint.pitch_rate_deg),
            radians(attitude_setpoint.yaw_rate_deg));

        _mavlink_passthrough.send_message(message);
    }

    MavlinkPassthrough& _mavlink_passthrough;
    AttitudeData& _attitude_data;
    std::thread _thread;
    std::atomic<bool> _should_exit{false};
};

class Tester {
public:
    explicit Tester(AttitudeData& attitude_data) : _attitude_data(attitude_data) {}

    bool test_pitch_angle()
    {
        const auto gimbal_limits = _attitude_data.gimbal_limits();

        // If limits are infinity, use arbitrary value.
        const float pitch_min =
            (!std::isinf(gimbal_limits.pitch_min_deg) ? gimbal_limits.pitch_min_deg : -90.0f);
        const float pitch_max =
            (!std::isinf(gimbal_limits.pitch_max_deg) ? gimbal_limits.pitch_max_deg : 20.0f);

        std::stringstream limit_down;
        limit_down << "Tilt " << -pitch_min << " down";
        std::stringstream limit_up;
        limit_up << "Tilt " << pitch_max << " up";

        return test_pitch_yaw_angle("Look forward", 0.0f, 0.0f, AttitudeData::Mode::Follow) &&
               // FIXME: We assume that -45 degrees is possible.
               test_pitch_yaw_angle(
                   "Tilt 45 degrees down", -45.0f, 0.0f, AttitudeData::Mode::Follow) &&
               test_pitch_yaw_angle(
                   limit_down.str(), pitch_min, 0.0f, AttitudeData::Mode::Follow) &&
               test_pitch_yaw_angle(limit_up.str(), pitch_max, 0.0f, AttitudeData::Mode::Follow) &&
               test_pitch_yaw_angle("Look forward again", 0.0f, 0.0f, AttitudeData::Mode::Follow);
    }

    bool test_yaw_angle_follow()
    {
        if (!test_pitch_yaw_angle(
                "Switch to follow mode", 0.0f, 0.0f, AttitudeData::Mode::Follow)) {
            return false;
        }

        std::cout << test_prefix << "Yaw 20 degrees to the right... ";
        _attitude_data.change_vehicle_attitude([](AttitudeData::VehicleAttitude& vehicle_attitude) {
            vehicle_attitude.yaw_deg = 20.0f;
        });
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::cout << "DONE\n";

        const auto gimbal_limits = _attitude_data.gimbal_limits();

        const float yaw_min =
            (!std::isinf(gimbal_limits.yaw_min_deg) ? gimbal_limits.yaw_min_deg : -60.0f);
        const float yaw_max =
            (!std::isinf(gimbal_limits.yaw_max_deg) ? gimbal_limits.yaw_max_deg : 60.0f);

        std::stringstream limit_right;
        limit_right << "Pan " << yaw_max << " right";
        std::stringstream limit_left;
        limit_left << "Pan " << -yaw_min << " left";

        return test_pitch_yaw_angle(limit_right.str(), 0.0f, yaw_max, AttitudeData::Mode::Follow) &&
               test_pitch_yaw_angle(limit_left.str(), 0.0f, yaw_min, AttitudeData::Mode::Follow) &&
               // FIXME: We assume that -45 degrees is possible.
               test_pitch_yaw_angle(
                   "Tilt 45 degrees down", -45.0f, 0.0f, AttitudeData::Mode::Follow) &&
               test_pitch_yaw_angle(
                   limit_right.str(), -45.0f, yaw_max, AttitudeData::Mode::Follow) &&
               test_pitch_yaw_angle(
                   limit_left.str(), -45.0f, yaw_min, AttitudeData::Mode::Follow) &&
               test_pitch_yaw_angle("Look forward again", 0.0f, 0.0f, AttitudeData::Mode::Follow);
    }

    bool test_yaw_angle_lock()
    {
        if (!test_pitch_yaw_angle("Switch to lock mode", 0.0f, 0.0f, AttitudeData::Mode::Lock)) {
            return false;
        }

        std::cout << test_prefix << "Yaw 30 degrees to the left... ";
        _attitude_data.change_vehicle_attitude([](AttitudeData::VehicleAttitude& vehicle_attitude) {
            vehicle_attitude.yaw_deg = -30.0f;
        });
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::cout << "DONE\n";

        const auto gimbal_limits = _attitude_data.gimbal_limits();

        // Use a smaller range on both sides.
        const float yaw_min =
            (!std::isinf(gimbal_limits.yaw_min_deg) ? gimbal_limits.yaw_min_deg + 30.0f : -60.0f);
        const float yaw_max =
            (!std::isinf(gimbal_limits.yaw_max_deg) ? gimbal_limits.yaw_max_deg - 30.0f : 60.0f);

        std::stringstream limit_right;
        limit_right << "Pan " << yaw_max << " right";
        std::stringstream limit_left;
        limit_left << "Pan " << -yaw_min << " left";

        return test_pitch_yaw_angle(limit_right.str(), 0.0f, yaw_max, AttitudeData::Mode::Lock) &&
               test_pitch_yaw_angle(limit_left.str(), 0.0f, yaw_min, AttitudeData::Mode::Lock) &&
               // FIXME: We assume that -45 degrees is possible.
               test_pitch_yaw_angle(
                   "Tilt 45 degrees down", -45.0f, 0.0f, AttitudeData::Mode::Lock) &&
               test_pitch_yaw_angle(limit_right.str(), -45.0f, yaw_max, AttitudeData::Mode::Lock) &&
               test_pitch_yaw_angle(limit_left.str(), -45.0f, yaw_min, AttitudeData::Mode::Lock) &&
               test_pitch_yaw_angle("Look forward again", 0.0f, 0.0f, AttitudeData::Mode::Lock);
    }

    bool test_pitch_rate()
    {
        // FIXME: We assume that -50 degrees is possible.

        return test_pitch_yaw_angle("Look forward first", 0.0f, 0.0f, AttitudeData::Mode::Follow) &&
               test_pitch_yaw_rate("Tilt down with 10 deg/s for 5s", -10.0f, 0.0f, 5.0f) &&
               test_pitch_yaw_rate("Tilt back up with 20 deg/s for 2.5s", 20.0f, 0.0f, 2.5f) &&
               test_pitch_yaw_angle("Look forward again", 0.0f, 0.0f, AttitudeData::Mode::Follow);
    }

    bool test_pitch_angle_and_rate()
    {
        // FIXME: We assume that -20 degrees is possible.

        return test_pitch_yaw_angle("Look forward first", 0.0f, 0.0f, AttitudeData::Mode::Follow) &&
               test_pitch_angle_and_rate(
                   "Tilt down with 10 deg/s until 20 deg",
                   -10.0f,
                   -20.0f,
                   AttitudeData::Mode::Follow) &&
               test_pitch_angle_and_rate(
                   "Tilt back up with 5 deg/s", 5.0f, 0.0f, AttitudeData::Mode::Follow) &&
               test_pitch_yaw_angle("Look forward again", 0.0f, 0.0f, AttitudeData::Mode::Follow);
    }

    bool test_yaw_rate()
    {
        // FIXME: We assume that +/-25 degrees is possible.

        return test_pitch_yaw_angle("Look forward first", 0.0f, 0.0f, AttitudeData::Mode::Follow) &&
               test_pitch_yaw_rate("Pan right 5 deg/s for 5s", 0.0f, 5.0f, 5.0f) &&
               test_pitch_yaw_rate("Pan to the right with 10 deg/s for 5s", 0.0f, -10.0f, 5.0f) &&
               test_pitch_yaw_angle("Look forward again", 0.0f, 0.0f, AttitudeData::Mode::Follow);
    }

    bool test_pitch_yaw_angle(
        const std::string& description, float pitch_deg, float yaw_deg, AttitudeData::Mode mode)
    {
        std::cout << test_prefix << description << "... " << std::flush;

        _attitude_data.change_attitude_setpoint(
            [&](AttitudeData::AttitudeSetpoint& attitude_setpoint) {
                attitude_setpoint.roll_deg = 0.0f;
                attitude_setpoint.pitch_deg = pitch_deg;
                attitude_setpoint.yaw_deg = yaw_deg;
                attitude_setpoint.roll_rate_deg = NAN;
                attitude_setpoint.pitch_rate_deg = NAN;
                attitude_setpoint.yaw_rate_deg = NAN;
                attitude_setpoint.mode = mode;
            });

        std::this_thread::sleep_for(std::chrono::seconds(2));

        const float margin_deg = 2.0f;

        if (mode == AttitudeData::Mode::Follow) {
            const auto vehicle_attitude = _attitude_data.vehicle_attitude();
            yaw_deg += vehicle_attitude.yaw_deg;
        }

        const auto gimbal_attitude = _attitude_data.gimbal_attitude();

        bool pitch_fail = false;
        bool yaw_fail = false;

        // TODO: We should do this check in quaternion to avoid gimbal locks.
        //       For now we avoid the check close to it.

        if (gimbal_attitude.pitch_deg > pitch_deg + margin_deg) {
            pitch_fail = true;
        } else if (gimbal_attitude.pitch_deg < pitch_deg - margin_deg) {
            pitch_fail = true;
        }

        if (gimbal_attitude.pitch_deg < 80.0f && gimbal_attitude.pitch_deg > -80.0) {
            if (gimbal_attitude.yaw_deg > yaw_deg + margin_deg) {
                yaw_fail = true;
            } else if (gimbal_attitude.yaw_deg < yaw_deg - margin_deg) {
                yaw_fail = true;
            }
        }

        if (pitch_fail || yaw_fail) {
            std::cout << "FAIL\n";
        } else {
            std::cout << "PASS\n";
        }

        if (pitch_fail) {
            std::cout << "-> pitch is " << gimbal_attitude.pitch_deg << " deg instead of "
                      << pitch_deg << " deg\n";
        }

        if (yaw_fail) {
            std::cout << "-> yaw is " << gimbal_attitude.yaw_deg << " deg instead of " << yaw_deg
                      << " deg\n";
        }

        return !(pitch_fail || yaw_fail);
    }

    bool test_pitch_yaw_rate(
        const std::string& description, float pitch_rate_deg, float yaw_rate_deg, float duration_s)
    {
        std::cout << test_prefix << description << "... " << std::flush;

        const auto initial_attitude = _attitude_data.gimbal_attitude();

        _attitude_data.change_attitude_setpoint(
            [&](AttitudeData::AttitudeSetpoint& attitude_setpoint) {
                attitude_setpoint.roll_deg = NAN;
                attitude_setpoint.pitch_deg = NAN;
                attitude_setpoint.yaw_deg = NAN;
                attitude_setpoint.roll_rate_deg = 0.0f;
                attitude_setpoint.pitch_rate_deg = pitch_rate_deg;
                attitude_setpoint.yaw_rate_deg = yaw_rate_deg;
                attitude_setpoint.mode = AttitudeData::Mode::Follow;
            });

        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<unsigned>(duration_s * 1000.0f)));

        const auto new_attitude = _attitude_data.gimbal_attitude();

        float expected_pitch_deg = initial_attitude.pitch_deg + pitch_rate_deg * duration_s;
        float expected_yaw_deg = initial_attitude.yaw_deg + yaw_rate_deg * duration_s;

        // TODO: check for wrap-arounds.

        bool pitch_fail = false;
        bool yaw_fail = false;

        const float margin_deg = 5.0f;

        if (new_attitude.pitch_deg > expected_pitch_deg + margin_deg) {
            pitch_fail = true;
        } else if (new_attitude.pitch_deg < expected_pitch_deg - margin_deg) {
            pitch_fail = true;
        }

        if (new_attitude.yaw_deg > expected_yaw_deg + margin_deg) {
            yaw_fail = true;
        } else if (new_attitude.yaw_deg < expected_yaw_deg - margin_deg) {
            yaw_fail = true;
        }

        if (pitch_fail || yaw_fail) {
            std::cout << "FAIL\n";
        } else {
            std::cout << "PASS\n";
        }

        if (pitch_fail) {
            std::cout << "-> pitch is " << new_attitude.pitch_deg << " deg instead of "
                      << expected_pitch_deg << " deg\n";
        }

        if (yaw_fail) {
            std::cout << "-> yaw is " << new_attitude.yaw_deg << " deg instead of "
                      << expected_yaw_deg << " deg\n";
        }

        return !(pitch_fail || yaw_fail);
    }

    bool test_pitch_angle_and_rate(
        const std::string& description,
        float pitch_rate_deg,
        float pitch_deg,
        AttitudeData::Mode mode)
    {
        // FIXME: this only works when started from 0.

        std::cout << test_prefix << description << "... " << std::flush;

        const auto initial_attitude = _attitude_data.gimbal_attitude();

        const auto time_needed_s = pitch_deg / pitch_rate_deg;

        _attitude_data.change_attitude_setpoint(
            [&](AttitudeData::AttitudeSetpoint& attitude_setpoint) {
                attitude_setpoint.roll_deg = 0.0f;
                attitude_setpoint.pitch_deg = pitch_deg;
                attitude_setpoint.yaw_deg = 0.0f;
                attitude_setpoint.roll_rate_deg = 0.0f;
                attitude_setpoint.pitch_rate_deg = pitch_rate_deg;
                attitude_setpoint.yaw_rate_deg = 0.0f;
                attitude_setpoint.mode = mode;
            });

        // We wait for half the time, then check to assess if the speed is corrct.
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<unsigned>(time_needed_s / 2.0f * 1000.0f)));

        bool halftime_fail = false;
        const auto halftime_attitude = _attitude_data.gimbal_attitude();
        const float halftime_expected_pitch_deg =
            initial_attitude.pitch_deg + pitch_rate_deg * time_needed_s / 2.0f;

        const float margin_deg = 3.0f;

        if (halftime_attitude.pitch_deg > halftime_expected_pitch_deg + margin_deg) {
            halftime_fail = true;
        } else if (halftime_attitude.pitch_deg < halftime_expected_pitch_deg - margin_deg) {
            halftime_fail = true;
        }

        // Then we wait longer to let it finish and add some margin.
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<unsigned>(time_needed_s / 2.0f * 1000.0f) + 1));

        bool end_fail = false;
        const auto end_attitude = _attitude_data.gimbal_attitude();
        const float end_expected_pitch_deg =
            initial_attitude.pitch_deg + pitch_rate_deg * time_needed_s;

        if (end_attitude.pitch_deg > end_expected_pitch_deg + margin_deg) {
            end_fail = true;
        } else if (end_attitude.pitch_deg < end_expected_pitch_deg - margin_deg) {
            end_fail = true;
        }

        if (halftime_fail || end_fail) {
            std::cout << "FAIL\n";
        } else {
            std::cout << "PASS\n";
        }

        if (halftime_fail) {
            std::cout << "-> pitch is " << halftime_attitude.pitch_deg << " deg instead of "
                      << halftime_expected_pitch_deg << " at halftime deg\n";
        }

        if (end_fail) {
            std::cout << "-> pitch is " << end_attitude.pitch_deg << " deg instead of "
                      << end_expected_pitch_deg << " at end deg\n";
        }

        return !(halftime_fail || end_fail);
    }

private:
    AttitudeData& _attitude_data;
};

bool wait_for_yaw_estimator_to_converge(const AttitudeData& attitude_data)
{
    std::cout << test_prefix << "Waiting for yaw estimator to converge..." << std::flush;
    for (unsigned i = 0; i < 200; ++i) {
        const auto gimbal_attitude = attitude_data.gimbal_attitude();
        if (gimbal_attitude.yaw_deg < 1.0f && gimbal_attitude.yaw_deg > -1.0f) {
            std::cout << "PASS" << std::endl;
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "FAIL\n";
    std::cout << "-> timeout waiting for yaw to converge to 0" << std::endl;

    return false;
}

bool request_gimbal_device_information(MavlinkPassthrough& mavlink_passthrough)
{
    MavlinkPassthrough::CommandLong command;
    command.command = MAV_CMD_REQUEST_MESSAGE;
    command.param1 = static_cast<float>(MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION);
    command.target_sysid = mavlink_passthrough.get_target_sysid();
    command.target_compid = mavlink_passthrough.get_target_compid();

    return (mavlink_passthrough.send_command_long(command) == MavlinkPassthrough::Result::Success);
}

bool test_device_information(MavlinkPassthrough& mavlink_passthrough, AttitudeData& attitude_data)
{
    std::cout << test_prefix << "Requests gimbal device information... " << std::flush;

    std::promise<void> prom;
    std::future<void> fut = prom.get_future();
    mavlink_passthrough.subscribe_message_async(
        MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION,
        [&prom, &mavlink_passthrough, &attitude_data](const mavlink_message_t& message) {
            mavlink_gimbal_device_information_t information;
            mavlink_msg_gimbal_device_information_decode(&message, &information);

            attitude_data.change_gimbal_limits(
                [&information](AttitudeData::GimbalLimits& gimbal_limits) {
                    // TODO: add these once they exist
                    // gimbal_limits.roll_min_deg = degrees(information.bank_min);
                    // gimbal_limits.roll_max_deg = degrees(information.bank_max);
                    gimbal_limits.pitch_min_deg = degrees(information.pitch_min);
                    gimbal_limits.pitch_max_deg = degrees(information.pitch_max);
                    gimbal_limits.yaw_min_deg = degrees(information.yaw_min);
                    gimbal_limits.yaw_max_deg = degrees(information.yaw_max);
                });

            // We only need it once.
            mavlink_passthrough.subscribe_message_async(
                MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION, nullptr);
            prom.set_value();
        });

    if (!request_gimbal_device_information(mavlink_passthrough)) {
        std::cout << "FAIL\n";
        std::cout << "-> could not request gimbal device information" << std::endl;
        return false;
    }

    if (fut.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
        std::cout << "FAIL\n";
        std::cout << "-> no gimbal device information received" << std::endl;
        return false;
    }

    std::cout << "PASS" << std::endl;
    return true;
}

void subscribe_to_gimbal_device_attitude_status(
    MavlinkPassthrough& mavlink_passthrough, AttitudeData& attitude_data)
{
    mavlink_passthrough.subscribe_message_async(
        MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS,
        [&attitude_data](const mavlink_message_t& message) {
            mavlink_gimbal_device_attitude_status_t attitude_status;
            mavlink_msg_gimbal_device_attitude_status_decode(&message, &attitude_status);

            float roll_rad, pitch_rad, yaw_rad;
            mavlink_quaternion_to_euler(attitude_status.q, &roll_rad, &pitch_rad, &yaw_rad);

            attitude_data.change_gimbal_attitude(
                [&](AttitudeData::GimbalAttitude& gimbal_attitude) {
                    gimbal_attitude.roll_deg = degrees(roll_rad);
                    gimbal_attitude.pitch_deg = degrees(pitch_rad);
                    gimbal_attitude.yaw_deg = degrees(yaw_rad);
                });
        });
}

void usage(const std::string& bin_name)
{
    std::cout << "Usage : " << bin_name << " <connection_url>" << std::endl
              << "Connection URL format should be :" << std::endl
              << " For TCP : tcp://[server_host][:server_port]" << std::endl
              << " For UDP : udp://[bind_host][:bind_port]" << std::endl
              << " For Serial : serial:///path/to/serial/dev[:baudrate]" << std::endl
              << "For example, to connect to the simulator use URL: udp://:14540" << std::endl;
}

int main(int argc, char** argv)
{
    Mavsdk mavsdk;
    std::string connection_url;
    ConnectionResult connection_result;

    std::cout << test_prefix << "Connecting... " << std::flush;

    if (argc == 2) {
        connection_url = argv[1];
        connection_result = mavsdk.add_any_connection(connection_url);
    } else {
        usage(argv[0]);
        return 1;
    }

    if (connection_result != ConnectionResult::Success) {
        std::cout << "FAIL\n";
        std::cout << "-> connection failed: " << connection_result << std::endl;
        return 1;
    }

    {
        std::promise<void> prom;
        std::future<void> fut = prom.get_future();
        mavsdk.subscribe_on_new_system([&prom]() { prom.set_value(); });

        if (fut.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
            std::cout << "FAIL\n";
            std::cout << "-> no device found" << std::endl;
            return 1;
        }
        mavsdk.subscribe_on_new_system(nullptr);
    }

    auto system = mavsdk.systems().at(0);
    MavlinkPassthrough mavlink_passthrough(system);

    AttitudeData attitude_data{};

    if (!test_device_information(mavlink_passthrough, attitude_data)) {
        return 1;
    }

    subscribe_to_gimbal_device_attitude_status(mavlink_passthrough, attitude_data);

    Sender sender(mavlink_passthrough, attitude_data);

    if (!wait_for_yaw_estimator_to_converge(attitude_data)) {
        return 1;
    }

    Tester tester(attitude_data);

    if (!tester.test_pitch_angle()) {
        return 1;
    }

    if (!tester.test_yaw_angle_follow()) {
        return 1;
    }

    if (!tester.test_yaw_angle_lock()) {
        return 1;
    }

    if (!tester.test_pitch_rate()) {
        return 1;
    }

    if (!tester.test_yaw_rate()) {
        return 1;
    }

    if (!tester.test_pitch_angle_and_rate()) {
        return 1;
    }

    return 0;
}
