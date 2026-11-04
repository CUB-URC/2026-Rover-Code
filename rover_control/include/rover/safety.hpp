// Link-loss watchdog. Time is passed in explicitly (seconds, monotonic clock)
// so the logic is deterministic and easy to unit test.
#pragma once

namespace rover {

class Watchdog {
public:
    explicit Watchdog(double timeout_s) : timeout_s_(timeout_s) {}

    void feed(double now_s) {
        last_s_ = now_s;
        fed_ = true;
    }
    // Expired if never fed, or if more than timeout_s has passed since the last feed.
    bool expired(double now_s) const { return !fed_ || (now_s - last_s_) > timeout_s_; }
    double timeout() const { return timeout_s_; }
    double since_last(double now_s) const { return fed_ ? now_s - last_s_ : -1.0; }

private:
    double timeout_s_;
    double last_s_ = 0.0;
    bool fed_ = false;
};

}  // namespace rover
