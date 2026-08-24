#pragma once

#include <cmath>
#include <cstdint>

namespace booster_soccer::score_run {

enum class State
{
    Inactive,
    Running,
    LossGrace,
    EntryRejected,
    ConfigInvalid,
    Interrupted,
    ClockInvalid,
    MaxDuration,
    BallLost,
};

inline const char *toString(State state)
{
    switch (state)
    {
    case State::Inactive: return "INACTIVE";
    case State::Running: return "RUNNING";
    case State::LossGrace: return "LOSS_GRACE";
    case State::EntryRejected: return "ENTRY_REJECTED";
    case State::ConfigInvalid: return "CONFIG_INVALID";
    case State::Interrupted: return "INTERRUPTED";
    case State::ClockInvalid: return "CLOCK_INVALID";
    case State::MaxDuration: return "MAX_DURATION";
    case State::BallLost: return "BALL_LOST";
    }
    return "UNKNOWN";
}

struct Config
{
    double ballLostGraceMsec = 800.0;
    double maxDurationMsec = 2500.0;
    double maxTickGapMsec = 300.0;
};

struct Input
{
    bool active = false;
    bool entryEligible = false;
    bool freshVisualBall = false;
    std::int64_t nowNsec = 0;
    std::int64_t observationNsec = 0;
    int nowClockDomain = 0;
    int observationClockDomain = 0;
};

struct Result
{
    State state = State::Inactive;
    bool running = false;
    bool drive = false;
    bool done = false;
    double elapsedMsec = 0.0;
    double lostMsec = 0.0;
};

class Controller
{
public:
    Result update(const Input &input, Config config)
    {
        const bool configValid =
            validDuration(config.ballLostGraceMsec) &&
            validDuration(config.maxDurationMsec) &&
            validDuration(config.maxTickGapMsec);
        config.ballLostGraceMsec = sanitizeDuration(config.ballLostGraceMsec);
        config.maxDurationMsec = sanitizeDuration(config.maxDurationMsec);
        config.maxTickGapMsec = sanitizeDuration(config.maxTickGapMsec);

        if (!input.active)
        {
            reset();
            return {};
        }

        if (!configValid)
        {
            finish(State::ConfigInvalid);
            Result invalid;
            invalid.state = State::ConfigInvalid;
            invalid.done = true;
            return invalid;
        }

        Result result;
        result.state = _finished ? _terminalState : State::EntryRejected;

        if (_running && _hasLastTick)
        {
            const bool compatibleClock = input.nowClockDomain == _lastTickClockDomain;
            const double tickGapMsec = compatibleClock
                ? differenceMsec(input.nowNsec, _lastTickNsec)
                : -1.0;
            if (!compatibleClock || !std::isfinite(tickGapMsec) ||
                tickGapMsec < 0.0 || tickGapMsec > config.maxTickGapMsec)
            {
                finish(State::Interrupted);
            }
        }
        _lastTickNsec = input.nowNsec;
        _lastTickClockDomain = input.nowClockDomain;
        _hasLastTick = true;

        if (!_running && !_finished)
        {
            const bool entryClockValid =
                input.nowClockDomain == input.observationClockDomain &&
                input.observationNsec <= input.nowNsec;
            if (input.entryEligible && input.freshVisualBall && entryClockValid)
            {
                _running = true;
                _startNsec = input.nowNsec;
                _lastObservationNsec = input.observationNsec;
                _clockDomain = input.nowClockDomain;
            }
            else
            {
                finish(State::EntryRejected);
            }
        }

        if (_running)
        {
            if (input.freshVisualBall &&
                input.observationClockDomain == _clockDomain &&
                input.observationNsec > _lastObservationNsec)
            {
                _lastObservationNsec = input.observationNsec;
            }

            const bool compatibleClock = input.nowClockDomain == _clockDomain;
            result.elapsedMsec = compatibleClock
                ? differenceMsec(input.nowNsec, _startNsec)
                : -1.0;
            result.lostMsec = compatibleClock
                ? differenceMsec(input.nowNsec, _lastObservationNsec)
                : -1.0;

            const bool clockInvalid =
                !compatibleClock || !std::isfinite(result.elapsedMsec) ||
                !std::isfinite(result.lostMsec) ||
                result.elapsedMsec < 0.0 || result.lostMsec < 0.0;
            if (clockInvalid)
            {
                finish(State::ClockInvalid);
            }
            else if (result.elapsedMsec >= config.maxDurationMsec)
            {
                finish(State::MaxDuration);
            }
            else if (result.lostMsec >= config.ballLostGraceMsec)
            {
                finish(State::BallLost);
            }
            else
            {
                result.state = input.freshVisualBall ? State::Running : State::LossGrace;
                result.running = true;
                result.drive = true;
                return result;
            }
        }

        result.state = _terminalState;
        result.done = _finished;
        return result;
    }

    void reset()
    {
        _running = false;
        _finished = false;
        _hasLastTick = false;
        _startNsec = 0;
        _lastObservationNsec = 0;
        _lastTickNsec = 0;
        _clockDomain = 0;
        _lastTickClockDomain = 0;
        _terminalState = State::Inactive;
    }

private:
    static double sanitizeDuration(double value)
    {
        return validDuration(value) ? value : 0.0;
    }

    static bool validDuration(double value)
    {
        return std::isfinite(value) && value > 0.0;
    }

    static double differenceMsec(std::int64_t newer, std::int64_t older)
    {
        return static_cast<double>(
            (static_cast<long double>(newer) - static_cast<long double>(older)) /
            1000000.0L);
    }

    void finish(State state)
    {
        _running = false;
        _finished = true;
        _terminalState = state;
    }

    bool _running = false;
    bool _finished = false;
    bool _hasLastTick = false;
    std::int64_t _startNsec = 0;
    std::int64_t _lastObservationNsec = 0;
    std::int64_t _lastTickNsec = 0;
    int _clockDomain = 0;
    int _lastTickClockDomain = 0;
    State _terminalState = State::Inactive;
};

} // namespace booster_soccer::score_run
