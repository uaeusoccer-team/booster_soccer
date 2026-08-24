#include "score_run_controller.h"

#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

namespace {

using booster_soccer::score_run::Config;
using booster_soccer::score_run::Controller;
using booster_soccer::score_run::Input;
using booster_soccer::score_run::State;

constexpr std::int64_t ms(double value)
{
    return static_cast<std::int64_t>(value * 1000000.0);
}

void expect(bool condition, const std::string &message)
{
    if (!condition)
    {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

Input sample(double nowMsec, double observationMsec, bool fresh = true)
{
    Input input;
    input.active = true;
    input.entryEligible = true;
    input.freshVisualBall = fresh;
    input.nowNsec = ms(nowMsec);
    input.observationNsec = ms(observationMsec);
    input.nowClockDomain = 1;
    input.observationClockDomain = 1;
    return input;
}

void testEntryRejectionIsOneShot()
{
    Controller controller;
    Input input = sample(1000.0, 950.0);
    input.entryEligible = false;
    auto result = controller.update(input, {});
    expect(result.done && !result.drive, "unsafe score entry must finish without driving");
    expect(result.state == State::EntryRejected, "unsafe entry state");

    input.entryEligible = true;
    input.nowNsec = ms(1010.0);
    result = controller.update(input, {});
    expect(result.done && !result.drive, "an active rejected request must never arm later");
}

void testRepeatedFrameDoesNotExtendLossGrace()
{
    Controller controller;
    Config config;
    config.maxDurationMsec = 5000.0;
    config.maxTickGapMsec = 1000.0;
    expect(controller.update(sample(1000.0, 950.0), config).drive, "valid entry drives");

    auto repeated = sample(1749.0, 950.0, true);
    expect(controller.update(repeated, config).drive, "same frame remains inside grace");

    repeated.nowNsec = ms(1750.0);
    const auto expired = controller.update(repeated, config);
    expect(expired.done && !expired.drive, "same frame stops exactly at loss boundary");
    expect(expired.state == State::BallLost, "loss boundary state");
}

void testNewFrameExtendsLossGrace()
{
    Controller controller;
    Config config;
    config.maxDurationMsec = 5000.0;
    config.maxTickGapMsec = 1000.0;
    controller.update(sample(1000.0, 950.0), config);
    expect(controller.update(sample(1600.0, 1500.0), config).drive, "new frame extends grace");
    expect(controller.update(sample(2299.0, 1500.0), config).drive, "extended grace remains active");
    const auto expired = controller.update(sample(2300.0, 1500.0), config);
    expect(expired.state == State::BallLost && expired.done, "extended grace exact boundary");
}

void testDurationAndTickBoundaries()
{
    Controller durationController;
    Config durationConfig;
    durationConfig.maxTickGapMsec = 3000.0;
    durationController.update(sample(1000.0, 1000.0), durationConfig);
    auto duration = durationController.update(sample(3500.0, 3500.0), durationConfig);
    expect(duration.state == State::MaxDuration && duration.done, "maximum duration is inclusive");

    Controller gapController;
    Config gapConfig;
    gapConfig.maxDurationMsec = 5000.0;
    gapController.update(sample(1000.0, 1000.0), gapConfig);
    auto gap = gapController.update(sample(1301.0, 1301.0), gapConfig);
    expect(gap.state == State::Interrupted && gap.done && !gap.drive, "301 ms tick gap aborts");
}

void testInactiveResetAndReentry()
{
    Controller controller;
    Input rejected = sample(1000.0, 950.0);
    rejected.entryEligible = false;
    controller.update(rejected, {});

    Input inactive;
    inactive.active = false;
    const auto reset = controller.update(inactive, {});
    expect(reset.state == State::Inactive && !reset.done, "inactive resets one-shot state");

    const auto reentry = controller.update(sample(1100.0, 1090.0), {});
    expect(reentry.drive && !reentry.done, "fresh activation may enter after reset");
}

void testBackwardAndChangedClockStop()
{
    Controller backwardController;
    backwardController.update(sample(1000.0, 990.0), {});
    auto backward = backwardController.update(sample(999.0, 995.0), {});
    expect(backward.state == State::Interrupted && backward.done, "backward tick time aborts");

    Controller changedClockController;
    changedClockController.update(sample(1000.0, 990.0), {});
    Input changed = sample(1010.0, 1000.0);
    changed.nowClockDomain = 2;
    changed.observationClockDomain = 2;
    auto changedResult = changedClockController.update(changed, {});
    expect(changedResult.state == State::Interrupted && changedResult.done, "clock-domain change aborts");
}

void testInvalidTimingFailsClosed()
{
    Controller negativeController;
    Config negative;
    negative.ballLostGraceMsec = -800.0;
    negative.maxDurationMsec = -2500.0;
    auto negativeResult = negativeController.update(sample(1000.0, 990.0), negative);
    expect(
        negativeResult.state == State::ConfigInvalid &&
        negativeResult.done && !negativeResult.drive,
        "negative timing configuration must fail closed");

    Controller nanController;
    Config nan;
    nan.maxDurationMsec = std::numeric_limits<double>::quiet_NaN();
    auto nanResult = nanController.update(sample(1000.0, 990.0), nan);
    expect(
        nanResult.state == State::ConfigInvalid && nanResult.done && !nanResult.drive,
        "NaN timing configuration must fail closed");
}

} // namespace

int main()
{
    testEntryRejectionIsOneShot();
    testRepeatedFrameDoesNotExtendLossGrace();
    testNewFrameExtendsLossGrace();
    testDurationAndTickBoundaries();
    testInactiveResetAndReentry();
    testBackwardAndChangedClockStop();
    testInvalidTimingFailsClosed();
    std::cout << "score_run_controller_test: PASS\n";
    return EXIT_SUCCESS;
}
