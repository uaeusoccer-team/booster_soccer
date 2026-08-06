#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace booster_soccer::occlusion
{

struct TimedBallPoint
{
    std::int64_t timeMsec{0};
    double odomX{0.0};
    double odomY{0.0};
};

struct StationaryBallEstimate
{
    bool valid{false};
    double odomX{0.0};
    double odomY{0.0};
    double spread{std::numeric_limits<double>::infinity()};
    double speed{std::numeric_limits<double>::infinity()};
};

inline StationaryBallEstimate estimateStationaryBall(
    const std::vector<TimedBallPoint> &history,
    std::int64_t nowMsec,
    std::size_t minimumSamples = 5,
    std::int64_t minimumSpanMsec = 300,
    std::int64_t maximumAgeMsec = 500,
    double maximumSpread = 0.10,
    double maximumSpeed = 0.15) noexcept
{
    StationaryBallEstimate result;
    if (history.size() < minimumSamples) return result;
    if (maximumAgeMsec < 0) return result;
    const auto &last = history.back();
    if (last.timeMsec > nowMsec || nowMsec - last.timeMsec > maximumAgeMsec) {
        return result;
    }

    // Estimate stationarity only from the recent observation window. Using an
    // older first point can dilute a fast movement immediately before loss and
    // incorrectly turn a moving ball into a stale pursuit target.
    std::size_t firstIndex = 0;
    while (firstIndex < history.size() &&
           nowMsec - history[firstIndex].timeMsec > maximumAgeMsec) {
        ++firstIndex;
    }
    if (history.size() - firstIndex < minimumSamples) return result;
    const auto &first = history[firstIndex];
    const std::int64_t spanMsec = last.timeMsec - first.timeMsec;
    if (spanMsec < minimumSpanMsec || spanMsec <= 0) return result;

    double sumX = 0.0;
    double sumY = 0.0;
    double spread = 0.0;
    double localSpeed = 0.0;
    constexpr std::int64_t kMinimumVelocitySpanMsec = 75;
    constexpr double kPositionNoiseAllowance = 0.01;
    for (std::size_t i = firstIndex; i < history.size(); ++i) {
        if (i > firstIndex && history[i].timeMsec <= history[i - 1].timeMsec) {
            return result;
        }
        if (!std::isfinite(history[i].odomX) || !std::isfinite(history[i].odomY)) {
            return result;
        }
        sumX += history[i].odomX;
        sumY += history[i].odomY;
        for (std::size_t j = i + 1; j < history.size(); ++j) {
            const double displacement = std::hypot(
                history[i].odomX - history[j].odomX,
                history[i].odomY - history[j].odomY);
            spread = std::max(spread, displacement);
            const std::int64_t velocitySpanMsec =
                history[j].timeMsec - history[i].timeMsec;
            if (velocitySpanMsec >= kMinimumVelocitySpanMsec) {
                localSpeed = std::max(
                    localSpeed,
                    std::max(0.0, displacement - kPositionNoiseAllowance) /
                        (static_cast<double>(velocitySpanMsec) / 1000.0));
            }
        }
    }
    const double seconds = static_cast<double>(spanMsec) / 1000.0;
    const double endpointSpeed = std::hypot(
        last.odomX - first.odomX,
        last.odomY - first.odomY) / seconds;
    const double speed = std::max(endpointSpeed, localSpeed);
    result.spread = spread;
    result.speed = speed;
    if (spread >= maximumSpread || speed >= maximumSpeed) return result;

    result.valid = true;
    const double selectedCount = static_cast<double>(history.size() - firstIndex);
    result.odomX = sumX / selectedCount;
    result.odomY = sumY / selectedCount;
    return result;
}

inline bool freshObstacleOccludesExpectedBall(
    bool freshObstacle,
    double expectedX,
    double expectedY,
    double obstacleX,
    double obstacleY,
    double obstacleRadius,
    double nearMargin = 0.15,
    double footprintPadding = 0.12,
    double bearingPaddingRad = 0.08726646259971647) noexcept
{
    if (!freshObstacle || !std::isfinite(expectedX) || !std::isfinite(expectedY) ||
        !std::isfinite(obstacleX) || !std::isfinite(obstacleY) ||
        !std::isfinite(obstacleRadius) || obstacleRadius < 0.0) return false;
    const double expectedRange = std::hypot(expectedX, expectedY);
    const double obstacleRange = std::hypot(obstacleX, obstacleY);
    if (expectedRange <= 1e-4 || obstacleRange <= 1e-4) return false;
    const double surfaceRange = std::max(0.0, obstacleRange - obstacleRadius);
    const double angularRadius = std::asin(std::clamp(
        (obstacleRadius + footprintPadding) / obstacleRange, 0.0, 1.0));
    double bearingError = std::atan2(obstacleY, obstacleX) -
        std::atan2(expectedY, expectedX);
    while (bearingError > 3.14159265358979323846) bearingError -= 6.28318530717958647692;
    while (bearingError < -3.14159265358979323846) bearingError += 6.28318530717958647692;
    return surfaceRange + nearMargin < expectedRange &&
        std::fabs(bearingError) <= angularRadius + bearingPaddingRad;
}

inline bool recoveryLimitExceeded(
    std::int64_t elapsedMsec,
    double displacementMeters) noexcept
{
    return elapsedMsec > 5000 || !std::isfinite(displacementMeters) ||
        displacementMeters > 1.0;
}

inline int updateClearViewMissCount(
    int currentCount,
    bool newDetectionFrame,
    bool ballVisible,
    bool freshDepth,
    bool expectedRayClear) noexcept
{
    if (ballVisible || !freshDepth || !expectedRayClear) return 0;
    return newDetectionFrame ? std::max(0, currentCount) + 1
                             : std::max(0, currentCount);
}

enum class BallEvidenceState
{
    VisibleDepth,
    VisibleRgbOnly,
    OccludedStationary,
    Lost,
    DetectionStale,
};

inline BallEvidenceState classifyBallEvidence(
    bool detectionStale,
    bool rgbVisible,
    bool depthVisible,
    bool expectedBallValid) noexcept
{
    if (detectionStale) return BallEvidenceState::DetectionStale;
    if (depthVisible) return BallEvidenceState::VisibleDepth;
    if (rgbVisible) return BallEvidenceState::VisibleRgbOnly;
    if (expectedBallValid) return BallEvidenceState::OccludedStationary;
    return BallEvidenceState::Lost;
}

} // namespace booster_soccer::occlusion
