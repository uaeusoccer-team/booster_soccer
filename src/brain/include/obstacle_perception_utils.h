#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace booster_soccer::perception
{

inline bool headPoseAgeAccepted(double ageMsecs, double toleranceMsecs) noexcept
{
    return std::isfinite(ageMsecs) && std::isfinite(toleranceMsecs) &&
        ageMsecs >= 0.0 && toleranceMsecs >= 0.0 && ageMsecs <= toleranceMsecs;
}

inline bool depthFrameHasEnoughSamples(int validSamples, int totalSampleSlots) noexcept
{
    if (validSamples < 0 || totalSampleSlots <= 0) return false;
    // A globally sparse/dropout frame must not manufacture isolated "clear"
    // rays. Ten percent is still tolerant of sky/edge invalidity while failing
    // closed on near-empty frames.
    return validSamples >= std::max(10, totalSampleSlots / 10);
}

inline bool angularBinObserved(int validSamples, int minimumSamples = 3) noexcept
{
    return validSamples >= std::max(1, minimumSamples);
}

inline bool planarRayFromCameraOrigin(
    double pointX,
    double pointY,
    double cameraOriginX,
    double cameraOriginY,
    double &bearing,
    double &range) noexcept
{
    if (!std::isfinite(pointX) || !std::isfinite(pointY) ||
        !std::isfinite(cameraOriginX) || !std::isfinite(cameraOriginY)) {
        bearing = 0.0;
        range = 0.0;
        return false;
    }
    const double rayX = pointX - cameraOriginX;
    const double rayY = pointY - cameraOriginY;
    range = std::hypot(rayX, rayY);
    if (!(range > 0.05)) {
        bearing = 0.0;
        return false;
    }
    bearing = std::atan2(rayY, rayX);
    return std::isfinite(bearing);
}

inline double obstacleGridMinimumX(double depthMapRange) noexcept
{
    // Preserve every observable point within the configured radial planning
    // horizon behind the robot centre. A person outside the current footprint
    // can still cross into a lateral route during the one-second prediction.
    return std::isfinite(depthMapRange) && depthMapRange > 0.0
        ? -depthMapRange
        : 0.0;
}

inline bool obstacleGridContains(
    double x,
    double y,
    double xMinimum,
    double xMaximum,
    double yMinimum,
    double yMaximum) noexcept
{
    return std::isfinite(x) && std::isfinite(y) &&
        std::isfinite(xMinimum) && std::isfinite(xMaximum) &&
        std::isfinite(yMinimum) && std::isfinite(yMaximum) &&
        x >= xMinimum && x < xMaximum && y >= yMinimum && y < yMaximum;
}

inline double wrappedAngularDistance(double left, double right) noexcept
{
    if (!std::isfinite(left) || !std::isfinite(right)) {
        return std::numeric_limits<double>::infinity();
    }
    constexpr double pi = 3.14159265358979323846;
    constexpr double twoPi = 2.0 * pi;
    double difference = left - right;
    while (difference > pi) difference -= twoPi;
    while (difference < -pi) difference += twoPi;
    return std::fabs(difference);
}

inline bool headTargetAttained(
    double measuredYaw,
    double targetYaw,
    double toleranceRad) noexcept
{
    return std::isfinite(toleranceRad) && toleranceRad >= 0.0 &&
        wrappedAngularDistance(measuredYaw, targetYaw) <= toleranceRad;
}

inline bool depthViewReadyToAdvance(
    std::int64_t nowMsec,
    std::int64_t attainedSinceMsec,
    std::int64_t latestDepthReceiveMsec,
    std::int64_t dwellMsec = 200) noexcept
{
    if (nowMsec < 0 || attainedSinceMsec < 0 ||
        latestDepthReceiveMsec < attainedSinceMsec || dwellMsec < 0) {
        return false;
    }
    return nowMsec - attainedSinceMsec >= dwellMsec;
}

inline double ballVisibleCoverageScanOffset(
    double horizontalFov,
    double ballHalfAngle,
    double maximumOffset) noexcept
{
    if (!std::isfinite(horizontalFov) || !std::isfinite(ballHalfAngle) ||
        !std::isfinite(maximumOffset) || horizontalFov <= 0.0 ||
        ballHalfAngle < 0.0 || maximumOffset < 0.0) {
        return 0.0;
    }
    // Keep the complete ball bounding box inside the nominal RGB/depth view.
    // Angular coverage bins add their calibrated half-bin at the sensor edge;
    // if this is still insufficient, the planner remains stopped.
    return std::min(
        maximumOffset,
        std::max(0.0, 0.5 * horizontalFov - ballHalfAngle));
}

inline double executableDetourScanOffset(
    double desiredForwardSpeed,
    double minimumForwardSpeed,
    double minimumLateralSpeed,
    double forwardLimit,
    double lateralLimit,
    double maximumOffset) noexcept
{
    constexpr double lateralCommandFloor = 0.30;
    if (!std::isfinite(desiredForwardSpeed) ||
        !std::isfinite(minimumForwardSpeed) ||
        !std::isfinite(minimumLateralSpeed) ||
        !std::isfinite(forwardLimit) || !std::isfinite(lateralLimit) ||
        !std::isfinite(maximumOffset) || minimumForwardSpeed < 0.0 ||
        minimumLateralSpeed < 0.0 || forwardLimit < 0.0 ||
        lateralLimit < 0.0 || maximumOffset < 0.0) {
        return 0.0;
    }
    const double lateral = std::max(
        lateralCommandFloor, minimumLateralSpeed);
    if (lateral > lateralLimit || maximumOffset == 0.0) return 0.0;
    const double forward = std::min(
        forwardLimit,
        std::max(std::fabs(desiredForwardSpeed), minimumForwardSpeed));
    if (forward <= 1e-9) return maximumOffset;
    return std::min(maximumOffset, std::atan2(lateral, forward));
}

inline bool obstacleHeightAccepted(
    double z, double minimumHeight, double maximumHeight) noexcept
{
    return std::isfinite(z) && std::isfinite(minimumHeight) &&
        std::isfinite(maximumHeight) && minimumHeight <= maximumHeight &&
        z >= minimumHeight && z <= maximumHeight;
}

inline bool componentEvidenceAccepted(
    std::size_t occupiedCellCount,
    int strongestCellSamples,
    int strongCellThreshold) noexcept
{
    return occupiedCellCount >= 2 ||
        (strongestCellSamples >= std::max(1, strongCellThreshold));
}

inline bool physicalBallSample(
    bool maskEnabled,
    double pixelX,
    double pixelY,
    double boxXMin,
    double boxXMax,
    double boxYMin,
    double boxYMax,
    double pointX,
    double pointY,
    double pointZ,
    double ballX,
    double ballY,
    double ballRadius,
    double ballHeight) noexcept
{
    if (!maskEnabled || !std::isfinite(pixelX) || !std::isfinite(pixelY) ||
        !std::isfinite(pointX) || !std::isfinite(pointY) || !std::isfinite(pointZ) ||
        !std::isfinite(ballX) || !std::isfinite(ballY) ||
        !std::isfinite(ballRadius) || !std::isfinite(ballHeight) ||
        ballRadius < 0.0 || ballHeight < -0.05) {
        return false;
    }
    const bool insidePixels = pixelX >= boxXMin && pixelX <= boxXMax &&
        pixelY >= boxYMin && pixelY <= boxYMax;
    return insidePixels &&
        std::hypot(pointX - ballX, pointY - ballY) <= ballRadius &&
        pointZ >= -0.05 && pointZ <= ballHeight;
}

inline bool physicalBallComponent(
    bool maskEnabled,
    std::size_t occupiedCellCount,
    int strongestCellSamples,
    int strongCellThreshold,
    double componentRadius,
    double centerError,
    double ballRadius,
    double gridCellPadding) noexcept
{
    return maskEnabled &&
        componentEvidenceAccepted(
            occupiedCellCount, strongestCellSamples, strongCellThreshold) &&
        std::isfinite(componentRadius) && componentRadius >= 0.0 &&
        std::isfinite(centerError) && centerError >= 0.0 &&
        std::isfinite(ballRadius) && ballRadius > 0.0 &&
        std::isfinite(gridCellPadding) && gridCellPadding >= 0.0 &&
        componentRadius <= ballRadius + gridCellPadding &&
        centerError <= std::max(0.5 * ballRadius, gridCellPadding);
}

inline double combinedFreshnessAge(
    double receiveAgeMsecs,
    double sensorAgeMsecs,
    bool sensorAgeValid) noexcept
{
    if (!std::isfinite(receiveAgeMsecs)) {
        return std::numeric_limits<double>::infinity();
    }
    // A missing, zero, or wrong-clock sensor stamp cannot prove that repeated
    // callback payloads are advancing. Fail closed instead of allowing a
    // replayed frame to refresh itself forever through receive time alone.
    if (!sensorAgeValid) {
        return std::numeric_limits<double>::infinity();
    }
    // A same-clock sensor stamp is authoritative. A future/corrupt stamp must
    // fail closed instead of being hidden by a fresh callback receive time.
    if (!std::isfinite(sensorAgeMsecs)) {
        return std::numeric_limits<double>::infinity();
    }
    return std::max({0.0, receiveAgeMsecs, sensorAgeMsecs});
}

inline bool observationWithinMemory(double ageMsecs, double memoryMsecs) noexcept
{
    return std::isfinite(ageMsecs) && std::isfinite(memoryMsecs) &&
        ageMsecs >= 0.0 && memoryMsecs >= 0.0 && ageMsecs <= memoryMsecs;
}

} // namespace booster_soccer::perception
