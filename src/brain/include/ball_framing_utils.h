#pragma once

#include <algorithm>
#include <cmath>

namespace booster_soccer::ball_framing
{

struct VerticalTarget
{
    bool valid{false};
    double ballCenterY{0.0};
    double targetCenterY{0.0};
    double errorY{0.0};
};

/**
 * Put the ball below image centre while keeping its complete bounding box
 * inside a configurable bottom safety margin.
 */
inline VerticalTarget calculateVerticalTarget(
    double imageHeight,
    double bboxYMin,
    double bboxYMax,
    double targetYRatio,
    double bottomMarginPx,
    double verticalDeadbandPx = 0.0) noexcept
{
    VerticalTarget result;
    if (!std::isfinite(imageHeight) || imageHeight <= 0.0 ||
        !std::isfinite(bboxYMin) || !std::isfinite(bboxYMax) ||
        bboxYMax <= bboxYMin) {
        return result;
    }

    const double ratio = std::clamp(
        std::isfinite(targetYRatio) ? targetYRatio : 0.5,
        0.0,
        1.0);
    const double margin = std::clamp(
        std::isfinite(bottomMarginPx) ? bottomMarginPx : 0.0,
        0.0,
        imageHeight);
    const double deadband = std::clamp(
        std::isfinite(verticalDeadbandPx) ? std::fabs(verticalDeadbandPx) : 0.0,
        0.0,
        imageHeight);
    const double halfBoxHeight = 0.5 * (bboxYMax - bboxYMin);
    const double nominalTarget = imageHeight * ratio;
    // The controller is allowed to hold anywhere inside its deadband. Move
    // the safety cap upward by that full allowance so a ball at the lower
    // edge of the deadband still keeps its complete bbox above the margin.
    const double bottomSafeTarget =
        imageHeight - margin - deadband - halfBoxHeight;

    result.valid = true;
    result.ballCenterY = 0.5 * (bboxYMin + bboxYMax);
    result.targetCenterY = std::clamp(
        std::min(nominalTarget, bottomSafeTarget),
        0.0,
        imageHeight);
    result.errorY = result.ballCenterY - result.targetCenterY;
    return result;
}

inline bool touchesBottomEdge(
    double imageHeight,
    double bboxYMax,
    double edgeMarginPx) noexcept
{
    if (!std::isfinite(imageHeight) || imageHeight <= 0.0 ||
        !std::isfinite(bboxYMax) || !std::isfinite(edgeMarginPx)) {
        return false;
    }
    const double margin = std::clamp(edgeMarginPx, 0.0, imageHeight);
    return bboxYMax >= imageHeight - margin;
}

inline double nextPitch(
    double measuredPitch,
    double verticalErrorPx,
    double deadbandPx,
    double normalStepRad,
    double settleStepRad,
    double minPitch,
    double maxPitch) noexcept
{
    if (!std::isfinite(measuredPitch)) return measuredPitch;

    const double low = std::min(minPitch, maxPitch);
    const double high = std::max(minPitch, maxPitch);
    const double deadband = std::fabs(deadbandPx);
    if (!std::isfinite(verticalErrorPx) ||
        std::fabs(verticalErrorPx) <= deadband) {
        return std::clamp(measuredPitch, low, high);
    }

    constexpr double settleZonePx = 20.0;
    const double normalStep = std::fabs(normalStepRad);
    const double settleStep = std::fabs(settleStepRad);
    const double step = std::fabs(verticalErrorPx) <= deadband + settleZonePx
        ? settleStep
        : normalStep;
    const double requested = measuredPitch +
        (verticalErrorPx > 0.0 ? step : -step);
    return std::clamp(requested, low, high);
}

} // namespace booster_soccer::ball_framing
