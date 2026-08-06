#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace shooting_adjust {

enum class GoalpostSide
{
    Unknown,
    Left,
    Right,
};

struct GoalpostPixelObservation
{
    GoalpostSide side = GoalpostSide::Unknown;
    double pixelX = 0.0;
    double confidence = 0.0;
    bool eligible = false;
};

struct GoalCenterSelection
{
    bool valid = false;
    double centerX = 0.0;
    double leftX = 0.0;
    double rightX = 0.0;
    double separationPx = 0.0;
    std::size_t eligibleCount = 0;
    bool usedLabeledPair = false;
};

namespace detail {

inline bool observationIsValid(const GoalpostPixelObservation &observation, double imageWidth)
{
    return observation.eligible && std::isfinite(observation.pixelX) &&
           std::isfinite(observation.confidence) && imageWidth > 0.0 &&
           observation.pixelX >= 0.0 && observation.pixelX < imageWidth;
}

inline GoalCenterSelection makeSelection(
    const GoalpostPixelObservation &first,
    const GoalpostPixelObservation &second,
    std::size_t eligibleCount)
{
    GoalCenterSelection selection;
    selection.valid = true;
    selection.leftX = std::min(first.pixelX, second.pixelX);
    selection.rightX = std::max(first.pixelX, second.pixelX);
    selection.centerX = (selection.leftX + selection.rightX) * 0.5;
    selection.separationPx = selection.rightX - selection.leftX;
    selection.eligibleCount = eligibleCount;
    selection.usedLabeledPair = true;
    return selection;
}

} // namespace detail

/**
 * Select the visible goal opening from two post observations.
 *
 * Fail closed unless there is exactly one eligible opponent-left observation
 * and exactly one eligible opponent-right observation. Rejecting ambiguous or
 * unlabeled pairs prevents an own-goal post or duplicate detection from being
 * used as the shooting target.
 */
inline GoalCenterSelection selectGoalCenter(
    const std::vector<GoalpostPixelObservation> &observations,
    double imageWidth,
    double minSeparationPx)
{
    minSeparationPx = std::max(0.0, std::fabs(minSeparationPx));

    std::vector<const GoalpostPixelObservation *> eligible;
    eligible.reserve(observations.size());
    for (const auto &observation : observations)
    {
        if (detail::observationIsValid(observation, imageWidth))
        {
            eligible.push_back(&observation);
        }
    }

    GoalCenterSelection none;
    none.eligibleCount = eligible.size();
    if (eligible.size() != 2)
    {
        return none;
    }

    const GoalpostPixelObservation *left = nullptr;
    const GoalpostPixelObservation *right = nullptr;
    for (const auto *observation : eligible)
    {
        if (observation->side == GoalpostSide::Left)
        {
            if (left != nullptr)
            {
                return none;
            }
            left = observation;
        }
        else if (observation->side == GoalpostSide::Right)
        {
            if (right != nullptr)
            {
                return none;
            }
            right = observation;
        }
    }

    if (left == nullptr || right == nullptr ||
        std::fabs(left->pixelX - right->pixelX) < minSeparationPx)
    {
        return none;
    }
    return detail::makeSelection(*left, *right, eligible.size());
}

class AlignmentDeadband
{
public:
    void configure(double tolerancePx, double hysteresisPx)
    {
        tolerancePx_ = std::max(0.0, std::fabs(tolerancePx));
        hysteresisPx_ = std::max(0.0, std::fabs(hysteresisPx));
    }

    bool update(const std::optional<double> errorPx)
    {
        if (!errorPx.has_value() || !std::isfinite(*errorPx))
        {
            reset();
            return false;
        }

        const double absoluteError = std::fabs(*errorPx);
        if (!hasTarget_)
        {
            active_ = absoluteError > tolerancePx_;
            hasTarget_ = true;
        }
        else if (active_)
        {
            if (absoluteError <= tolerancePx_)
            {
                active_ = false;
            }
        }
        else if (absoluteError > tolerancePx_ + hysteresisPx_)
        {
            active_ = true;
        }
        return active_;
    }

    bool update(double errorPx)
    {
        return update(std::optional<double>(errorPx));
    }

    void reset()
    {
        hasTarget_ = false;
        active_ = false;
    }

    bool hasTarget() const { return hasTarget_; }
    bool active() const { return active_; }
    bool aligned() const { return hasTarget_ && !active_; }

private:
    double tolerancePx_ = 0.0;
    double hysteresisPx_ = 0.0;
    bool hasTarget_ = false;
    bool active_ = false;
};

inline double lateralVelocityFromPixelError(
    double errorPx,
    double imageWidth,
    double gain,
    double limit,
    AlignmentDeadband &deadband)
{
    if (!deadband.update(errorPx) || !std::isfinite(imageWidth) || imageWidth <= 0.0 ||
        !std::isfinite(gain) || !std::isfinite(limit))
    {
        return 0.0;
    }

    const double absoluteLimit = std::fabs(limit);
    const double command = errorPx / imageWidth * std::max(0.0, gain);
    return std::clamp(command, -absoluteLimit, absoluteLimit);
}

} // namespace shooting_adjust
