#include "ball_framing_utils.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace
{
void require(bool condition, const std::string &message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

void requireNear(double actual, double expected, const std::string &message)
{
    require(std::fabs(actual - expected) < 1e-9, message);
}
} // namespace

int main()
{
    using booster_soccer::ball_framing::calculateVerticalTarget;
    using booster_soccer::ball_framing::nextPitch;
    using booster_soccer::ball_framing::touchesBottomEdge;

    const auto normal = calculateVerticalTarget(
        720.0, 470.0, 530.0, 0.70, 30.0, 35.0);
    require(normal.valid, "normal bounding box is valid");
    requireNear(normal.targetCenterY, 504.0, "70 percent target is below image centre");
    requireNear(normal.errorY, -4.0, "vertical error uses the lower target");

    const auto large = calculateVerticalTarget(
        720.0, 200.0, 700.0, 0.70, 30.0, 35.0);
    require(large.valid, "large bounding box is valid");
    requireNear(large.targetCenterY, 405.0, "large box is capped by its bottom safety margin");
    require(
        large.targetCenterY + 35.0 + 250.0 <= 720.0 - 30.0,
        "lower deadband edge preserves the complete bbox bottom margin");

    const auto invalid = calculateVerticalTarget(0.0, 10.0, 20.0, 0.70, 30.0);
    require(!invalid.valid, "zero-height image is rejected");

    requireNear(
        nextPitch(0.60, -100.0, 35.0, 0.04, 0.02, 0.45, 0.90),
        0.56,
        "ball above target raises the head");
    requireNear(
        nextPitch(0.60, 45.0, 35.0, 0.04, 0.02, 0.45, 0.90),
        0.62,
        "near-boundary error uses settle step");
    requireNear(
        nextPitch(0.60, 10.0, 35.0, 0.04, 0.02, 0.45, 0.90),
        0.60,
        "deadband holds pitch");
    requireNear(
        nextPitch(0.46, -100.0, 35.0, 0.04, 0.02, 0.45, 0.90),
        0.45,
        "upward pitch command respects the configured limit");

    require(touchesBottomEdge(720.0, 712.0, 10.0), "bbox at bottom edge is detected");
    require(!touchesBottomEdge(720.0, 700.0, 10.0), "bbox above bottom edge is not detected");

    std::cout << "ball_framing_utils_test: PASS\n";
    return 0;
}
