#pragma once

#include <memory>

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

#include "booster_vision/base/intrin.h"
#include "booster_vision/base/pose.h"
#include "booster_vision/model//detector.h"

namespace booster_vision {

class PoseEstimator {
public:
    using Ptr = std::shared_ptr<PoseEstimator>;
    PoseEstimator(const Intrinsics &intr) :
        intr_(intr) {
    }
    ~PoseEstimator() = default;

    virtual void Init(const YAML::Node &node){};

    virtual Pose EstimateByColor(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &rgb);
    virtual Pose EstimateByDepth(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &rgb, const cv::Mat &depth);

    bool use_depth_;
protected:
    Intrinsics intr_;
};

class BallPoseEstimator : public PoseEstimator {
public:
    BallPoseEstimator(const Intrinsics &intr) :
        PoseEstimator(intr) {
    }
    ~BallPoseEstimator() = default;

    void Init(const YAML::Node &node) override;
    Pose EstimateByColor(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &rgb) override;
    Pose EstimateByDepth(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &rgb, const cv::Mat &depth) override;

private:
    bool debug_depth_gate_ = false;
    int depth_sample_step_ = 2;
    int min_ground_points_ = 12;
    int min_depth_points_ = 8;
    int min_points_above_ground_ = 3;
    float min_above_ground_ratio_ = 0.05f;
    float min_height_above_ground_ = 0.04f;
    float max_height_above_ground_ = 0.35f;
    float ground_ring_scale_ = 1.5f;
};

class HumanLikePoseEstimator : public PoseEstimator {
public:
    HumanLikePoseEstimator(const Intrinsics &intr) :
        PoseEstimator(intr) {
    }
    ~HumanLikePoseEstimator() = default;

    void Init(const YAML::Node &node) override;
    Pose EstimateByColor(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &rgb) override;
    Pose EstimateByDepth(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &rgb, const cv::Mat &depth) override;

private:
    float downsample_leaf_size_;
    float statistic_outlier_multiplier_;
    float fitting_distance_threshold_;
};

class FieldMarkerPoseEstimator : public PoseEstimator {
public:
    FieldMarkerPoseEstimator(const Intrinsics &intr) :
        PoseEstimator(intr) {
    }
    ~FieldMarkerPoseEstimator() = default;

    void Init(const YAML::Node &node) override;
    Pose EstimateByColor(const Pose &p_eye2base, const DetectionRes &detection, const cv::Mat &rgb) override;

private:
    bool refine_;
};

cv::Point3f CalculatePositionByIntersection(const Pose &p_eye2base, const cv::Point2f target_uv, const Intrinsics &intr);

struct FieldLineSegment {
    std::vector<cv::Point> contour_2d_points;
    std::vector<cv::Point3f> contour_3d_points;
    std::vector<cv::Point3f> line_model;
    std::vector<cv::Point2f> end_points_2d;
    std::vector<cv::Point3f> end_points_3d;
    unsigned int inlier_count;
    float accu_distance;
    double area;
};

std::vector<FieldLineSegment> FitFieldLineSegments(const Pose &p_eye2base,
                                                   const Intrinsics &intr,
                                                   const std::vector<std::vector<cv::Point>> &contours,
                                                   const int &line_segment_area_threshold);
cv::Mat DrawFieldLineSegments(cv::Mat &img, const std::vector<FieldLineSegment> &segs);

} // namespace booster_vision
