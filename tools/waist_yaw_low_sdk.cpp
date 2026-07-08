#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <booster/idl/b1/LowCmd.h>
#include <booster/idl/b1/MotorCmd.h>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/channel/channel_publisher.hpp>

namespace {

constexpr float kSafeLimitDeg = 50.0F;
constexpr float kHardLimitDeg = 55.0F;
constexpr float kDefaultKp = 60.0F;
constexpr float kDefaultKd = 3.0F;
constexpr float kDefaultWeight = 0.35F;
constexpr float kControlDt = 0.02F;
constexpr float kPi = 3.14159265358979323846F;

float degToRad(float degrees) {
  return degrees * kPi / 180.0F;
}

bool parseFloat(const std::string &text, float *value) {
  std::istringstream stream(text);
  stream >> *value;
  return !stream.fail() && stream.eof();
}

void printUsage(const char *program) {
  std::cout
      << "Usage:\n"
      << "  " << program << " <network_interface> [options] <angle_degrees>...\n\n"
      << "Examples:\n"
      << "  " << program << " eth0 10 0 -10 0\n"
      << "  " << program << " eth0 --hold 2 50 0 -50 0\n\n"
      << "Options:\n"
      << "  --hold <seconds>       Hold each target. Default: 1.0\n"
      << "  --kp <value>           Waist position gain. Default: " << kDefaultKp << "\n"
      << "  --kd <value>           Waist damping gain. Default: " << kDefaultKd << "\n"
      << "  --weight <value>       Waist command weight 0..1. Default: " << kDefaultWeight << "\n"
      << "  --allow-near-limit     Allow up to +/-" << kHardLimitDeg << " deg instead of +/-"
      << kSafeLimitDeg << " deg\n";
}

}  // namespace

int main(int argc, char const *argv[]) {
  if (argc < 3) {
    printUsage(argv[0]);
    return 1;
  }

  const std::string network_interface = argv[1];
  float hold_seconds = 1.0F;
  float kp = kDefaultKp;
  float kd = kDefaultKd;
  float weight = kDefaultWeight;
  bool allow_near_limit = false;
  std::vector<float> targets_deg;

  for (int i = 2; i < argc; ++i) {
    const std::string arg = argv[i];
    auto needValue = [&](float *target) -> bool {
      if (i + 1 >= argc || !parseFloat(argv[i + 1], target)) {
        std::cerr << "Missing or invalid value after " << arg << "\n";
        return false;
      }
      ++i;
      return true;
    };

    if (arg == "--hold") {
      if (!needValue(&hold_seconds)) return 1;
    } else if (arg == "--kp") {
      if (!needValue(&kp)) return 1;
    } else if (arg == "--kd") {
      if (!needValue(&kd)) return 1;
    } else if (arg == "--weight") {
      if (!needValue(&weight)) return 1;
    } else if (arg == "--allow-near-limit") {
      allow_near_limit = true;
    } else {
      float degrees = 0.0F;
      if (!parseFloat(arg, &degrees)) {
        std::cerr << "Unknown argument: " << arg << "\n";
        printUsage(argv[0]);
        return 1;
      }
      targets_deg.push_back(degrees);
    }
  }

  if (targets_deg.empty()) {
    std::cerr << "No waist targets provided.\n";
    printUsage(argv[0]);
    return 1;
  }

  const float limit = allow_near_limit ? kHardLimitDeg : kSafeLimitDeg;
  for (float degrees : targets_deg) {
    if (std::fabs(degrees) > limit) {
      std::cerr << "Refusing " << degrees << " deg. Limit is +/-" << limit << " deg.\n";
      return 1;
    }
  }

  if (hold_seconds < kControlDt) {
    std::cerr << "Hold time must be at least " << kControlDt << " seconds.\n";
    return 1;
  }
  if (weight < 0.0F || weight > 1.0F) {
    std::cerr << "Weight must be between 0 and 1.\n";
    return 1;
  }

  std::cout
      << "Low-level waist yaw test\n"
      << "  topic: " << booster::robot::b1::kTopicJointCtrl << "\n"
      << "  network_interface: " << network_interface << "\n"
      << "  waist_joint_index: " << static_cast<int>(booster::robot::b1::JointIndex::kWaist) << "\n"
      << "  kp: " << kp << "  kd: " << kd << "  weight: " << weight << "\n\n"
      << "Safety check:\n"
      << "  Robot must be stable and clear around the waist.\n"
      << "  Follow the SDK low-level requirement: Prepare mode first, then Custom when ready.\n"
      << "  Be ready to stop immediately.\n\n"
      << "Targets:\n";

  for (float degrees : targets_deg) {
    std::cout << "  " << degrees << " deg = " << degToRad(degrees) << " rad\n";
  }

  std::cout << "\nPress ENTER to start publishing waist commands, or Ctrl-C to cancel.\n";
  std::cin.get();

  booster::robot::ChannelFactory::Instance()->Init(0, network_interface);

  booster::robot::ChannelPublisherPtr<booster_interface::msg::LowCmd> publisher;
  publisher.reset(new booster::robot::ChannelPublisher<booster_interface::msg::LowCmd>(
      booster::robot::b1::kTopicJointCtrl));
  publisher->InitChannel();

  booster_interface::msg::LowCmd msg;
  msg.cmd_type(booster_interface::msg::CmdType::PARALLEL);

  for (size_t i = 0; i < booster::robot::b1::kJointCnt; ++i) {
    booster_interface::msg::MotorCmd motor_cmd;
    motor_cmd.mode(0);
    motor_cmd.q(0.0F);
    motor_cmd.dq(0.0F);
    motor_cmd.tau(0.0F);
    motor_cmd.kp(0.0F);
    motor_cmd.kd(0.0F);
    motor_cmd.weight(0.0F);
    msg.motor_cmd().push_back(motor_cmd);
  }

  const auto waist_index = static_cast<size_t>(booster::robot::b1::JointIndex::kWaist);
  const auto sleep_time = std::chrono::milliseconds(static_cast<int>(kControlDt * 1000.0F));
  const int hold_steps = static_cast<int>(hold_seconds / kControlDt);

  for (float degrees : targets_deg) {
    const float radians = degToRad(degrees);
    auto &waist = msg.motor_cmd().at(waist_index);
    waist.mode(0);
    waist.q(radians);
    waist.dq(0.0F);
    waist.tau(0.0F);
    waist.kp(kp);
    waist.kd(kd);
    waist.weight(weight);

    std::cout << "Publishing waist target " << degrees << " deg (" << radians << " rad)\n";
    for (int step = 0; step < hold_steps; ++step) {
      publisher->Write(msg);
      std::this_thread::sleep_for(sleep_time);
    }
  }

  auto &waist = msg.motor_cmd().at(waist_index);
  waist.weight(0.0F);
  waist.kp(0.0F);
  waist.kd(0.0F);
  for (int step = 0; step < 10; ++step) {
    publisher->Write(msg);
    std::this_thread::sleep_for(sleep_time);
  }

  std::cout << "Done. Waist command weight released.\n";
  return 0;
}
