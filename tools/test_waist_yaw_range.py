#!/usr/bin/env python3
import math
import sys

SAFE_LIMIT_DEG = 55.0
PUBLISHED_LIMIT_DEG = 58.0

def main():
    if len(sys.argv) != 2:
        print("Usage:")
        print("  python3 tools/test_waist_yaw_range.py <angle_degrees>")
        print()
        print("Examples:")
        print("  python3 tools/test_waist_yaw_range.py 10")
        print("  python3 tools/test_waist_yaw_range.py -10")
        sys.exit(1)

    angle_deg = float(sys.argv[1])

    if abs(angle_deg) > SAFE_LIMIT_DEG:
        print(f"Refusing angle: {angle_deg} degrees")
        print(f"Safe test limit is ±{SAFE_LIMIT_DEG} degrees.")
        print(f"Published waist yaw range is around ±{PUBLISHED_LIMIT_DEG} degrees.")
        print("Do not test the full limit directly.")
        sys.exit(1)

    angle_rad = math.radians(angle_deg)

    print("Waist yaw test value:")
    print(f"  degrees: {angle_deg}")
    print(f"  radians: {angle_rad:.4f}")
    print()
    print("This script is only a safe range checker.")
    print("It does NOT move the robot yet.")

if __name__ == "__main__":
    main()
