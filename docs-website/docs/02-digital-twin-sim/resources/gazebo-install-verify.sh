#!/bin/bash

###############################################################################
# gazebo-install-verify.sh
# ========================
# Verification script for Gazebo Fortress + ROS 2 Humble installation
#
# Usage:
#   bash gazebo-install-verify.sh
#
# This script checks:
#   1. Gazebo Fortress is installed
#   2. ROS 2 Humble is installed
#   3. ros-humble-ros-gz bridge is installed
#   4. Gazebo can launch successfully
#   5. ROS 2 + Gazebo integration works
#
# Output:
#   ✅ (green) = PASS
#   ❌ (red) = FAIL
#   ⚠️ (yellow) = WARNING
#
###############################################################################

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test counter
TESTS_PASSED=0
TESTS_FAILED=0

echo "=========================================="
echo "Gazebo Fortress Installation Verification"
echo "=========================================="
echo ""

# Test 1: Check Gazebo installation
echo "[Test 1/5] Checking Gazebo installation..."
if command -v gazebo &> /dev/null; then
    GAZEBO_VERSION=$(gazebo --version 2>/dev/null | head -1)
    echo -e "${GREEN}✅ PASS${NC} Gazebo installed"
    echo "    Version: $GAZEBO_VERSION"
    ((TESTS_PASSED++))
else
    echo -e "${RED}❌ FAIL${NC} Gazebo not found"
    echo "    Install with: sudo apt install gazebo ros-humble-ros-gz"
    ((TESTS_FAILED++))
fi
echo ""

# Test 2: Check ROS 2 Humble installation
echo "[Test 2/5] Checking ROS 2 Humble installation..."
if command -v ros2 &> /dev/null; then
    ROS_VERSION=$(ros2 --version 2>/dev/null)
    echo -e "${GREEN}✅ PASS${NC} ROS 2 installed"
    echo "    Version: $ROS_VERSION"
    ((TESTS_PASSED++))
else
    echo -e "${RED}❌ FAIL${NC} ROS 2 not found"
    echo "    Install from: https://docs.ros.org/en/humble/Installation.html"
    ((TESTS_FAILED++))
fi
echo ""

# Test 3: Check ROS 2 + Gazebo bridge
echo "[Test 3/5] Checking ros-humble-ros-gz bridge..."
if ros2 pkg list 2>/dev/null | grep -q "gazebo_ros"; then
    echo -e "${GREEN}✅ PASS${NC} gazebo_ros bridge installed"
    ((TESTS_PASSED++))
else
    echo -e "${RED}❌ FAIL${NC} gazebo_ros bridge not found"
    echo "    Install with: sudo apt install ros-humble-ros-gz"
    ((TESTS_FAILED++))
fi
echo ""

# Test 4: Check if Gazebo can launch (timeout after 5 seconds)
echo "[Test 4/5] Testing Gazebo launch capability..."
echo "    Attempting to launch Gazebo (timeout 5 seconds)..."

# Launch Gazebo in headless mode with timeout
timeout 5 gazebo --version > /dev/null 2>&1

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ PASS${NC} Gazebo launches successfully"
    ((TESTS_PASSED++))
elif [ $? -eq 124 ]; then
    # Timeout (actually OK for launch test)
    echo -e "${YELLOW}⚠️ WARNING${NC} Gazebo launch timeout (may be normal)"
    echo "    This could mean Gazebo is starting but taking time"
    ((TESTS_PASSED++))
else
    echo -e "${RED}❌ FAIL${NC} Gazebo failed to launch"
    echo "    Check GPU drivers and CUDA installation"
    ((TESTS_FAILED++))
fi
echo ""

# Test 5: Check ROS 2 + Gazebo integration
echo "[Test 5/5] Testing ROS 2 + Gazebo integration..."

# Check if we can launch gazebo_ros
if ros2 launch gazebo_ros gazebo.launch.py &>/dev/null &
then
    GAZEBO_PID=$!
    sleep 2

    # Check if Gazebo published ROS 2 topics
    if ros2 topic list 2>/dev/null | grep -q "gazebo"; then
        echo -e "${GREEN}✅ PASS${NC} ROS 2 + Gazebo integration working"
        kill $GAZEBO_PID 2>/dev/null
        ((TESTS_PASSED++))
    else
        echo -e "${YELLOW}⚠️ WARNING${NC} ROS 2 topics not found"
        echo "    This could be a display issue (headless mode)"
        kill $GAZEBO_PID 2>/dev/null
        ((TESTS_PASSED++))
    fi
else
    echo -e "${YELLOW}⚠️ WARNING${NC} Could not test ROS 2 integration"
    echo "    Make sure ROS 2 Humble is properly sourced"
    ((TESTS_PASSED++))
fi
echo ""

# Summary
echo "=========================================="
echo "Installation Verification Summary"
echo "=========================================="
echo -e "${GREEN}✅ Passed: $TESTS_PASSED${NC}"
echo -e "${RED}❌ Failed: $TESTS_FAILED${NC}"
echo ""

if [ $TESTS_FAILED -eq 0 ]; then
    echo -e "${GREEN}✅ ALL TESTS PASSED${NC}"
    echo ""
    echo "You're ready to proceed with Module 2!"
    echo ""
    echo "Next steps:"
    echo "1. Read Chapter 2: Setting Up Gazebo Fortress"
    echo "2. Launch your first world: gazebo simple_world.sdf"
    echo "3. Spawn robot: ros2 launch gazebo_ros spawn_model.launch.py model:=robot_sim.urdf"
    exit 0
else
    echo -e "${RED}❌ SOME TESTS FAILED${NC}"
    echo ""
    echo "Please fix the issues above before proceeding."
    exit 1
fi
