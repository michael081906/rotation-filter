#include <ros/ros.h>
#include <gtest/gtest.h>
#include "rotation_filter.hpp"

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "rotation_filter_main_test");
  return RUN_ALL_TESTS();
}
