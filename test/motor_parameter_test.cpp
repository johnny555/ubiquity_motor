/**
Copyright (c) 2016, Ubiquity Robotics
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of ubiquity_motor nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#include <gtest/gtest.h>
#include <ubiquity_motor/motor_parameters.hpp>

TEST(MotorParameterTests, getParamOrDefaultTest) {
  ros::NodeHandle nh;
  ASSERT_EQ(5, getParamOrDefault(nh, "getParamOrDefaultTest", 5));
  int foo;
  ASSERT_TRUE(nh.getParam("getParamOrDefaultTest", foo));
  ASSERT_EQ(5, foo);

  ASSERT_EQ(5, getParamOrDefault(nh, "getParamOrDefaultTest", 10));
}

TEST(MotorParameterTests, NodeParamTest) {
  ros::NodeHandle nh;
  NodeParams np(nh);
  ASSERT_DOUBLE_EQ(10.0, np.controller_loop_rate);
  nh.setParam("ubiquity_motor/controller_loop_rate", 50.0);
  np = NodeParams(nh);
  ASSERT_DOUBLE_EQ(50.0, np.controller_loop_rate);
}

TEST(MotorParameterTests, CommsParamsTest) {
  ros::NodeHandle nh;
  CommsParams cp(nh);
  ASSERT_EQ("/dev/ttyS0", cp.serial_port);
  nh.setParam("ubiquity_motor/serial_port", "/dev/foo");
  cp = CommsParams(nh);
  ASSERT_EQ("/dev/foo", cp.serial_port);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "param_test");
  return RUN_ALL_TESTS();
}
