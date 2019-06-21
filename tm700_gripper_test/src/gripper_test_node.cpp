/*
 * Copyright 2019 Liang-Chi Hsieh
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include "tm_driver/tm_driver.h"

void getDI(TmDriver* robot, std::vector<bool>& vec) {
  robot->interface->stateRT->getDigitalInputEE(vec);
}

void getDO(TmDriver* robot, std::vector<bool>& vec) {
  robot->interface->stateRT->getDigitalOutputEE(vec);
}

void grip(TmDriver* robot) {
  std::vector<bool> vec = {false, false, false, false};

  getDO(robot, vec);
  if (vec[0] == false) {
    print_info("Gripper is now open. Going to close it...");

    // Set DO1
    robot->setDigitalOutputEE('1', true);
    // Set DO0
    robot->setDigitalOutputEE('0', true);

    // Read input
    getDI(robot, vec);
    while (vec[0] != false && vec[2] != true) {
      print_info("DI0: %d, DI2: %d", vec[0], vec[2]);
      getDI(robot, vec);
    }

    robot->setDigitalOutputEE('1', false);

    getDI(robot, vec);
    while (vec[1] != true && vec[2] != true) {
      print_info("DI1: %d, DI2: %d", vec[1], vec[2]);
      getDI(robot, vec);
    }
    print_info("Gripper is closed.");
  } else {
    print_info("Gripper is not in open status. Release it first!");
  }  
}

void release(TmDriver* robot) {
  std::vector<bool> vec = {false, false, false, false};

  getDO(robot, vec);
  if (vec[0] == true) {
    print_info("Gripper is now closed. Going to open it...");

    // Set DO1
    robot->setDigitalOutputEE('1', true);
    // Set DO0
    robot->setDigitalOutputEE('0', false);

    // Read input
    getDI(robot, vec);
    while (vec[1] != false && vec[2] != true) {
      print_info("DI1: %d, DI2: %d", vec[1], vec[2]);
      getDI(robot, vec);
    }

    robot->setDigitalOutputEE('1', false);

    getDI(robot, vec);
    while (vec[0] != true && vec[2] != true) {
      print_info("DI0: %d, DI2: %d", vec[1], vec[2]);
      getDI(robot, vec);
    }
    print_info("Gripper is released.");
  } else {
    print_info("Gripper is not in close status. Close it first!");
  }  
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gripper test");
  ros::NodeHandle nh;

  std::string host;
  std::condition_variable data_cv;
  std::condition_variable data_cv_rt;

  if (!(ros::param::get("~robot_ip_address", host))) {
    if (argc > 1) {
      host = argv[1];
    } else {
      exit(1);
    }
  }
  ros::Rate loop_rate(10);

  print_info("Connecting to the robot...");
  TmDriver* robot = new TmDriver(data_cv, data_cv_rt, host, 0);
  bool isRobotStarted = robot->interface->start();

  if (!isRobotStarted) {
    print_info("Can't connect to TM robot: %s", host.c_str());
    return 0;
  }
  print_info("TM5_700 gripper test");
  char cstr[512];

  while (ros::ok()) {

    print_info("command:");
    memset(cstr, 0, 512);
    fgets(cstr, 512, stdin);

    if (strncmp(cstr, "grip", 4) == 0) {
      grip(robot);
    } else if (strncmp(cstr, "release", 7) == 0) {
      release(robot);
    } else if (strncmp(cstr, "quit", 4) == 0) {
      return 0;
    } else {
      print_info("unknown command: %s", cstr);
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
