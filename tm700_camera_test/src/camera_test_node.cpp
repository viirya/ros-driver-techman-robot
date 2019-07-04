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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Empty.h>

static const std::string OPENCV_WINDOW = "Image window";

void getImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  ROS_INFO("Got image published to 'image_raw' topic");

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(3);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "camera test");
  ros::NodeHandle nh;
  
  std::string imageTopic;
  // Get image topic ROS parameter
  if (!(ros::param::get("~image_topic", imageTopic))) {
    if (argc > 1) {
      imageTopic = argv[1];
    } else {
      exit(1);
    }
  }

  ROS_INFO("Going to capture image from ROS topic: %s", imageTopic.c_str());

  ros::Rate loop_rate(10);
  ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>(imageTopic.c_str(), 1000, getImageCallback);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
