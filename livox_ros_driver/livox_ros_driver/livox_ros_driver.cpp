//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "include/livox_ros_driver.h"
#include "livox_ros_driver/LidarControl.h"

#include <chrono>
#include <vector>

#include <ros/ros.h>
#include "lddc.h"
#include "lds_hub.h"
#include "lds_lidar.h"
#include "lds_lvx.h"
#include "livox_sdk.h"
#include <std_srvs/SetBool.h>

using namespace livox_ros;

const int32_t kSdkVersionMajorLimit = 2;

bool lidarControlCallback(livox_ros_driver::LidarControl::Request &req,
                          livox_ros_driver::LidarControl::Response &res) {
    ROS_INFO("Received LidarControl service call");
    uint32_t interval_ms = 100;
    LdsLidar* lidar = LdsLidar::GetInstance(interval_ms);
    if (lidar == nullptr) {
        ROS_ERROR("LiDAR driver not initialized");
        res.message = "LiDAR driver not initialized";
        return true;
    }

    uint8_t handle = 0;
    std::string broadcast_code = lidar->GetConnectedLidarBroadcastCode(handle);
    if (broadcast_code.empty()) {
        ROS_ERROR("No LiDAR connected");
        res.message = "No LiDAR connected";
        return true;
    }

    if (req.set_state) {
        ROS_INFO("Attempting to set LiDAR state");
        LidarMode mode = req.target_state ? kLidarModeNormal : kLidarModePowerSaving;
        bool success = lidar->SetLidarMode(handle, mode);
        res.current_state = lidar->GetLidarState(handle);
        res.message = success ? "LiDAR state set successfully" : "Failed to set LiDAR state";
        
        // If waking up the LiDAR, reinitialize it
        if (success && mode == kLidarModeNormal) {
            ROS_INFO("Reinitializing LiDAR after wake-up");
            lidar->ReinitializeLidar(handle);
        }
    } else {
        ROS_INFO("Retrieving current LiDAR state");
        res.current_state = lidar->GetLidarState(handle);
        res.message = "Current LiDAR state retrieved";
    }

    ROS_INFO("Response - current_state: %d, message: %s", res.current_state, res.message.c_str());
    return true;
}

int main(int argc, char **argv) {
  ROS_INFO("Livox Ros Driver Version: %s", LIVOX_ROS_DRIVER_VERSION_STRING);

  /** Ros related */
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  ros::init(argc, argv, "livox_lidar_publisher");
  ros::NodeHandle livox_node;

  ros::ServiceServer lidar_control_service = livox_node.advertiseService("livox_lidar_control", lidarControlCallback);
  if (!lidar_control_service) {
      ROS_ERROR("Failed to create Livox LiDAR control service");
      // Consider whether you want to exit here or continue
      // return -1;  // Uncomment this if you want to exit on failure
  } else {
      ROS_INFO("Livox LiDAR control service created successfully");
  }

  /** Check sdk version */
  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  if (_sdkversion.major < kSdkVersionMajorLimit) {
    ROS_INFO("The SDK version[%d.%d.%d] is too low", _sdkversion.major,
             _sdkversion.minor, _sdkversion.patch);
    return 0;
  }

  /** Init default system parameter */
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq = 10.0; /* Hz */
  int output_type = kOutputToRos;
  std::string frame_id = "livox_frame";

  livox_node.getParam("xfer_format", xfer_format);
  livox_node.getParam("multi_topic", multi_topic);
  livox_node.getParam("data_src", data_src);
  livox_node.getParam("publish_freq", publish_freq);
  livox_node.getParam("output_data_type", output_type);
  livox_node.getParam("frame_id", frame_id);
  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 1.0) {
    publish_freq = 1.0;
  } else {
    publish_freq = publish_freq;
  }

  /** Lidar data distribute control and lidar data source set */
  Lddc *lddc = new Lddc(xfer_format, multi_topic, data_src, output_type,
                        publish_freq, frame_id);
  lddc->SetRosNode(&livox_node);

  int ret = 0;
  if (data_src == kSourceRawLidar) {
    ROS_INFO("Data Source is raw lidar.");

    std::string user_config_path;
    livox_node.getParam("user_config_path", user_config_path);
    ROS_INFO("Config file : %s", user_config_path.c_str());

    std::string cmdline_bd_code;
    livox_node.getParam("cmdline_str", cmdline_bd_code);

    std::vector<std::string> bd_code_list;
    ParseCommandlineInputBdCode(cmdline_bd_code.c_str(), bd_code_list);

    LdsLidar *read_lidar = LdsLidar::GetInstance(1000 / publish_freq);
    lddc->RegisterLds(static_cast<Lds *>(read_lidar));
    ret = read_lidar->InitLdsLidar(bd_code_list, user_config_path.c_str());
    if (!ret) {
      ROS_INFO("Init lds lidar success!");     
      

    } else {
      ROS_ERROR("Init lds lidar fail!");
    }
  } else if (data_src == kSourceRawHub) {
    ROS_INFO("Data Source is hub.");

    std::string user_config_path;
    livox_node.getParam("user_config_path", user_config_path);
    ROS_INFO("Config file : %s", user_config_path.c_str());

    std::string cmdline_bd_code;
    livox_node.getParam("cmdline_str", cmdline_bd_code);

    std::vector<std::string> bd_code_list;
    ParseCommandlineInputBdCode(cmdline_bd_code.c_str(), bd_code_list);

    LdsHub *read_hub = LdsHub::GetInstance(1000 / publish_freq);
    lddc->RegisterLds(static_cast<Lds *>(read_hub));
    ret = read_hub->InitLdsHub(bd_code_list, user_config_path.c_str());
    if (!ret) {
      ROS_INFO("Init lds hub success!");
    } else {
      ROS_ERROR("Init lds hub fail!");
    }
  } else {
    ROS_INFO("Data Source is lvx file.");


    std::string cmdline_file_path;
    livox_node.getParam("cmdline_file_path", cmdline_file_path);

    do {
      if (!IsFilePathValid(cmdline_file_path.c_str())) {
        ROS_ERROR("File path invalid : %s !", cmdline_file_path.c_str());
        break;
      }

      std::string rosbag_file_path;
      int path_end_pos = cmdline_file_path.find_last_of('.');
      rosbag_file_path = cmdline_file_path.substr(0, path_end_pos);
      rosbag_file_path += ".bag";

      LdsLvx *read_lvx = LdsLvx::GetInstance(1000 / publish_freq);
      lddc->RegisterLds(static_cast<Lds *>(read_lvx));
      lddc->CreateBagFile(rosbag_file_path);
      int ret = read_lvx->InitLdsLvx(cmdline_file_path.c_str());
      if (!ret) {
        ROS_INFO("Init lds lvx file success!");
      } else {
        ROS_ERROR("Init lds lvx file fail!");
      }
    } while (0);
  }

  ros::Time::init();

  ros::AsyncSpinner spinner(2); // Use 2 threads
  spinner.start();

  ros::Rate loop_rate(100); // 100 Hz, adjust as needed

  while (ros::ok()) {
    lddc->DistributeLidarData();
    loop_rate.sleep();
  }
  spinner.stop();

  return 0;
}
