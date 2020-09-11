#ifndef ODRIVE_HPP_
#define ODRIVE_HPP_

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <iostream>
#include <string>
#include <vector>
#include <libusb-1.0/libusb.h>
#include "ros_odrive/odrive_msg.h"
#include "ros_odrive/odrive_ctrl.h"
#include <string>
#include <fstream>

#include "ros/ros.h"
#include "ros_odrive/odrive_endpoint.hpp"
#include "ros_odrive/odrive_utils.hpp"
#include "ros_odrive/odrive_enums.hpp"
#include <jsoncpp/json/json.h>

#define ODRIVE_OK    0
#define ODRIVE_ERROR 1

#define MAX_NR_OF_TARGETS 16

using namespace std;

// Listener commands
enum commands {
    CMD_AXIS_RESET,
    CMD_AXIS_IDLE,
    CMD_AXIS_CLOSED_LOOP,
    CMD_AXIS_SET_VELOCITY,
    CMD_AXIS_SET_VELOCITY_DUAL,
    CMD_REBOOT
};

class odrive{
    private:
        void msgCallback(const ros_odrive::odrive_ctrl::ConstPtr& msg);

    public:
        vector<string> target_sn;
        vector<string> target_cfg;
        vector<ros::Publisher> odrive_pub;
        vector<ros::Subscriber> odrive_sub;
        vector<odrive_endpoint *> endpoint;
        vector<Json::Value> json;
};

#endif
