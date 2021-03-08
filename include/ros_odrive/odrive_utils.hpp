#ifndef ODRIVE_UTILS_HPP_
#define ODRIVE_UTILS_HPP_

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <iostream>
#include <string>
#include <vector>
#include <jsoncpp/json/json.h>

#include "ros/ros.h"
#include "ros_odrive/odrive.hpp"
#include "ros_odrive/odrive_endpoint.hpp"

typedef struct _odrive_object {
    std::string name;
    int id;
    std::string type;
    std::string access;
} odrive_object;

int updateTargetConfig(odrive_endpoint *endpoint, Json::Value odrive_json, std::string config_file);

int getJson(odrive_endpoint *endpoint, Json::Value *json);

int setChannelConfig(odrive_endpoint *endpoint, Json::Value odrive_json, Json::Value config_json,
        bool save_config);
int calibrateAxis0(odrive_endpoint *endpoint, Json::Value odrive_json);
int calibrateAxis1(odrive_endpoint *endpoint, Json::Value odrive_json);

int getObjectByName(Json::Value odrive_json, std::string name, odrive_object *odo);

template<typename TT>
int readOdriveData(odrive_endpoint *endpoint, Json::Value odrive_json,
        std::string object, TT &value);

template<typename T>
int writeOdriveData(odrive_endpoint *endpoint, Json::Value odrive_json,
        std::string object, T &value);

int execOdriveFunc(odrive_endpoint *endpoint, Json::Value odrive_json, std::string object);

int execOdriveGetTemp(odrive_endpoint *endpoint, Json::Value odrive_json,
        std::string object, float &temp);

#endif
