#include "ros_odrive/odrive_utils.hpp"

using namespace std;

/**
 *
 *  update odrive config
 *  Configure parameter on odrive hardware
 *  @param endpoint odrive enumarated endpoint
 *  @param odrive_json target json
 *  @param config_json json including configuration parameters
 *  @return ODRIVE_OK on success
 *
 */
int updateTargetConfig(odrive_endpoint *endpoint, Json::Value odrive_json, string config_file)
{
    ifstream cfg;
    string line, json;
    cfg.open (config_file, ios::in);
    cout << config_file;
    if (cfg.is_open()) {
        while (getline(cfg, line)) {
            json.append(line);
        }
        cfg.close();
        Json::Reader reader;
        Json::Value config_json;
        bool res = reader.parse(json, config_json);
        if (!res) {
            ROS_ERROR("* Error parsing %s json!", config_file.c_str());
            return ODRIVE_ERROR;
        }
        else {
            if (setChannelConfig(endpoint, odrive_json, config_json, false) != ODRIVE_OK) {
                ROS_ERROR("* Error setting configuration!");
                return ODRIVE_ERROR;
            }
        }
        if (calibrateAxis0(endpoint, odrive_json) != ODRIVE_OK) {
            ROS_ERROR("* Error calibrating axis 0!");
            return ODRIVE_ERROR;
        }
        if (calibrateAxis1(endpoint, odrive_json) != ODRIVE_OK) {
            ROS_ERROR("* Error calibrating axis 1!");
            return ODRIVE_ERROR;
        }
        if (execOdriveFunc(endpoint, odrive_json, "save_configuration") != ODRIVE_OK) {
            ROS_ERROR("* Error saving configuration!");
            return ODRIVE_ERROR;
        }
    }
    else {
        ROS_ERROR("* Error opening configuration file!");
    return ODRIVE_ERROR;
    }
}

/**
 *
 *  Set odrive config
 *  Configure parameter on odrive hardware
 *  @param endpoint odrive enumarated endpoint
 *  @param odrive_json target json
 *  @param config_json json including configuration parameters
 *  @return ODRIVE_OK on success
 *
 */
int setChannelConfig(odrive_endpoint *endpoint, Json::Value odrive_json, Json::Value config_json,
    	bool save_config = 0)
{
    int ret = ODRIVE_OK;

    for (int i = 0 ; i < config_json.size() ; i++) {
    string name = config_json[i]["name"].asString();
    string type = config_json[i]["type"].asString();

    ROS_INFO("Setting %s config value", name.c_str());

    if (!type.compare("float")) {
            float val = config_json[i]["value"].asFloat();
            writeOdriveData(endpoint, odrive_json, name, val);
    }
        else if (!type.compare("uint8")) {
            uint8_t val = config_json[i]["value"].asUInt();
            writeOdriveData(endpoint, odrive_json, name, val);
    }
        else if (!type.compare("uint16")) {
            uint16_t val = config_json[i]["value"].asUInt();
            writeOdriveData(endpoint, odrive_json, name, val);
    }
        else if (!type.compare("uint32")) {
            uint32_t val = config_json[i]["value"].asUInt();
            writeOdriveData(endpoint, odrive_json, name, val);
    }
        else if (!type.compare("uint64")) {
            uint64_t val = config_json[i]["value"].asUInt64();
            writeOdriveData(endpoint, odrive_json, name, val);
    }
        else if (!type.compare("int32")) {
            int val = config_json[i]["value"].asInt();
            writeOdriveData(endpoint, odrive_json, name, val);
    }
        else if (!type.compare("int16")) {
            short val = config_json[i]["value"].asInt();
            writeOdriveData(endpoint, odrive_json, name, val);
    }
        else if (!type.compare("bool")) {
            bool val = config_json[i]["value"].asBool();
            writeOdriveData(endpoint, odrive_json, name, val);
    }
        else {
            ROS_ERROR("* Error: invalid type for %s", name.c_str());
            return ODRIVE_ERROR;
        }
    }

    // Save configuration
    if (save_config) {
        ret = execOdriveFunc(endpoint, odrive_json, "save_configuration");
    }

    return ret;
}

/**
 *
 *  Calibrate odrive axis 0
 *  Run calibration for axis 0
 *  @param endpoint odrive enumarated endpoint
 *  @param odrive_json target json
 *  @return ODRIVE_OK on success
 *
 */
int calibrateAxis0(odrive_endpoint *endpoint, Json::Value odrive_json)
{
    float fval;
    uint32_t u32val;
    bool bval;

    u32val = AXIS_STATE_MOTOR_CALIBRATION;
    writeOdriveData(endpoint, odrive_json, string("axis0.requested_state"), u32val);
    ros::Duration(10.0).sleep();
    bval = true;
    writeOdriveData(endpoint, odrive_json, string("axis0.motor.config.pre_calibrated"), bval);
    u32val = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
    writeOdriveData(endpoint, odrive_json, string("axis0.requested_state"), u32val);
    ros::Duration(10.0).sleep();
    bval = true;
    writeOdriveData(endpoint, odrive_json, string("axis0.encoder.config.pre_calibrated"), bval);

    fval = 3.0;
    writeOdriveData(endpoint, odrive_json, string("axis0.config.watchdog_timeout"), fval);

    execOdriveFunc(endpoint, odrive_json, string("save_configuration"));
}

/**
 *
 *  Calibrate odrive axis 1
 *  Run calibration for axis 1
 *  @param endpoint odrive enumarated endpoint
 *  @param odrive_json target json
 *  @return ODRIVE_OK on success
 *
 */
int calibrateAxis1(odrive_endpoint *endpoint, Json::Value odrive_json)
{
    float fval;
    uint8_t u32val;
    bool bval;

    u32val = AXIS_STATE_MOTOR_CALIBRATION;
    writeOdriveData(endpoint, odrive_json, string("axis1.requested_state"), u32val);
    ros::Duration(10.0).sleep();
    bval = true;
    writeOdriveData(endpoint, odrive_json, string("axis1.motor.config.pre_calibrated"), bval);
    u32val = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
    writeOdriveData(endpoint, odrive_json, string("axis1.requested_state"), u32val);
    ros::Duration(10.0).sleep();
    bval = true;
    writeOdriveData(endpoint, odrive_json, string("axis1.encoder.config.pre_calibrated"), bval);

    fval = 3.0;
    writeOdriveData(endpoint, odrive_json, string("axis1.config.watchdog_timeout"), fval);

    execOdriveFunc(endpoint, odrive_json, string("save_configuration"));
}

/**
 *
 *  Read JSON file from target
 *  @param endpoint odrive enumarated endpoint
 *  @param odrive_json pointer to target json object
 *
 */
int getJson(odrive_endpoint *endpoint, Json::Value *odrive_json)
{

    commBuffer rx;
    commBuffer tx;
    int len;
    int address = 0;
    string json;

    do {
        endpoint->endpointRequest(0, rx, len, tx, true, 64, true, address);
        address = address + len;
        json.append((const char *)&rx[0], (size_t)len);
    } while (len > 0);

    Json::Reader reader;
    bool res = reader.parse(json, *odrive_json);
    if (!res) {
        ROS_ERROR("* Error parsing json!");
        return 1;
    }
    return 0;
}

/**
 *
 *  Scan for object name in target JSON
 *  @param odrive_json target json
 *  @param name object name to be found
 *  @param odo odrive object pointer including object parameters
 *  @return ODRIVE_OK on success
 *
 */
int getObjectByName(Json::Value odrive_json, std::string name, odrive_object *odo)
{
    int ret = -1;
    int i, pos;
    string token;
    Json::Value js;
    Json::Value js2 = odrive_json;

    while ((pos = name.find(".")) != std::string::npos) {
    js = js2;
        token = name.substr(0, pos);
        for (i = 0 ; i < js.size() ; i++) {
            if (!token.compare(js[i]["name"].asString())) {
    	if (!string("object").compare(js[i]["type"].asString())) {
                js2 = js[i]["members"];
    	}
    	else {
                    js2 = js[i];
    	}
    	break;
            }
        }
    name.erase(0, pos + 1);
    }

    for (i = 0 ; i < js2.size() ; i++) {
        if (!name.compare(js2[i]["name"].asString())) {
            odo->name = js2[i]["name"].asString();
            odo->id = js2[i]["id"].asInt();
            odo->type = js2[i]["type"].asString();
            odo->access = js2[i]["access"].asString();
        ret = 0;
        break;
        }
    }

    if (ret) {
        ROS_ERROR("* %s not found!", name.c_str());
    }
    return ret;
}

/**
 *
 *  Read single value from target
 *  @param endpoint odrive enumarated endpoint
 *  @param odrive_json target json
 *  @param value return value
 *  @return ODRIVE_OK on success
 *
 */
template<typename TT>
int readOdriveData(odrive_endpoint *endpoint, Json::Value odrive_json,
    	std::string object, TT &value)
{
    int ret;
    odrive_object odo;

    ret = getObjectByName(odrive_json, object, &odo);
    if (ret) {
        ROS_ERROR("* Error getting ID for %s", object.c_str());
    return ret;
    }

    if (odo.access.find("r") == string::npos) {
        ROS_ERROR("* Error: invalid read access for %s", object.c_str());
        return ret;
    }

    if (!odo.type.compare("float")) {
    if (sizeof(value) != sizeof(float)) {
            ROS_ERROR("* Error value for %s is not float", object.c_str());
            return ODRIVE_ERROR;
    }
    }
    else if (!odo.type.compare("uint8")) {
        if (sizeof(value) != sizeof(uint8_t)) {
            ROS_ERROR("* Error value for %s is not uint8_t", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("uint16")) {
        if (sizeof(value) != sizeof(uint16_t)) {
            ROS_ERROR("* Error value for %s is not uint16_t", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("uint32")) {
        if (sizeof(value) != sizeof(uint32_t)) {
            ROS_ERROR("* Error value for %s is not uint32_t", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("uint64")) {
        if (sizeof(value) != sizeof(uint64_t)) {
            ROS_ERROR("* Error value for %s is not uint64_t", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("int32")) {
        if (sizeof(value) != sizeof(int)) {
            ROS_ERROR("* Error value for %s is not int", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("int16")) {
        if (sizeof(value) != sizeof(short)) {
            ROS_ERROR("* Error value for %s is not short", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("bool")) {
        if (sizeof(value) != sizeof(bool)) {
            ROS_ERROR("* Error value for %s is not bool", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else {
        ROS_ERROR("* Error: invalid type for %s", object.c_str());
        return ODRIVE_ERROR;
    }

    ret = endpoint->getData(odo.id, value);

    return ret;
}

/**
 *
 *  Write single value to target
 *  @param endpoint odrive enumarated endpoint
 *  @param odrive_json target json
 *  @param value value to be written
 *  @return ODRIVE_OK on success
 *
 */
template<typename T>
int writeOdriveData(odrive_endpoint *endpoint, Json::Value odrive_json,
                std::string object, T &value)
{
    int ret;
    odrive_object odo;

    ret = getObjectByName(odrive_json, object, &odo);
    if (ret) {
        ROS_ERROR("* Error: getting ID for %s", object.c_str());
        return ODRIVE_ERROR;
    }

    if (odo.access.find("w") == string::npos) {
        ROS_ERROR("* Error: invalid write access for %s", object.c_str());
        return ODRIVE_ERROR;
    }

    if (!odo.type.compare("float")) {
        if (sizeof(value) != sizeof(float)) {
            ROS_ERROR("* Error value for %s is not float", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("uint8")) {
        if (sizeof(value) != sizeof(uint8_t)) {
            ROS_ERROR("* Error value for %s is not uint8_t", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("uint16")) {
        if (sizeof(value) != sizeof(uint16_t)) {
            ROS_ERROR("* Error value for %s is not uint16_t", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("uint32")) {
        if (sizeof(value) != sizeof(uint32_t)) {
            ROS_ERROR("* Error value for %s is not uint32_t", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("uint64")) {
        if (sizeof(value) != sizeof(uint64_t)) {
            ROS_ERROR("* Error value for %s is not uint64_t", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("int32")) {
        if (sizeof(value) != sizeof(int)) {
            ROS_ERROR("* Error value for %s is not int", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("int16")) {
        if (sizeof(value) != sizeof(short)) {
            ROS_ERROR("* Error value for %s is not short", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else if (!odo.type.compare("bool")) {
        if (sizeof(value) != sizeof(bool)) {
            ROS_ERROR("* Error value for %s is not bool", object.c_str());
            return ODRIVE_ERROR;
        }
    }
    else {
        ROS_ERROR("* Error: invalid type for %s", object.c_str());
        return ODRIVE_ERROR;
    }

    ret = endpoint->setData(odo.id, value);

    return ret;
}

/**
 *
 *  Exec target function
 *  @param endpoint odrive enumarated endpoint
 *  @param odrive_json target json
 *  @param object name
 *  @return ODRIVE_OK on success
 *
 */
int execOdriveFunc(odrive_endpoint *endpoint, Json::Value odrive_json,
                std::string object)
{
    int ret;
    odrive_object odo;

    ret = getObjectByName(odrive_json, object, &odo);
    if (ret) {
        ROS_ERROR("* Error getting ID");
        return ret;
    }

    if (odo.type.compare("function")) {
        ROS_ERROR("* Error invalid type");
        return ret;
    }

    ret = endpoint->execFunc(odo.id);
    if (ret != LIBUSB_SUCCESS) {
        ROS_ERROR("* Error executing %s function", object.c_str());
    }
    return ret;
}

template int writeOdriveData(odrive_endpoint *, Json::Value, std::string, uint8_t &);
template int writeOdriveData(odrive_endpoint *, Json::Value, std::string, uint16_t &);
template int writeOdriveData(odrive_endpoint *, Json::Value, std::string, uint32_t &);
template int writeOdriveData(odrive_endpoint *, Json::Value, std::string, uint64_t &);
template int writeOdriveData(odrive_endpoint *, Json::Value, std::string, int &);
template int writeOdriveData(odrive_endpoint *, Json::Value, std::string, short &);
template int writeOdriveData(odrive_endpoint *, Json::Value, std::string, float &);
template int writeOdriveData(odrive_endpoint *, Json::Value, std::string, bool &);

template int readOdriveData(odrive_endpoint *, Json::Value, std::string, uint8_t &);
template int readOdriveData(odrive_endpoint *, Json::Value, std::string, uint16_t &);
template int readOdriveData(odrive_endpoint *, Json::Value, std::string, uint32_t &);
template int readOdriveData(odrive_endpoint *, Json::Value, std::string, uint64_t &);
template int readOdriveData(odrive_endpoint *, Json::Value, std::string, int &);
template int readOdriveData(odrive_endpoint *, Json::Value, std::string, short &);
template int readOdriveData(odrive_endpoint *, Json::Value, std::string, float &);
template int readOdriveData(odrive_endpoint *, Json::Value, std::string, bool &);

