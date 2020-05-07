#include "ros_odrive/odrive.hpp"
#include "ros_odrive/odrive.h"
#include <string>
#include <fstream>

using namespace std;

Json::Value odrive_json;

int main(int argc, char **argv)		
{
    std::string od_sn;
    std::string od_cfg;

    ROS_INFO("Starting ODrive...");

    // Initialize ROS node
    ros::init(argc, argv, "ros_odrive"); // Initializes Node Name
    ros::NodeHandle nh("~");
    ros::Publisher odrive_pub =
        nh.advertise<ros_odrive::odrive>("ros_odrive_msg", 100);
    ros::Rate r(1);
    ros_odrive::odrive msg;
    nh.param<std::string>("od_sn", od_sn, "0x00000000");
    nh.param<std::string>("od_cfg", od_cfg, "");

    // Get device serial number
    if (nh.getParam("od_sn", od_sn)) {
        ROS_INFO("Node odrive S/N: %s", od_sn.c_str());
    }
    else {
        ROS_ERROR("Failed to get sn parameter %s!", od_sn.c_str());
        return 1;
    }

    // Get odrive endpoint instance
    odrive_endpoint *endpoint = new odrive_endpoint();

    // Enumarate Odrive target
    if (endpoint->init(stoull(od_sn, 0, 16)))
    {
        ROS_ERROR("Device not found!");
        return 1;
    }

    // Read JSON from target
    getJson(endpoint, &odrive_json);

    // Process configuration file
    if (nh.searchParam("od_cfg", od_cfg)) {
        nh.getParam("od_cfg", od_cfg);
        ROS_INFO("Using configuration file: %s", od_cfg.c_str());

        ifstream cfg;
        string line, json;
        cfg.open (od_cfg, ios::in);
        if (cfg.is_open()) {
            while (getline(cfg, line)) {
                json.append(line);
            }
            cfg.close();
            Json::Reader reader;
            Json::Value config_json;
            bool res = reader.parse(json, config_json);
            if (!res) {
                ROS_ERROR("Error parsing %s json!", od_cfg.c_str());
            }
            else {
                setChannelConfig(endpoint, odrive_json, config_json, false);
            }
        }
//      calibrateAxis0(endpoint, odrive_json);
        execOdriveFunc(endpoint, odrive_json, "save_configuration");
    }

    // Init 
    float vbus, c0b, c0c, c1b, c1c, fval, temp;
    uint16_t error0, error1, u16val;
    int ival;
    uint8_t u8val;
    bool bval;

    // Reset watchdog/errors on target
    execOdriveFunc(endpoint, odrive_json, "axis0.watchdog_feed");
    u16val = 0;
    writeOdriveData(endpoint, odrive_json, 
                    string("axis0.error"), u16val);
    execOdriveFunc(endpoint, odrive_json, "axis0.watchdog_feed");

    // Enable LOOP Control
    u8val = AXIS_STATE_CLOSED_LOOP_CONTROL;
    writeOdriveData(endpoint, odrive_json, 
		    string("axis0.requested_state"), u8val);

    int i = 0;
    // Example loop - reading values and updating motor velocity
    while (ros::ok()) {
        readOdriveData(endpoint, odrive_json, string("axis0.motor.config.pole_pairs"), ival);
        cout << "poles0: " << ival << endl;

        readOdriveData(endpoint, odrive_json, string("vbus_voltage"), vbus);
        cout << "VBUS: " << vbus << endl;
        readOdriveData(endpoint, odrive_json, string("axis0.error"), error0);
        cout << "axis0: " << error0 << endl;
        readOdriveData(endpoint, odrive_json, string("axis1.error"), error1);
        cout << "axis1: " << error1 << endl;
        readOdriveData(endpoint, odrive_json, 
			string("axis0.motor.current_meas_phB"), c0b);
        cout << "C0B: " << c0b << endl;
        readOdriveData(endpoint, odrive_json, 
			string("axis0.motor.current_meas_phC"), c0c);
        cout << "C0C: " << c0c << endl;
        readOdriveData(endpoint, 
			odrive_json, 
			string("axis1.motor.current_meas_phB"), c1b);
        cout << "C1B: " << c1b << endl;
        readOdriveData(endpoint, 
			odrive_json, 
			string("axis1.motor.current_meas_phC"), c1c);
        cout << "C1C: " << c1c << endl;

	execOdriveGetTemp(endpoint, odrive_json, 
			string("axis0.motor.get_inverter_temp"), temp);
        cout << "Temp0: " << temp << endl;
        execOdriveGetTemp(endpoint, odrive_json, 
                        string("axis1.motor.get_inverter_temp"), temp);
        cout << "Temp1: " << temp << endl;

	// Update watchdog
        execOdriveFunc(endpoint, odrive_json, "axis0.watchdog_feed");

	// Send message
	msg.vbus = vbus;
	msg.error0 = error0;
	msg.error1 = error1;
	msg.curr0B = c0b;
        msg.curr0C = c0c;
        msg.curr1B = c1b;
        msg.curr1C = c1c;
        odrive_pub.publish(msg);

	// Update velocity
        fval = 40 + (i++)%100;
        writeOdriveData(endpoint, odrive_json, 
			string("axis0.controller.vel_setpoint"), fval);

	r.sleep();
        ros::spinOnce();
    }

//    execOdriveFunc(endpoint, odrive_json, string("reboot"));

    endpoint->remove();

    delete endpoint;

    return 0;
}
