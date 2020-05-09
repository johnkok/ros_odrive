#include "ros_odrive/odrive.hpp"

using namespace std;

Json::Value odrive_json;
bool targetJsonValid = false;

void msgCallback(const ros_odrive::odrive_ctrl::ConstPtr& msg)
{
    int i;

}

/**
 *
 * Publise odrive message to ROS
 * @param endpoint odrive enumarated endpoint
 * @param odrive_json target json
 * @param odrive_pub ROS publisher
 * return ODRIVE_OK in success
 *
 */
int publishMessage(odrive_endpoint *endpoint, Json::Value odrive_json, ros::Publisher odrive_pub)
{
    uint16_t u16val;
    uint8_t u8val;
    float fval;
    ros_odrive::odrive_msg msg;

    // Collect data
    readOdriveData(endpoint, odrive_json, string("vbus_voltage"), fval);
    msg.vbus = fval;
    readOdriveData(endpoint, odrive_json, string("axis0.error"), u16val);
    msg.error0 = u16val;
    readOdriveData(endpoint, odrive_json, string("axis1.error"), u16val);
    msg.error1 = u16val;
    readOdriveData(endpoint, odrive_json, string("axis0.current_state"), u8val);
    msg.state0 = u8val;
    readOdriveData(endpoint, odrive_json, string("axis1.current_state"), u8val);
    msg.state1 = u8val;
    readOdriveData(endpoint, odrive_json,
                    string("axis0.encoder.vel_estimate"), fval);
    msg.vel0 = fval;
    readOdriveData(endpoint, odrive_json,
                    string("axis1.encoder.vel_estimate"), fval);
    msg.vel1 = fval;
    readOdriveData(endpoint, odrive_json,
                    string("axis0.encoder.pos_estimate"), fval);
    msg.pos0 = fval;
    readOdriveData(endpoint, odrive_json,
                    string("axis1.encoder.pos_estimate"), fval);
    msg.pos1 = fval;
    readOdriveData(endpoint, odrive_json,
                    string("axis0.motor.current_meas_phB"), fval);
    msg.curr0B = fval;
    readOdriveData(endpoint, odrive_json,
                    string("axis0.motor.current_meas_phC"), fval);
    msg.curr0C = fval;
    readOdriveData(endpoint, odrive_json,
                    string("axis1.motor.current_meas_phB"), fval);
    msg.curr1B = fval;
    readOdriveData(endpoint, odrive_json,
                    string("axis1.motor.current_meas_phC"), fval);
    msg.curr1C = fval;
    execOdriveGetTemp(endpoint, odrive_json,
                    string("axis0.motor.get_inverter_temp"), fval);
    msg.temp0 = fval;
    execOdriveGetTemp(endpoint, odrive_json,
                    string("axis1.motor.get_inverter_temp"), fval);
    msg.temp1 = fval;

    // Publish message
    odrive_pub.publish(msg);

    return ODRIVE_OK;
}

/**
 *
 * Node main function
 *
 */
int main(int argc, char **argv)		
{
    std::string od_sn;
    std::string od_cfg;

    ROS_INFO("Starting ODrive...");

    // Initialize ROS node
    ros::init(argc, argv, "ros_odrive"); // Initializes Node Name
    ros::NodeHandle nh("~");
    ros::Rate r(1);
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
    ros::Publisher odrive_pub = nh.advertise<ros_odrive::odrive_msg>("odrive_msg_" + od_sn, 100);
    ros::Subscriber odrive_sub = nh.subscribe("odrive_ctrl_" + od_sn, 10, msgCallback);

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

	updateTargetConfig(endpoint, odrive_json, od_cfg);
    }

    // Init 
    float vbus, c0b, c0c, c1b, c1c, fval, temp;
    uint16_t error0, error1, u16val;
    int ival;
    uint8_t u8val;
    bool bval;
    float pos0;

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
    ROS_INFO("Starting idle loop");
    while (ros::ok()) {
	// Update velocity
        fval = 40 + (i++)%100;
        writeOdriveData(endpoint, odrive_json, 
			string("axis0.controller.vel_setpoint"), fval);

        // Publish status message
	publishMessage(endpoint, odrive_json, odrive_pub);

	// update watchdog
        execOdriveFunc(endpoint, odrive_json, "axis0.watchdog_feed");
        execOdriveFunc(endpoint, odrive_json, "axis1.watchdog_feed");

	// idle loop
	r.sleep();
        ros::spinOnce();
    }

//    execOdriveFunc(endpoint, odrive_json, string("reboot"));

    endpoint->remove();

    delete endpoint;

    return 0;
}
