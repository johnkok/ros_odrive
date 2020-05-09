#include "ros_odrive/odrive.hpp"

using namespace std;

Json::Value odrive_json;
bool targetJsonValid = false;
odrive_endpoint *endpoint = NULL;

void msgCallback(const ros_odrive::odrive_ctrl::ConstPtr& msg)
{
    std::string cmd;
    uint8_t u8val;
    uint16_t u16val;
    float fval;

    if (msg->axis == 0) {
        cmd = "axis0";
    }
    else if (msg->axis == 1){
        cmd = "axis1";
    }
    else {
        ROS_ERROR("Invalid axis value in message!");
	return;
    }

    switch (msg->command) {
        case (CMD_AXIS_RESET):
	    // Reset errors
            u16val = u8val = 0;
	    writeOdriveData(endpoint, odrive_json,
                    cmd.append(".motor.error"), u16val);
            writeOdriveData(endpoint, odrive_json,
                    cmd.append(".encoder.error"), u8val);
            writeOdriveData(endpoint, odrive_json,
                    cmd.append(".controller.error"), u8val);
	    writeOdriveData(endpoint, odrive_json,
                    cmd.append(".error"), u16val);
	    break;
	case (CMD_AXIS_IDLE):
	    // Set channel to Idle
            u8val = AXIS_STATE_IDLE;
            writeOdriveData(endpoint, odrive_json,
                    cmd.append(".requested_state"), u8val);
	    break;
	case (CMD_AXIS_CLOSED_LOOP):
            // Enable Closed Loop Control
            u8val = AXIS_STATE_CLOSED_LOOP_CONTROL;
            writeOdriveData(endpoint, odrive_json,
                    cmd.append(".requested_state"), u8val);
	    break;
	case (CMD_AXIS_SET_VELOCITY):
	    // Set velocity
	    fval = msg->fval;
            writeOdriveData(endpoint, odrive_json,
                    cmd.append(".controller.vel_setpoint"), fval);
	    break;
	case (CMD_REBOOT):
	    execOdriveFunc(endpoint, odrive_json, string("reboot"));
	    break;
	default:
	    ROS_ERROR("Invalid command type in message!");
	    return;
    }
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
int publishMessage(ros::Publisher odrive_pub)
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
    endpoint = new odrive_endpoint();

    // Enumarate Odrive target
    if (endpoint->init(stoull(od_sn, 0, 16)))
    {
        ROS_ERROR("Device not found!");
        return 1;
    }

    // Read JSON from target
    if (getJson(endpoint, &odrive_json)) {
        return 1;
    }
    targetJsonValid = true;

    // Process configuration file
    if (nh.searchParam("od_cfg", od_cfg)) {
        nh.getParam("od_cfg", od_cfg);
        ROS_INFO("Using configuration file: %s", od_cfg.c_str());

	updateTargetConfig(endpoint, odrive_json, od_cfg);
    }

    // Example loop - reading values and updating motor velocity
    ROS_INFO("Starting idle loop");
    while (ros::ok()) {
        // Publish status message
	publishMessage(odrive_pub);

	// update watchdog
        execOdriveFunc(endpoint, odrive_json, "axis0.watchdog_feed");
        execOdriveFunc(endpoint, odrive_json, "axis1.watchdog_feed");

	// idle loop
	r.sleep();
        ros::spinOnce();
    }

    endpoint->remove();

    delete endpoint;

    return 0;
}
