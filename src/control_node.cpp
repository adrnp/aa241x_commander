/**
 * skeleton / example code for a node to do command and control of the pixhawk
 */

// includes
#include <math.h>
#include <ros/ros.h>

// topic data
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

#include <aa241x_mission/SensorMeasurement.h>
#include <aa241x_mission/MissionState.h>

enum class MissionElement {
	None,
	Takeoff,
	Searching,
	Landing
};

/**
 * class to contain the functionality of the controller node.
 */
class ControlNode {

public:

	/**
	 * example constructor.
	 * @param flight_alt the desired altitude for the takeoff point.
	 */
	ControlNode(float flight_alt, float flight_speed);

	/**
	 * the main loop to be run for this node (called by the `main` function)
	 * @return exit code
	 */
	int run();

	/**
	 * allow setting the mission element manually to force different modes of operation
	 * @param element the mission element to change to
	 */
	inline void setMissionElement(MissionElement element) { _mission_element = element; };


private:


	// node handler
	ros::NodeHandle _nh;

	// TODO: add any settings, etc, here
	float _flight_alt = 20.0f;		// desired flight altitude [m] AGL (above takeoff)
	float _flight_speed = 4.0f;

	// data
	mavros_msgs::State _current_state;
	geometry_msgs::PoseStamped _current_local_pos;
	geometry_msgs::PoseStamped _landing_range;

	// waypoint handling (example)
	MissionElement _mission_element = MissionElement::None;
	int _wp_index = -1;
	int _n_waypoints = 1;
	float _target_alt = 0.0f;

	// offset information
	float _e_offset = 0.0f;
	float _n_offset = 0.0f;
	float _u_offset = 0.0f;

	// subscribers
	ros::Subscriber _state_sub;			// the current state of the pixhawk
	ros::Subscriber _local_pos_sub;		// local position information
	ros::Subscriber _sensor_meas_sub;	// mission sensor measurement
	ros::Subscriber _mission_state_sub;	// mission state
	ros::Subscriber _landing_range_sub;	// measurement from the imaging node
	// TODO: add subscribers here

	// publishers
	ros::Publisher _cmd_pub;
	// TODO: recommend adding publishers for data you might want to log

	// callbacks

	/**
	 * callback for the current state of the pixhawk.
	 * @param msg mavros state message
	 */
	void stateCallback(const mavros_msgs::State::ConstPtr& msg);

	/**
	 * callback for the local position and orientation computed by the pixhawk.
	 * @param msg pose stamped message type
	 */
	void localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

	/**
	 * callback for the sensor measurement for the AA241x mission
	 * NOTE: you may end up wanting to move this to a separate mission handling
	 * node
	 * @param msg the AA241x sensor measurement
	 */
	void sensorMeasCallback(const aa241x_mission::SensorMeasurement::ConstPtr& msg);

	/**
	 * callback for the mission state for the AA241x mission
	 * this includes the offset information for the lake lag coordinate frame
	 * @param msg mission state
	 */
	void missionStateCallback(const aa241x_mission::MissionState::ConstPtr& msg);

	/**
	 * callback for the relative vector to the tag for auto landing.
	 * @param msg the landing tag's relative position
	 */
	void landingPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

	// TODO: add callbacks here

	// helper functions

	/**
	 * wait for the connection to the Pixhawk to be established.
	 */
	void waitForFCUConnection();


};


ControlNode::ControlNode(float flight_alt, float flight_speed) :
_flight_alt(flight_alt),
_flight_speed(flight_speed)
{

	// subscribe to the desired topics
	_state_sub = _nh.subscribe<mavros_msgs::State>("mavros/state", 1, &ControlNode::stateCallback, this);
	_local_pos_sub = _nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &ControlNode::localPosCallback, this);
	_sensor_meas_sub =_nh.subscribe<aa241x_mission::SensorMeasurement>("measurement", 10, &ControlNode::sensorMeasCallback, this);
	_mission_state_sub = _nh.subscribe<aa241x_mission::MissionState>("mission_state", 10, &ControlNode::missionStateCallback, this);
	_landing_range_sub = _nh.subscribe<geometry_msgs::PoseStamped>("landing_pose", 10, &ControlNode::landingPoseCallback, this);

	// advertise the published detailed

	// publish a PositionTarget to the `/mavros/setpoint_raw/local` topic which
	// mavros subscribes to in order to send commands to the pixhawk
	_cmd_pub = _nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);

}

void ControlNode::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
	// save the state locally to be used in the main loop
	_current_state = *msg;
}

void ControlNode::localPosCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	// save the current local position locally to be used in the main loop
	// TODO: account for offset to convert from PX4 coordinate to lake lag frame
	_current_local_pos = *msg;

	// update adjust the local position with the offset information to convert
	// to lake lag coordinate frame
	_current_local_pos.pose.position.x += _e_offset;
	_current_local_pos.pose.position.y += _n_offset;
	_current_local_pos.pose.position.z += _u_offset;

	// check to see if have completed the waypoint
	// NOTE: for this case we only have a single waypoint
	switch (_mission_element) {
		case MissionElement::None:
			// NOTE: nothing to do here, waiting on the state to be connected
			break;
		case MissionElement::Takeoff:
			// check condition on being "close enough" to the waypoint
			if (abs(_current_local_pos.pose.position.z - _target_alt - _u_offset) < 0.5) {
				// change to being in the searching state
				_mission_element = MissionElement::Searching;
			}

			break;
		case MissionElement::Searching:
			break;
		case MissionElement::Landing:
			break;
	}
}

void ControlNode::sensorMeasCallback(const aa241x_mission::SensorMeasurement::ConstPtr& msg) {
	// TODO: use the information from the measurement as desired

	// NOTE: this callback is for an example of how to setup a callback, you may
	// want to move this information to a mission handling node
}

void ControlNode::missionStateCallback(const aa241x_mission::MissionState::ConstPtr& msg) {
	// save the offset information
	_e_offset = msg->e_offset;
	_n_offset = msg->n_offset;
	_u_offset = msg->u_offset;
}

void ControlNode::landingPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	// save the landing range locally -> will just be doing control from this range itself
	_landing_range = *msg;
}

void ControlNode::waitForFCUConnection() {
	// wait for FCU connection by just spinning the callback until connected
	ros::Rate rate(5.0);
	while (ros::ok() && _current_state.connected) {
		ros::spinOnce();
		rate.sleep();
	}
}


int ControlNode::run() {

	// wait for the controller connection
	waitForFCUConnection();
	ROS_INFO("connected to the FCU");

	// TODO: move these to be static defines maybe (?)
	uint16_t position_control_mask = (mavros_msgs::PositionTarget::IGNORE_VX |
		mavros_msgs::PositionTarget::IGNORE_VY |
		mavros_msgs::PositionTarget::IGNORE_VZ |
		mavros_msgs::PositionTarget::IGNORE_AFX |
		mavros_msgs::PositionTarget::IGNORE_AFY |
		mavros_msgs::PositionTarget::IGNORE_AFZ |
		mavros_msgs::PositionTarget::IGNORE_YAW_RATE);

	uint16_t velocity_control_mask = (mavros_msgs::PositionTarget::IGNORE_PX |
		mavros_msgs::PositionTarget::IGNORE_PY |
		mavros_msgs::PositionTarget::IGNORE_PZ |
		mavros_msgs::PositionTarget::IGNORE_AFX |
		mavros_msgs::PositionTarget::IGNORE_AFY |
		mavros_msgs::PositionTarget::IGNORE_AFZ |
		mavros_msgs::PositionTarget::IGNORE_YAW_RATE);

	uint16_t custom_mask = 2499;

	// set up the general command parameters
	// NOTE: these will be true for all commands send
	mavros_msgs::PositionTarget cmd;
	cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;	// use the local frame

	// configure the type mask to command only position information
	// NOTE: type mask sets the fields to IGNORE
	// TODO: need to add a link to the mask to explain the value
	cmd.type_mask = velocity_control_mask;


	// cmd.type_mask = 2499;  // mask for Vx Vy and Pz control

	// the yaw information
	// NOTE: just keeping the heading north
	cmd.yaw = 1.5707;	// north in [rad] in an ENU frame

	// the position information for the command
	// NOTE: this is defined in ENU
	geometry_msgs::Point pos;
	pos.x = 0;	// E
	pos.y = 0;	// N
	pos.z = 0;	// U

	// the velocity information for the command
	// NOTE: this is defined in ENU
	geometry_msgs::Vector3 vel;
	vel.x = 0;	// E
	vel.y = 0;	// N
	vel.z = 0;	// U

	// set the loop rate in [Hz]
	// NOTE: must be faster than 2Hz
	ros::Rate rate(10.0);

	// main loop
	while (ros::ok()) {

		// if not in offboard mode, just keep waiting until we are and if not
		// enabled, then keep waiting
		//
		// NOTE: need to be streaming setpoints in order for offboard to be
		// allowed, hence the publishing of an empty command
		if (_current_state.mode != "OFFBOARD") {

			// send a position for 0 velocity in any direction
			cmd.type_mask = velocity_control_mask;
			vel.x = 0;
			vel.y = 0;
			vel.z = 0;

			// timestamp the message and send it
			cmd.header.stamp = ros::Time::now();
			cmd.position = pos;
			cmd.velocity = vel;
			_cmd_pub.publish(cmd);

			// run the ros components
			ros::spinOnce();
			rate.sleep();
			continue;
		}

		// TODO: if drone is not armed at this point, need to send a command to
		// arm it
		//
		// NOTE: this can be done from either the callback or this main
		// function, so need to decide where I want to put it

		// at this point the pixhawk is in offboard control, so we can now fly
		// the drone as desired


		// handle the control based on the current mission element
		switch (_mission_element) {

			case MissionElement::None:
			{
				// just transition straight to taking off
				_mission_element = MissionElement::Searching;
				_target_alt = _flight_alt;
				break;
			}
			case MissionElement::Takeoff:
			{
				// sending a position command
				cmd.type_mask = velocity_control_mask;

				// TODO: would actually need to do a controller on velocity here
				vel.x = 0;
				vel.y = 0;
				vel.z = 0;	// TODO: make this a gain based on altitude error
				break;
			}
			case MissionElement::Searching:
			{
				// want to do lateral velocity control and let the pixhawk take care of holding altitude
				cmd.type_mask = custom_mask;

				// compute vn and ve as an inward spiral
				float ang = atan2(-_current_local_pos.pose.position.x, -_current_local_pos.pose.position.y);

				// fly at 8m/s
				float ve = _flight_speed * sin(ang - 60 * M_PI/180.0f);
				float vn = _flight_speed * cos(ang - 60 * M_PI/180.0f);

				vel.x = ve;
				vel.y = vn;
				pos.z = _target_alt;

				break;
			}
			case MissionElement::Landing:
			{
				// velocity control in all directions
				cmd.type_mask = velocity_control_mask;

				// TODO: compute some gains for this.... might do it low for now

				// NOTE: I'm not account for heading!!!! I'm assuming that the vehicle is being flown pointing north
				// and that the camera in mounted in an orientation such that the camera x axis is out the right and y is out the back

				// range I believe is in camera frame, so for a downward facing camera
				// x -> out the right
				// y -> out the back (down in the image frame itself)
				// z -> down (into the image in the image frame itself)

				float ve = 0.5f * _landing_range.pose.position.x;  // NOTE: there are also no safeguards if the target isn't seen...
				float vn = -0.5f * _landing_range.pose.position.y;
				float vz = -0.1f * _landing_range.pose.position.z;		// NOTE: really should get over the tag before landing

				// set the velocities
				vel.x = ve;
				vel.y = vn;
				vel.z = vz;

				break;
			}
			default:

				// don't move -> 0 velocity
				cmd.type_mask = velocity_control_mask;
				vel.x = 0.0f;
				vel.y = 0.0f;
				vel.z = 0.0f;

				break;
		}


		// publish the command
		cmd.header.stamp = ros::Time::now();
		cmd.position = pos;
		cmd.velocity = vel;
		_cmd_pub.publish(cmd);

		// remember need to always call spin once for the callbacks to trigger
		ros::spinOnce();
		rate.sleep();
	}

	// return  exit code
	return EXIT_SUCCESS;
}


int main(int argc, char **argv) {

	// initialize th enode
	ros::init(argc, argv, "control_node");

	// get parameters from the launch file which define some mission
	// settings
	ros::NodeHandle private_nh("~");
	bool start_in_landing;
	float flight_alt, flight_speed;
	private_nh.param("start_in_landing", start_in_landing, false);
	private_nh.param("flight_altitude", flight_alt, 20.0f);
	private_nh.param("flight_speed", flight_speed, 4.0f);


	// create the node
	ControlNode node(flight_alt, flight_speed);

	if (start_in_landing) {
		node.setMissionElement(MissionElement::Landing);
	}

	// run the node
	return node.run();
}