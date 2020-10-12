/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/lastlast/qnode.hpp"

#include <QString>

//class MainWindow : public QMainWindow;

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace lastlast {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{
		}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"lastlast");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	send_mode = n.advertise<std_msgs::Int32>("/mode",1);
	send_sensor = n.advertise<std_msgs::Int32MultiArray>("/sensor",1);
	send_reset = n.advertise<std_msgs::Int32>("/reset",1);
	send_start = n.advertise<std_msgs::Int32>("/start",1);

	//test mode
	send_testsensor = n.advertise<std_msgs::Int32>("/test_sensor",1);
	send_testposition = n.advertise<std_msgs::Int32>("/test_position",1);
	send_testonoff = n.advertise<std_msgs::Int32>("/test_onoff",1);
	send_testmotor = n.advertise<std_msgs::Int32>("/test_motor",1);
	get_testvalue = n.subscribe("/test_value",1, &QNode::testvalue,this);

	get_sensor = n.subscribe("/sensor_value", 1, &QNode::getsensors,this);
	get_position = n.subscribe("/robot_position",1, &QNode::getposition,this);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"lastlast");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	send_mode = n.advertise<std_msgs::Int32>("/mode",1);
	send_sensor = n.advertise<std_msgs::Int32MultiArray>("/sensor",1);
	send_reset = n.advertise<std_msgs::Int32>("/reset",1);
	send_start = n.advertise<std_msgs::Int32>("/start",1);

	//test mode
	send_testsensor = n.advertise<std_msgs::Int32>("/test_sensor",1);
	send_testposition = n.advertise<std_msgs::Int32>("/test_position",1);
	send_testonoff = n.advertise<std_msgs::Int32>("/test_onoff",1);
	send_testmotor = n.advertise<std_msgs::Int32>("/test_motor",1);
	get_testvalue = n.subscribe("/test_value",1, &QNode::testvalue,this);


	get_sensor = n.subscribe("/sensor_value",1, &QNode::getsensors,this);
	get_position = n.subscribe("/robot_position",1, &QNode::getposition,this);
	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {

		//std_msgs::String msg;
		//std::stringstream ss;
		//ss << "hello world " << count;
		//msg.data = ss.str();
		//chatter_publisher.publish(msg);
		//log(Info,std::string("I sent: ")+msg.data);
		ros::spinOnce();
		loop_rate.sleep();
		//++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::sendmode(std_msgs::Int32 msg)
{
	send_mode.publish(msg);
}

void QNode::sendsensor(std_msgs::Int32MultiArray msg)
{
	send_sensor.publish(msg);
}

void QNode::sendreset(std_msgs::Int32 msg)
{
	send_reset.publish(msg);
}

void QNode::sendstart(std_msgs::Int32 msg)
{
	send_start.publish(msg);
}

//test pub
void QNode::sendtestsensor(std_msgs::Int32 msg)
{
	send_testsensor.publish(msg);
}

void QNode::sendtestonoff(std_msgs::Int32 msg)
{
	send_testonoff.publish(msg);
}

void QNode::sendtestmotor(std_msgs::Int32 msg)
{
	send_testmotor.publish(msg);
}

void QNode::sendtestposition(std_msgs::Int32 msg)
{
	send_testposition.publish(msg);
}

//sub
void QNode::getposition(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
	robot_position_x = msg -> data[0];
	robot_position_y = msg -> data[1];
	
	QString ss_position = "x : " + QString::number(robot_position_x) + "  y : " + QString::number(robot_position_y);
	//std::cout << "Here is -qnode getposition-"<<ss_position.toStdString() << std::endl;
	Q_EMIT setPosition(ss_position);
	//sleep(1);
}

void QNode::getsensors(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
	QString sensor_type_name;
	sensor_type_name = "";

	sensor_type = msg -> data[0];
	sensor_value = msg -> data[1];

	if (sensor_type == 1){
		sensor_type_name = "sonar";
	}

	else if (sensor_type == 2){
		sensor_type_name = "color";
	}

	else if (sensor_type == 3){
		sensor_type_name = "IR";
	}

	else if (sensor_type == 4){
		sensor_type_name = "temperature";
	}

	else if (sensor_type == 5){
		sensor_type_name = "humidity";
	}

	else if (sensor_type == 6){
		sensor_type_name = "person detection";
	}

	else if (sensor_type == 7){
		sensor_type_name = "sound";
	}

	else if (sensor_type == 8){
		sensor_type_name = "illuminance";
	}

	QString ss_sensor = "sensor : " + sensor_type_name + " / value : " + QString::number(sensor_value);
	//std::cout << "Here is -qnode getsensors-"<< ss_sensor.toStdString() << std::endl;
	Q_EMIT setValues(ss_sensor);
	//sleep(1);
}

//test sub
void QNode::testvalue(const std_msgs::Int32::ConstPtr &msg)
{
	test_value_int = msg -> data;
	//test_value_int = msg -> data[0];
        QString ss_sensor = QString::number(test_value_int);
	Q_EMIT setsensor(ss_sensor);
}

}  // namespace lastlast
