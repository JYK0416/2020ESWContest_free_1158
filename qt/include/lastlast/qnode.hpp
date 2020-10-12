/**
 * @file /include/lastlast/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef lastlast_QNODE_HPP_
#define lastlast_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/


namespace lastlast {

/*****************************************************************************
** Class
*****************************************************************************/


class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	void sendmode(std_msgs::Int32 msg); 
	void sendsensor(std_msgs::Int32MultiArray msg);
	void sendreset(std_msgs::Int32 msg);
	void sendstart(std_msgs::Int32 msg);

	void sendtestsensor(std_msgs::Int32 msg);
	void sendtestonoff(std_msgs::Int32 msg);
	void sendtestmotor(std_msgs::Int32 msg);
	void sendtestposition(std_msgs::Int32 msg);


	QString ss_sensor;
	QString ss_position;


	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

    void setValues(QString tex);
    void setPosition(QString tex);
    void setsensor(QString tex);

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
	ros::Publisher send_sensor;
	ros::Publisher send_mode;
	ros::Publisher send_start;
	ros::Publisher send_reset;

	ros::Publisher send_testsensor;
	ros::Publisher send_testonoff;
	ros::Publisher send_testmotor;
	ros::Publisher send_testposition;
	ros::Subscriber get_testvalue;

	ros::Subscriber get_sensor;
	ros::Subscriber get_position;

	int robot_position_x;
    int robot_position_y;
    int sensor_type;
    int sensor_value;

    int test_value_int;

    void getsensors(const std_msgs::Int32MultiArray::ConstPtr &msg);
	void getposition(const std_msgs::Int32MultiArray::ConstPtr &msg);
	void testvalue(const std_msgs::Int32::ConstPtr &msg);

    QStringListModel logging_model;
};

}  // namespace lastlast

#endif /* lastlast_QNODE_HPP_ */
