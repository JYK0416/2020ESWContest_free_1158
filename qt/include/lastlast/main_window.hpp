/**
 * @file /include/lastlast/main_window.hpp
 *
 * @brief Qt based gui for lastlast.
 *
 * @date November 2010
 **/
#ifndef lastlast_MAIN_WINDOW_H
#define lastlast_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace lastlast {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

	//final files
	QString mode_str;
    int mode_int;

    QString sensor_str;
    int sensor_int;

    QString sensor_position_str;
    int sensor_position_int;

    QString sensor_value_str;
    int sensor_value_int;
    int sensor_value_int_r;

    QString motor_direction_str;
    int motor_direction_int;

    int motor_value_int;

    QString txt;

    std_msgs::Int32MultiArray msg1;

    std_msgs::Int32 msg_testsensor;
    std_msgs::Int32 msg_testonoff;


    

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

    //final files
    void sensor_reset_free();
    void sensor_reset_maze();

    void radio_freemode_unchecked();
    void radio_mazemode_unchecked();

    void radio_freemode_enable();
    void radio_mazemode_enable();

    void on_freemode_clicked();
    void on_mazemode_clicked();

    void on_generally_free_clicked();
    void on_generally_maze_clicked();

    void on_Temperature_clicked();
    void on_Persondetection_clicked();
    void on_Sound_clicked();
    void on_Illuminance_clicked();
    void on_IR_free_clicked();
    void on_Sonar_free_clicked();

    void on_Sonar_maze_clicked();

    void on_sensor_position_free_currentIndexChanged(int index);
    void on_sensor_position_maze_currentIndexChanged(int index);

    void on_sensor_value_free_valueChanged(int arg1);
    void on_sensor_value_maze_valueChanged(int arg1);

    void on_motor_position_free_currentIndexChanged(int index);
    void on_motor_position_maze_currentIndexChanged(int index);

    void on_motor_value_free_valueChanged(int arg1);
    void on_motor_value_maze_valueChanged(int arg1);

    void on_smaller_free_clicked();
    void on_bigger_free_clicked();
    void on_smaller_maze_clicked();
    void on_bigger_maze_clicked();

    void on_irblack_free_clicked();
    void on_irwhite_free_clicked();

    void on_exist_free_clicked();
    void on_none_free_clicked();

    void on_Add_free_clicked();
    void on_Add_maze_clicked();

    void on_reset_free_clicked();
    void on_reset_maze_clicked();

    void on_start_free_clicked();
    void on_start_maze_clicked();

    void showValues(QString tex);
    void showPosition(QString tex);

    //test mode
    void on_Sonar_test_clicked();
    void on_IR_test_clicked();
    void on_Temperature_test_clicked();
    void on_Persondetection_test_clicked();
    void on_Sound_test_clicked();
    void on_Illuminance_test_clicked();
    void on_on_test_clicked();
    void on_off_test_clicked();

    void on_forward_test_p_clicked();
    void on_backward_test_p_clicked();
    void on_left_test_p_clicked();
    void on_right_test_p_clicked();
    void on_under_test_p_clicked();

    void on_forward_test_clicked();
    void on_backward_test_clicked();
    void on_left_test_clicked();
    void on_right_test_clicked();
    void on_stop_test_clicked();

    void showsensor(QString tex);

private:
	Ui::MainWindowDesign ui;
	//Ui::MainWindowDesign *ui;
	QNode qnode;
};

}  // namespace lastlast

#endif // lastlast_MAIN_WINDOW_H
