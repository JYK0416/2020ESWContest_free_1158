/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/lastlast/main_window.hpp"
#include "../include/lastlast/qnode.hpp"

#include <std_msgs/String.h>
//#include <std_msgs/Int32MultiArrary>

/*****************************************************************************
** Namespaces
*****************************************************************************/



namespace lastlast {

using namespace Qt;


/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
    
    QString workspace_str;
    workspace_str = "mode / sensor(s.) / s.position / s.condition / s.guide / motor / m.value ";
    ui.workspace_free -> addItem(workspace_str);
    ui.workspace_maze -> addItem(workspace_str);

    QObject::connect(&qnode, SIGNAL(setValues(QString)), this, SLOT(showValues(QString)));
    QObject::connect(&qnode, SIGNAL(setPosition(QString)), this, SLOT(showPosition(QString)));
    QObject::connect(&qnode, SIGNAL(setsensor(QString)), this, SLOT(showsensor(QString)));
    
    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
/*
    ui.workspace_free -> setDragDropMode(QAbstractItemView::InternalMove);
    ui.workspace_driving -> setDragDropMode(QAbstractItemView::InternalMove);
    ui.workspace_maze -> setDragDropMode(QAbstractItemView::InternalMove);
*/

    }
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "lastlast");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "lastlast");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

//mainwindow.cpp
void MainWindow::sensor_reset_free()
{
    ui.Temperature -> setEnabled(true);
    ui.Sound -> setEnabled(true);
    ui.Persondetection -> setEnabled(true);
    ui.Illuminance -> setEnabled(true);
    ui.IR_free -> setEnabled(true);
    ui.Sonar_free -> setEnabled(true);

    sensor_str = "";
    sensor_value_int = 0;
    sensor_value_str = "";
}

void MainWindow::sensor_reset_maze()
{
    ui.Sonar_maze -> setEnabled(true);

    sensor_str = "";
    sensor_value_int = 0;
    sensor_value_str = "";
}


//radio button reset
void MainWindow::radio_freemode_unchecked()
{
    ui.smaller_free -> setChecked(false);
    ui.bigger_free -> setChecked(false);

    ui.exist_free -> setChecked(false);
    ui.none_free -> setChecked(false);

    ui.irblack_free -> setChecked(false);
    ui.irwhite_free -> setChecked(false);
}

void MainWindow::radio_mazemode_unchecked()
{
    ui.smaller_maze -> setChecked(false);
    ui.bigger_maze -> setChecked(false);
}

//enable
void MainWindow::radio_freemode_enable()
{
    ui.smaller_free -> setEnabled(false);
    ui.bigger_free -> setEnabled(false);

    ui.exist_free -> setEnabled(false);
    ui.none_free -> setEnabled(false);

    ui.irblack_free -> setEnabled(false);
    ui.irwhite_free -> setEnabled(false);
}

void MainWindow::radio_mazemode_enable()
{
    ui.smaller_maze -> setEnabled(false);
    ui.bigger_maze -> setEnabled(false);
}


//Mode Change
void MainWindow::on_freemode_clicked()
{
    sensor_reset_free();
    radio_freemode_unchecked();
    mode_str = ui.freemode -> text();

    ui.mazemode -> setChecked(false);

    std_msgs::Int32 msg;
    msg.data = {0};
    qnode.sendmode(msg);
}


void MainWindow::on_mazemode_clicked()
{
    sensor_reset_maze();
    radio_mazemode_unchecked();
    mode_str = ui.mazemode -> text();

    ui.freemode -> setChecked(false);

    std_msgs::Int32 msg;
    msg.data = {1};
    qnode.sendmode(msg);
}

//generally mode
void MainWindow::on_generally_free_clicked()
{
     sensor_reset_free();
     radio_freemode_enable();
     radio_freemode_unchecked();
     ui.sensor_value_free -> setEnabled(true);
     ui.sensor_position_free -> setEnabled(true);

     ui.Temperature -> setChecked(0);
     ui.Sound -> setChecked(0);
     ui.Persondetection -> setChecked(0);
     ui.Illuminance -> setChecked(0);
     ui.IR_free -> setChecked(0);
     ui.Sonar_free -> setChecked(0);

     sensor_str = ui.generally_free -> text();

     sensor_int = 0;
     sensor_value_int = 0;

}


void MainWindow::on_generally_maze_clicked()
{
        sensor_reset_maze();
        radio_mazemode_enable();
        radio_mazemode_unchecked();
        ui.sensor_value_maze -> setEnabled(true);
        ui.sensor_position_maze -> setEnabled(true);

        ui.Sonar_maze -> setChecked(0);

        sensor_str = ui.generally_maze -> text();

        sensor_int = 0;
        sensor_value_int = 0;
       
}

//Free mode sensor
void MainWindow::on_Temperature_clicked()
{
    ui.generally_free -> setChecked(false);

    radio_freemode_enable();
    radio_freemode_unchecked();
    ui.smaller_free -> setEnabled(true);
    ui.bigger_free -> setEnabled(true);
    ui.sensor_value_free -> setEnabled(true);

    sensor_str = ui.Temperature -> text();

    sensor_int = 1;

}

void MainWindow::on_Persondetection_clicked()
{
    ui.generally_free -> setChecked(false);

    radio_freemode_enable();
    radio_freemode_unchecked();
    ui.exist_free -> setEnabled(true);
    ui.none_free -> setEnabled(true);
    ui.sensor_value_free ->setEnabled(false);

    sensor_str = ui.Persondetection -> text();

    sensor_int = 3;
}

void MainWindow::on_Sound_clicked()
{
    ui.generally_free -> setChecked(false);

    radio_freemode_enable();
    radio_freemode_unchecked();
    ui.exist_free -> setEnabled(true);
    ui.none_free -> setEnabled(true);
    ui.sensor_value_free ->setEnabled(false);

    sensor_str = ui.Sound -> text();

    sensor_int = 4;
}

void MainWindow::on_Illuminance_clicked()
{
    ui.generally_free -> setChecked(false);

    radio_freemode_enable();
    radio_freemode_unchecked();
    ui.smaller_free -> setEnabled(true);
    ui.bigger_free -> setEnabled(true);
    ui.sensor_value_free -> setEnabled(true);

    sensor_str = ui.Illuminance -> text();

    sensor_int = 5;
}

void MainWindow::on_Sonar_free_clicked()
{
    ui.generally_free -> setChecked(false);

    radio_freemode_enable();
    radio_freemode_unchecked();
    ui.smaller_free -> setEnabled(true);
    ui.bigger_free -> setEnabled(true);
    ui.sensor_value_free -> setEnabled(true);

    sensor_str = ui.Sonar_free -> text();

    sensor_int = 6;
}

void MainWindow::on_IR_free_clicked()
{
    ui.generally_free -> setChecked(false);

    radio_freemode_enable();
    radio_freemode_unchecked();
    ui.irblack_free -> setEnabled(true);
    ui.irwhite_free -> setEnabled(true);
    ui.sensor_value_free ->setEnabled(false);

    sensor_str = ui.IR_free -> text();

    sensor_int = 8;
}


//maze mode sensor
void MainWindow::on_Sonar_maze_clicked()
{
    ui.generally_maze -> setChecked(false);

    radio_mazemode_enable();
    radio_mazemode_unchecked();
    ui.smaller_maze -> setEnabled(true);
    ui.bigger_maze-> setEnabled(true);

    sensor_str = ui.Sonar_maze -> text();
    
    sensor_int = 1;
}


//common sensor value setting
//Sensor position
void MainWindow::on_sensor_position_free_currentIndexChanged(int index)
{
    sensor_position_int = 5;
    sensor_position_str ="None";

    if (index == 1){
        sensor_position_str = "forward";
        sensor_position_int = 0;
    }

    else if (index == 2){
        sensor_position_str = "bottom";
        sensor_position_int = 1;
}
    else if (index == 3)
 {       sensor_position_str = "left";
        sensor_position_int = 2;
}
    else if (index == 4)
 {       sensor_position_str = "right";
        sensor_position_int = 3;
}
    else if (index == 5)
 {       sensor_position_str = "up";
        sensor_position_int = 4;
}}

void MainWindow::on_sensor_position_maze_currentIndexChanged(int index)
{
    sensor_position_int = 5;
    sensor_position_str ="None";

    if (index == 1){
        sensor_position_str = "forward";
        sensor_position_int = 0;
    }
    else if (index == 2){
        sensor_position_str = "bottom";
        sensor_position_int = 1;
    }
    else if (index == 3)
    {   sensor_position_str = "left";
        sensor_position_int = 2;
    }
    else if (index == 4)
    {   sensor_position_str = "right";
        sensor_position_int = 3;
    }
    else if (index == 5)
    {   sensor_position_str = "up";
        sensor_position_int = 4;
    }
    
}

//sensor value
void MainWindow::on_sensor_value_free_valueChanged(int arg1)
{
     sensor_value_int_r = ui.sensor_value_free -> value();
}

void MainWindow::on_sensor_value_maze_valueChanged(int arg1)
{
     sensor_value_int_r = ui.sensor_value_maze -> value();

}

//smaller & bigger
void MainWindow::on_smaller_free_clicked()
{
    sensor_value_str = ui.smaller_free -> text();
    ui.bigger_free -> setEnabled(true);
    ui.smaller_free -> setEnabled(false);

    sensor_value_int = 1;
}

void MainWindow::on_bigger_free_clicked()
{
    sensor_value_str = ui.bigger_free -> text();
    ui.bigger_free -> setEnabled(false);
    ui.smaller_free -> setEnabled(true);

    sensor_value_int = 2;
}

void MainWindow::on_smaller_maze_clicked()
{
    sensor_value_str = ui.smaller_maze -> text();
    ui.bigger_maze -> setEnabled(true);
    ui.smaller_maze -> setEnabled(false);

    sensor_value_int = 1;
}

void MainWindow::on_bigger_maze_clicked()
{
    sensor_value_str = ui.bigger_maze-> text();
    ui.bigger_maze -> setEnabled(false);
    ui.smaller_maze -> setEnabled(true);

    sensor_value_int = 2;
}


//ir sensor value setting
void MainWindow::on_irblack_free_clicked()
{
    sensor_value_str = "black";
    sensor_value_int = 3;

    ui.irblack_free -> setEnabled(false);
    ui.irwhite_free -> setEnabled(true);
}

void MainWindow::on_irwhite_free_clicked()
{
    sensor_value_str = "white";
    sensor_value_int = 4;

    ui.irblack_free -> setEnabled(true);
    ui.irwhite_free -> setEnabled(false);
}

//persondetecting value setting
void MainWindow::on_exist_free_clicked()
{
    sensor_value_str = "exist";
    sensor_value_int = 7;
}

void MainWindow::on_none_free_clicked()
{
    sensor_value_str = "none";
    sensor_value_int = 8;
}

//motor direction setting
void MainWindow::on_motor_position_free_currentIndexChanged(int index)
{
    motor_direction_int = 5;
    motor_direction_str ="None";
    
    if (index == 1){
        motor_direction_str = "forward";
        motor_direction_int = 0;}

    else if (index == 2){
        motor_direction_str = "back";
        motor_direction_int = 1;}

    else if (index == 3){
        motor_direction_str = "left";
        motor_direction_int = 2;}

    else if (index == 4){
        motor_direction_str = "right";
        motor_direction_int = 3;}


}


void MainWindow::on_motor_position_maze_currentIndexChanged(int index)
{

    motor_direction_int = 5;
    motor_direction_str ="None";

    if (index == 1){
        motor_direction_str = "forward";
        motor_direction_int = 0;
    }

    else if (index == 2){
        motor_direction_str = "back";
        motor_direction_int = 1;
    }

    else if (index == 3){
        motor_direction_str = "left";
        motor_direction_int = 2;
    }

    else if (index == 4){
        motor_direction_str = "right";
        motor_direction_int = 3;
    }

}


//motor value setting
void MainWindow::on_motor_value_free_valueChanged(int arg1)
{
    motor_value_int = ui.motor_value_free -> value();
}

void MainWindow::on_motor_value_maze_valueChanged(int arg1)
{
     motor_value_int = ui.motor_value_maze -> value();
}

//pass button
void MainWindow::on_Add_free_clicked()
{
    ui.Temperature -> setChecked(0);
    ui.Sound -> setChecked(0);
    ui.Persondetection -> setChecked(0);
    ui.Illuminance -> setChecked(0);
    ui.IR_free -> setChecked(0);
    ui.Sonar_free -> setChecked(0);
    ui.generally_free -> setChecked(0);

    radio_freemode_enable();
    radio_freemode_unchecked();

    txt =  mode_str +" / "+ sensor_str +" / "+ sensor_position_str +" / "+ sensor_value_str +" / "+ QString::number(sensor_value_int) +" / "+ QString::number(motor_direction_int) +" / "+ QString::number(motor_value_int);
    ui.workspace_free -> addItem(txt);

    msg1.data = {sensor_int, sensor_position_int, sensor_value_int_r, sensor_value_int, motor_direction_int, motor_value_int};
    qnode.sendsensor(msg1);
}

void MainWindow::on_Add_maze_clicked()
{
    ui.Sonar_maze -> setChecked(0);
    ui.generally_maze -> setChecked(0);

    radio_mazemode_enable();
    radio_mazemode_unchecked();

    //sensor_value_int = ui.sensor_value_maze -> value();

    txt =  mode_str+" / " + sensor_str+" / " + sensor_position_str +" / "+ sensor_value_str +" / "+ QString::number(sensor_value_int) +" / "+ QString::number(motor_direction_int) +" / "+ QString::number(motor_value_int);
    ui.workspace_maze -> addItem(txt);

    msg1.data = {sensor_int, sensor_position_int, sensor_value_int_r, sensor_value_int, motor_direction_int, motor_value_int};
    qnode.sendsensor(msg1);
}

//reset button
void MainWindow::on_reset_free_clicked()
{
    ui.freemode -> setChecked(0);

    ui.Temperature -> setChecked(0);
    ui.Sound -> setChecked(0);
    ui.Persondetection -> setChecked(0);
    ui.Illuminance -> setChecked(0);
    ui.IR_free -> setChecked(0);
    ui.Sonar_free -> setChecked(0);
    ui.generally_free -> setChecked(0);

    radio_freemode_enable();
    radio_freemode_unchecked();

    std_msgs::Int32 msg;
    msg.data = {1};
    qnode.sendreset(msg);

    ui.workspace_free -> clear();

}

void MainWindow::on_reset_maze_clicked()
{
    ui.mazemode -> setChecked(0);

    ui.Sonar_maze -> setChecked(0);
    ui.generally_maze -> setChecked(0);

    radio_mazemode_enable();
    radio_mazemode_unchecked();

    std_msgs::Int32 msg;
    msg.data = {1};
    qnode.sendreset(msg);

    ui.workspace_maze -> clear();
}

void MainWindow::on_start_free_clicked()
{
    std_msgs::Int32 msg;
    msg.data = {1};
    qnode.sendstart(msg);

    ui.workspace_free -> clear();
}

void MainWindow::on_start_maze_clicked()
{
    std_msgs::Int32 msg;
    msg.data = {1};
    qnode.sendstart(msg);

    ui.workspace_maze -> clear();
}

void MainWindow::showValues(QString tex)
{
    std::cout << "Here is -mainwindow showvalues-" << std::endl;
   // tex = qnode.ss_sensor;
    ui.lineEdit_sensorvalue -> setText(tex);
}

void MainWindow::showPosition(QString tex)
{
    std::cout << "Here is -mainwindow showposition-" << std::endl;
    //tex = qnode.ss_position;
    ui.lineEdit_motorposition -> setText(tex);
}

//--------------------------------------
void MainWindow::on_Sonar_test_clicked()
{
    msg_testsensor.data = {1};
    qnode.sendtestsensor(msg_testsensor);
}

void MainWindow::on_IR_test_clicked()
{
    msg_testsensor.data = {3};
    qnode.sendtestsensor(msg_testsensor);

}
void MainWindow::on_Temperature_test_clicked()
{
    msg_testsensor.data = {4};
    qnode.sendtestsensor(msg_testsensor);

}
void MainWindow::on_Persondetection_test_clicked()
{
    msg_testsensor.data = {6};
    qnode.sendtestsensor(msg_testsensor);

}
void MainWindow::on_Sound_test_clicked()
{
    msg_testsensor.data = {7};
    qnode.sendtestsensor(msg_testsensor);

}
void MainWindow::on_Illuminance_test_clicked()
{
    msg_testsensor.data = {8};
    qnode.sendtestsensor(msg_testsensor);

}

//-------------------------------
void MainWindow::on_on_test_clicked()
{
    ui.Sonar_test -> setEnabled(false);
    ui.IR_test -> setEnabled(false);
    ui.Temperature_test -> setEnabled(false);
    ui.Persondetection_test -> setEnabled(false);
    ui.Sound_test -> setEnabled(false);
    ui.Illuminance_test -> setEnabled(false);

    ui.forward_test_p -> setEnabled(false);
    ui.backward_test_p -> setEnabled(false);
    ui.left_test_p -> setEnabled(false);
    ui.right_test_p -> setEnabled(false);
    ui.under_test_p -> setEnabled(false);

    msg_testonoff.data = {1};
    qnode.sendtestonoff(msg_testonoff);
}

void MainWindow::on_off_test_clicked()
{
    ui.Sonar_test -> setEnabled(true);
    ui.IR_test -> setEnabled(true);
    ui.Temperature_test -> setEnabled(true);
    ui.Persondetection_test -> setEnabled(true);
    ui.Sound_test -> setEnabled(true);
    ui.Illuminance_test -> setEnabled(true);

    ui.forward_test_p -> setEnabled(true);
    ui.backward_test_p -> setEnabled(true);
    ui.left_test_p -> setEnabled(true);
    ui.right_test_p -> setEnabled(true);
    ui.under_test_p -> setEnabled(true);

    msg_testonoff.data = {0};
    qnode.sendtestonoff(msg_testonoff);
}
//------------------------------------------
void MainWindow::on_forward_test_p_clicked()
{
    std_msgs::Int32 msg;
    msg.data = {0};
    qnode.sendtestposition(msg);
}

void MainWindow::on_backward_test_p_clicked()
{
    std_msgs::Int32 msg;
    msg.data = {1};
    qnode.sendtestposition(msg);
}

void MainWindow::on_left_test_p_clicked()
{
    std_msgs::Int32 msg;
    msg.data = {2};
    qnode.sendtestposition(msg);
}

void MainWindow::on_right_test_p_clicked()
{
    std_msgs::Int32 msg;
    msg.data = {3};
    qnode.sendtestposition(msg);
}

void MainWindow::on_under_test_p_clicked()
{
    std_msgs::Int32 msg;
    msg.data = {4};
    qnode.sendtestposition(msg);
}
//------------------------------------------
void MainWindow::on_forward_test_clicked()
{
    if(msg_testonoff.data == 1)
    {
        std_msgs::Int32 msg;
        msg.data = 0;
        qnode.sendtestmotor(msg); 
    }
}

void MainWindow::on_backward_test_clicked()
{
    if(msg_testonoff.data == 1)
    {
        std_msgs::Int32 msg;
        msg.data = 1;
        qnode.sendtestmotor(msg); 
    }
}

void MainWindow::on_left_test_clicked()
{
    if(msg_testonoff.data == 1)
    {
        std_msgs::Int32 msg;
        msg.data = 2;
        qnode.sendtestmotor(msg); 
    }
}

void MainWindow::on_right_test_clicked()
{
    if(msg_testonoff.data == 1)
    {
        std_msgs::Int32 msg;
        msg.data = 3;
        qnode.sendtestmotor(msg); 
    }
}

void MainWindow::on_stop_test_clicked()
{
    if(msg_testonoff.data == 1)
    {
        std_msgs::Int32 msg;
        msg.data = 4;
        qnode.sendtestmotor(msg); 
    }
}

//----------------------------------------
void MainWindow::showsensor(QString tex)
{
    ui.lineEdit_sensorvalue -> setText(tex);
}

}  // namespace lastlast

    //