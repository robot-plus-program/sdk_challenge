#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/tool_manager.h>

#include <keti_robot_control/RobotState.h>
#include <keti_robot_control/GripperState.h>

#include "Rviz/rviz.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    
    Rviz *rvizRobot, *rvizCamera;

    ros::Subscriber subCameraImage;
    ros::Subscriber subRobotState, subGripperState;

    void RobotStateCallback(const keti_robot_control::RobotState &msg);
    void GripperStateCallback(const keti_robot_control::GripperState &msg);

};
#endif // MAINWINDOW_H
