#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  ros::init(argc, argv, "robot_control_gui_node");
  ros::NodeHandle nh;

  rvizRobot = new Rviz();
  rvizRobot->initRvizRobotModel(ui->vlRobotModel);

  rvizCamera = new Rviz();
  rvizCamera->initRvizCameraView(ui->labelCameraView);

  subCameraImage = nh.subscribe("/camera/rgb/image_raw", 1, &Rviz::cameraCallback, rvizCamera);
  subRobotState = nh.subscribe("/keti_robot_state", 1, &MainWindow::RobotStateCallback, this);
  subGripperState = nh.subscribe("/keti_gripper_state", 1, &MainWindow::GripperStateCallback, this);
}

MainWindow::~MainWindow()
{
  delete ui;
  delete rvizRobot;
  delete rvizCamera;
}

void MainWindow::RobotStateCallback(const keti_robot_control::RobotState &msg)
{
  ui->txtRobotState->setNum(msg.state);

  ui->txtJoint1->setText(QString::number(msg.current_joint[0] - offset[0]));
  ui->txtJoint2->setText(QString::number(msg.current_joint[1] - offset[1]));
  ui->txtJoint3->setText(QString::number(msg.current_joint[2] - offset[2]));
  ui->txtJoint4->setText(QString::number(msg.current_joint[3] - offset[3]));
  ui->txtJoint5->setText(QString::number(msg.current_joint[4] - offset[4]));
  ui->txtJoint6->setText(QString::number(msg.current_joint[5] - offset[5]));

  ui->txtMatrix1->setText(QString::number(msg.current_T_matrix[0]));
  ui->txtMatrix2->setText(QString::number(msg.current_T_matrix[1]));
  ui->txtMatrix3->setText(QString::number(msg.current_T_matrix[2]));
  ui->txtMatrix4->setText(QString::number(msg.current_T_matrix[3]));
  ui->txtMatrix5->setText(QString::number(msg.current_T_matrix[4]));
  ui->txtMatrix6->setText(QString::number(msg.current_T_matrix[5]));
  ui->txtMatrix7->setText(QString::number(msg.current_T_matrix[6]));
  ui->txtMatrix8->setText(QString::number(msg.current_T_matrix[7]));
  ui->txtMatrix9->setText(QString::number(msg.current_T_matrix[8]));
  ui->txtMatrix10->setText(QString::number(msg.current_T_matrix[9]));
  ui->txtMatrix11->setText(QString::number(msg.current_T_matrix[10]));
  ui->txtMatrix12->setText(QString::number(msg.current_T_matrix[11]));
  ui->txtMatrix13->setText(QString::number(msg.current_T_matrix[12]));
  ui->txtMatrix14->setText(QString::number(msg.current_T_matrix[13]));
  ui->txtMatrix15->setText(QString::number(msg.current_T_matrix[14]));
  ui->txtMatrix16->setText(QString::number(msg.current_T_matrix[15]));
}

void MainWindow::GripperStateCallback(const keti_robot_control::GripperState &msg)
{
  ui->txtGripperState->setNum(msg.state);
  ui->txtGripperWidth->setText(QString::number(msg.width));
}
