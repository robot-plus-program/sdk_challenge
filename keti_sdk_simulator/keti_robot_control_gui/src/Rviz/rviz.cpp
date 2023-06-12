#include "rviz.h"
#include "ui_mainwindow.h"

Rviz::Rviz()
{
}

void Rviz::initRvizRobotModel(void *_layout)
{
  	QVBoxLayout *layout = static_cast<QVBoxLayout *>(_layout);

	m_RvizRenderPanel = new rviz::RenderPanel();

	layout->addWidget(m_RvizRenderPanel);

	m_RvizManager = new rviz::VisualizationManager(m_RvizRenderPanel);

	m_RvizRenderPanel->initialize(m_RvizManager->getSceneManager(), m_RvizManager);
	m_RvizManager->initialize();
	m_RvizManager->startUpdate();

	setTopicRobot();
}

void Rviz::initRvizCameraView(void *_layout)
{
	labelImage = static_cast<QLabel*>(_layout);
}

void Rviz::setTopicRobot()
{
	m_RvizManager->setFixedFrame("world");

	m_RvizGrid = m_RvizManager->createDisplay("rviz/Grid", "adjustable grid", true);
	m_RvizGrid->subProp("Line Style")->setValue("Billboards");

	m_RvizRobotModel = m_RvizManager->createDisplay("rviz/RobotModel", "robotmodel", true);
  	m_RvizRobotModel->subProp("Robot Description")->setValue("robot_description");
}

void Rviz::cameraCallback(const sensor_msgs::ImageConstPtr &msg)
{
//  ROS_INFO("%d, %d, %s", msg->width, msg->height, msg->encoding.c_str());
	QImage::Format format = QImage::Format_RGB888;
	QImage source_image(&msg->data[0], static_cast<int>(msg->width), static_cast<int>(msg->height), format);
//	source_image = source_image.rgbSwapped();
	QImage small = source_image.scaled(msg->width/3*2, msg->height/3*2, Qt::KeepAspectRatio);

	labelImage->setPixmap(QPixmap::fromImage((small)));
}
