#pragma once

#include <ros/ros.h>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <sensor_msgs/Image.h>

#include <QLabel>

#include "image_view.h"

class Rviz{

public:
    Rviz();

    void initRvizRobotModel(void *_ui);
    void initRvizCameraView(void *_ui);

    void cameraCallback(const sensor_msgs::ImageConstPtr &msg);

private:
    rviz::VisualizationManager  *m_RvizManager;
    rviz::RenderPanel           *m_RvizRenderPanel;

    rviz::Display               *m_RvizGrid;
    rviz::Display               *m_RvizSetFixedFrame;
    rviz::Display               *m_RvizRobotModel;
    rviz::Display               *m_RvizCamera;

    ImageView                   *m_RvizImage;
    QLabel *labelImage;

    void setTopicRobot();
    void setTopicCamera();
};

