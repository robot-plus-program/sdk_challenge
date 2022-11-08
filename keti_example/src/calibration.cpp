#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <librealsense2/rs.hpp>

static cv::Mat current_frame;
static rs2::frameset data;

void mouse_event(int event, int x, int y, int flags, void*)
{
    if(event == cv::EVENT_LBUTTONDOWN){
        ROS_INFO("x : %d, y : %d", x, y);
        ROS_INFO("z : %f", data.get_depth_frame().get_distance(x, y));
    }
}

void imageDepthCallback(const sensor_msgs::CompressedImageConstPtr &msg){
    ROS_INFO("depth size : %ld", msg.get()->data.size());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keti_calibration_sample_node");

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    image_transport::ImageTransport it(nh);

    // Subscribe to the /camera topic
    // image_transport::Subscriber sub = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, imageColorCallback);
    // image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1, imageColorCallback);
    // rospy.Subscriber('/camera/aligned_depth_to_color/image_raw/compressedDepth', CompressedImage, depthImagCallback)


    ros::Rate rate(10);

    std::string window_name = "result";
    cv::namedWindow(window_name);
    cv::setMouseCallback(window_name, mouse_event);

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    pipe.start();

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();

        data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame depth = data.get_color_frame().apply_filter(color_map);

        // Query frame size (width and height)
        int w = depth.as<rs2::video_frame>().get_width();
        int h = depth.as<rs2::video_frame>().get_height();

        // ROS_INFO("w : %d, h : %d", w, h);

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        cv::Mat frame(cv::Size(w, h), CV_8UC3, (void*)depth.get_data(), cv::Mat::AUTO_STEP);

        frame.copyTo(current_frame);

        // Display the current frame
        if(!current_frame.empty())
            cv::imshow(window_name, current_frame);

        cv::waitKey(1);
    }

    // Close down OpenCV
    cv::destroyWindow(window_name);
}