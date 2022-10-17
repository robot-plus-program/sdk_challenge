#include "robot_move_sim_action.h"

RobotMoveSimActionClass::RobotMoveSimActionClass(std::string name, ros::NodeHandle nh) : as(nh, name, boost::bind(&RobotMoveSimActionClass::executeCB, this, _1), false), action_name(name)
{
    robot_state = 0;
    memset(current_joint, 0, sizeof(double)*6);
    memset(current_position, 0, sizeof(double)*3);
    memset(current_rotation, 0, sizeof(double)*3);
    memset(current_T_matrix, 0, sizeof(double)*16);

    move_group_interface = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup *joint_model_group = move_group_interface->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    ROS_INFO_NAMED(PLANNING_GROUP.c_str(), "Planning frame: %s", move_group_interface->getPlanningFrame().c_str());
    ROS_INFO_NAMED(PLANNING_GROUP.c_str(), "End effector link: %s", move_group_interface->getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED(PLANNING_GROUP.c_str(), "Available Planning Groups:");
    std::copy(move_group_interface->getJointModelGroupNames().begin(), move_group_interface->getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    // Environment visualization
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface->getPlanningFrame();
    collision_object.id = "box";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 1.850;
    primitive.dimensions[primitive.BOX_Y] = 0.9;
    primitive.dimensions[primitive.BOX_Z] = 0.7;

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.45;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.35;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    moveit_msgs::ObjectColor color_box;
    color_box.id = collision_object.id;
    color_box.color.a = 1.0;
    color_box.color.r = 0.9;
    color_box.color.g = 0.9;
    color_box.color.b = 0.9;

    moveit_msgs::CollisionObject collision_object_center;
    collision_object_center.header.frame_id = move_group_interface->getPlanningFrame();
    collision_object_center.id = "center";

    shape_msgs::SolidPrimitive primitive_center;
    primitive_center.type = primitive_center.BOX;
    primitive_center.dimensions.resize(3);
    primitive_center.dimensions[primitive_center.BOX_X] = 0.05;
    primitive_center.dimensions[primitive_center.BOX_Y] = 0.9;
    primitive_center.dimensions[primitive_center.BOX_Z] = 0.01;
    
    geometry_msgs::Pose box_center_pose;
    box_center_pose.orientation.w = 1.0;
    box_center_pose.position.x = box_pose.position.x;
    box_center_pose.position.y = 0.0;
    box_center_pose.position.z = 0.0;

    collision_object_center.primitives.push_back(primitive_center);
    collision_object_center.primitive_poses.push_back(box_center_pose);
    collision_object_center.operation = collision_object_center.ADD;

    moveit_msgs::ObjectColor color_center;
    color_center.id = collision_object_center.id;
    color_center.color.a = 1.0;
    color_center.color.r = 1.0;
    color_center.color.g = 1.0;
    color_center.color.b = 1.0;

    moveit_msgs::CollisionObject collision_object_obstacle;
    collision_object_obstacle.header.frame_id = move_group_interface->getPlanningFrame();
    collision_object_obstacle.id = "obstacle";

    shape_msgs::SolidPrimitive primitive_obstacle;
    primitive_obstacle.type = primitive_obstacle.BOX;
    primitive_obstacle.dimensions.resize(3);
    primitive_obstacle.dimensions[primitive_obstacle.BOX_X] = 0.5;
    primitive_obstacle.dimensions[primitive_obstacle.BOX_Y] = 0.05;
    primitive_obstacle.dimensions[primitive_obstacle.BOX_Z] = 0.15;

    geometry_msgs::Pose box_obstacle_pose;
    box_obstacle_pose.orientation.w = 1.0;
    box_obstacle_pose.position.x = box_pose.position.x + 0.25 + 0.2;
    box_obstacle_pose.position.y = 0;
    box_obstacle_pose.position.z = 0;

    collision_object_obstacle.primitives.push_back(primitive_obstacle);
    collision_object_obstacle.primitive_poses.push_back(box_obstacle_pose);
    collision_object_obstacle.operation = collision_object_obstacle.ADD;

    moveit_msgs::ObjectColor color_obstacle;
    color_obstacle.id = collision_object_obstacle.id;
    color_obstacle.color.a = 1.0;
    color_obstacle.color.r = 0.5;
    color_obstacle.color.g = 0.5;
    color_obstacle.color.b = 0.5;

    moveit_msgs::CollisionObject collision_object_pick;
    collision_object_pick.header.frame_id = move_group_interface->getPlanningFrame();
    collision_object_pick.id = "pick";

    shape_msgs::SolidPrimitive primitive_pick;
    primitive_pick.type = primitive_pick.BOX;
    primitive_pick.dimensions.resize(3);
    primitive_pick.dimensions[primitive_pick.BOX_X] = 0.4;
    primitive_pick.dimensions[primitive_pick.BOX_Y] = 0.4;
    primitive_pick.dimensions[primitive_pick.BOX_Z] = 0.02;

    geometry_msgs::Pose box_pick_pose;
    box_pick_pose.orientation.w = 1.0;
    box_pick_pose.position.x = box_obstacle_pose.position.x;
    box_pick_pose.position.y = -0.25;
    box_pick_pose.position.z = box_obstacle_pose.position.z;

    collision_object_pick.primitives.push_back(primitive_pick);
    collision_object_pick.primitive_poses.push_back(box_pick_pose);
    collision_object_pick.operation = collision_object_pick.ADD;

    moveit_msgs::ObjectColor color_pick;
    color_pick.id = collision_object_pick.id;
    color_pick.color.a = 1.0;
    color_pick.color.r = 0.7;
    color_pick.color.g = 0.7;
    color_pick.color.b = 0.7;

    moveit_msgs::CollisionObject collision_object_place;
    collision_object_place.header.frame_id = move_group_interface->getPlanningFrame();
    collision_object_place.id = "place";

    shape_msgs::SolidPrimitive primitive_place;
    primitive_place.type = primitive_place.BOX;
    primitive_place.dimensions.resize(3);
    primitive_place.dimensions[primitive_place.BOX_X] = 0.4;
    primitive_place.dimensions[primitive_place.BOX_Y] = 0.4;
    primitive_place.dimensions[primitive_place.BOX_Z] = 0.02;

    geometry_msgs::Pose box_place_pose;
    box_place_pose.orientation.w = 1.0;
    box_place_pose.position.x = box_obstacle_pose.position.x;
    box_place_pose.position.y = 0.25;
    box_place_pose.position.z = box_obstacle_pose.position.z;

    collision_object_place.primitives.push_back(primitive_place);
    collision_object_place.primitive_poses.push_back(box_place_pose);
    collision_object_place.operation = collision_object_place.ADD;

    moveit_msgs::ObjectColor color_place;
    color_place.id = collision_object_place.id;
    color_place.color.a = 1.0;
    color_place.color.r = 0.7;
    color_place.color.g = 0.7;
    color_place.color.b = 0.7;

    moveit_msgs::CollisionObject collision_object_metal;
    collision_object_metal.header.frame_id = move_group_interface->getPlanningFrame();
    collision_object_metal.id = "metal";

    shape_msgs::SolidPrimitive primitive_metal;
    primitive_metal.type = primitive_metal.CYLINDER;
    primitive_metal.dimensions.resize(2);
    primitive_metal.dimensions[primitive_metal.CYLINDER_HEIGHT] = 0.14;
    primitive_metal.dimensions[primitive_metal.CYLINDER_RADIUS] = 0.053*0.5;

    geometry_msgs::Pose cylinder_metal;
    cylinder_metal.orientation.w = 1.0;
    cylinder_metal.position.x = box_pick_pose.position.x;
    cylinder_metal.position.y = box_pick_pose.position.y;
    cylinder_metal.position.z = box_pick_pose.position.z + 0.14*0.5 + 0.02;

    collision_object_metal.primitives.push_back(primitive_metal);
    collision_object_metal.primitive_poses.push_back(cylinder_metal);
    collision_object_metal.operation = collision_object_metal.ADD;

    moveit_msgs::ObjectColor color_metal;
    color_metal.id = collision_object_metal.id;
    color_metal.color.a = 1.0;
    color_metal.color.r = 0.0;
    color_metal.color.g = 0.0;
    color_metal.color.b = 1.0;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    collision_objects.push_back(collision_object_center);
    collision_objects.push_back(collision_object_obstacle);
    collision_objects.push_back(collision_object_pick);
    collision_objects.push_back(collision_object_place);
    collision_objects.push_back(collision_object_metal);

    std::vector<moveit_msgs::ObjectColor> colors;
    colors.push_back(color_box);
    colors.push_back(color_center);
    colors.push_back(color_obstacle);
    colors.push_back(color_pick);
    colors.push_back(color_place);
    colors.push_back(color_metal);

    std::vector<std::string> object_ids;
    for(unsigned int i = 0; i < collision_objects.size(); i++){
        object_ids.push_back(collision_objects[i].id);
    }

    planning_scene_interface.removeCollisionObjects(object_ids);

    planning_scene_interface.addCollisionObjects(collision_objects, colors);

    // move to initial pose, move Joint sapce
    move_group_interface->setPlanningTime(2.0);
    move_group_interface->setJointValueTarget(init_joint);
    success = (move_group_interface->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED(PLANNING_GROUP.c_str(), "Visualizing plan 1 (joint goal) %s", success ? "SUCCEED" : "FAILED");
    if(success){
        move_group_interface->asyncExecute(plan);
        ros::Duration().sleep();
    }

    as.start();
}

RobotMoveSimActionClass::~RobotMoveSimActionClass()
{
}

void RobotMoveSimActionClass::getCurrentJoint(std::vector<double> *vec_dst)
{
    std::vector<double> vec_src = move_group_interface->getCurrentJointValues();
    std::copy(vec_src.begin(), vec_src.end(), vec_dst->begin());
}

void RobotMoveSimActionClass::getCurrentPose(geometry_msgs::Pose *pose_dst)
{
    geometry_msgs::PoseStamped current_pose = move_group_interface->getCurrentPose();
    pose_dst->orientation = current_pose.pose.orientation;
    pose_dst->position = current_pose.pose.position;
}

void RobotMoveSimActionClass::RobotStateCallback(const keti_msgs::RobotState &msg)
{
    // ROS_INFO("robot_state : %d", msg.robot_state);
    // ROS_INFO("current joint : %f, %f, %f, %f, %f, %f", msg.current_joint[0], msg.current_joint[1], msg.current_joint[2], msg.current_joint[3], msg.current_joint[4], msg.current_joint[5]);
    // ROS_INFO("current position : %f, %f, %f", msg.current_position[0], msg.current_position[1], msg.current_position[2]);
    // ROS_INFO("current rotation : %f, %f, %f", msg.current_rotation[0], msg.current_rotation[1], msg.current_rotation[2]);
    // ROS_INFO("current_T_matrix 1: %f, %f, %f, %f", msg.current_T_matrix[0], msg.current_T_matrix[1], msg.current_T_matrix[2], msg.current_T_matrix[3]);
    // ROS_INFO("current_T_matrix 2: %f, %f, %f, %f", msg.current_T_matrix[4], msg.current_T_matrix[5], msg.current_T_matrix[6], msg.current_T_matrix[7]);
    // ROS_INFO("current_T_matrix 3: %f, %f, %f, %f", msg.current_T_matrix[8], msg.current_T_matrix[9], msg.current_T_matrix[10], msg.current_T_matrix[11]);
    // ROS_INFO("current_T_matrix 4: %f, %f, %f, %f", msg.current_T_matrix[12], msg.current_T_matrix[13], msg.current_T_matrix[14], msg.current_T_matrix[15]);

    // robot_state = msg.robot_state;
    memcpy(current_joint, msg.current_joint.data(), sizeof(double) * 6);
    memcpy(current_position, msg.current_position.data(), sizeof(double) * 3);
    memcpy(current_rotation, msg.current_rotation.data(), sizeof(double) * 3);
    memcpy(current_T_matrix, msg.current_T_matrix.data(), sizeof(double) * 16);
}

void RobotMoveSimActionClass::RobotMoveStateCallback(const moveit_msgs::ExecuteTrajectoryActionFeedback &feedback)
{
    // ROS_INFO("robot_state : %d", feedback.status.status);
    if(feedback.status.status == 1){
        robot_state = 2;
    }
    else if(feedback.status.status == 3){
        robot_state = 1;
    }
}

void RobotMoveSimActionClass::executeCB(const keti_robot::RobotMoveGoalConstPtr &goal)
{
    // helper variables
    ros::Rate r(10);
    bool success = true;

    feedback.sequence.clear();
    result.sequence.clear();

    // publish info to the console for the user
    // ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->cmd, feedback_.sequence[0], feedback_.sequence[1]);
    ROS_INFO("%s: Executing, creating robot move sequence, cmd : %d, value : %f, %f, %f, %f, %f, %f, velocity : %f",
             action_name.c_str(), goal->cmd, goal->value[0], goal->value[1], goal->value[2], goal->value[3], goal->value[4], goal->value[5], goal->velocity);

    double value[6];
    memcpy(value, goal->value.data(), sizeof(double)*6);
    if (goal->cmd == 1)
    {
        // movej(value);
        std::vector<double> target_joint(6,0);
        memcpy(target_joint.data(), value, sizeof(double)*6);
        move_group_interface->setPlanningTime(2.0);
        move_group_interface->setJointValueTarget(target_joint);
        success = (move_group_interface->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED(PLANNING_GROUP.c_str(), "Visualizing plan (joint goal) %s", success ? "SUCCEED" : "FAILED");
        if(success){
            move_group_interface->asyncExecute(plan);
            ros::Duration().sleep();
        }
    }
    else if (goal->cmd == 2)
    {
        double mat[9] = {0,};
        double ang_z1 = goal->value[3], ang_y = goal->value[4], ang_z2 = goal->value[5];
        double Rz1[9] = {cos(ang_z1), -sin(ang_z1), 0, sin(ang_z1), cos(ang_z1), 0, 0, 0, 1};
        double Ry[9] = {cos(ang_y), 0, sin(ang_y), 0, 1, 0, -sin(ang_y), 0, cos(ang_y)};
        double Rz2[9] = {cos(ang_z2), -sin(ang_z2), 0, sin(ang_z2), cos(ang_z2), 0, 0, 0, 1};
        double Rz1y[9] = {0,};
        double temp = 0;
        for(unsigned int i = 0; i < 3; i++){
            for(unsigned int j = 0; j < 3; j++){
                temp = 0;
                for(unsigned int k = 0; k < 3; k++){
                    temp += Rz1[i*3 + k]*Ry[k*3 + j];
                }
                Rz1y[i*3 + j] = temp;
            }
        }

        for(unsigned int i = 0; i < 3; i++){
            for(unsigned int j = 0; j < 3; j++){
                temp = 0;
                for(unsigned int k = 0; k < 3; k++){
                    temp += Rz1y[i*3 + k]*Rz2[k*3 + j];
                }
                mat[i*3 + j] = temp;
            }
        }

        double result_mat[16];
        memcpy(result_mat, current_T_matrix, sizeof(double)*16);
        result_mat[3]  = value[0];
        result_mat[7]  = value[1];
        result_mat[11] = value[2];
        for(unsigned int i = 0; i < 3; i++){
            for(unsigned int j = 0; j < 3; j++){
                result_mat[i*4 + j] = mat[i*3 + j];
            }
        }

        geometry_msgs::Pose current_pose = move_group_interface->getCurrentPose().pose;
        geometry_msgs::Pose target_pose;

        target_pose = current_pose;
        target_pose.position.x = value[0];
        target_pose.position.y = value[1];
        target_pose.position.z = value[2];

        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(current_pose);
        waypoints.push_back(target_pose);

        moveit_msgs::RobotTrajectory trajectory_pick;
        double jump_threshold = 0.0;
        double eef_step = 0.01;
        double fraction = move_group_interface->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_pick);
        ROS_INFO_NAMED(PLANNING_GROUP.c_str(), "Visualizing plan (Cartesian path to pick) (%.2f%% achieved)", fraction * 100.0);

        // movel(base, result_mat);
        if(fraction >= 0.7){
            move_group_interface->asyncExecute(trajectory_pick);
            ros::Duration().sleep();
        }
    }
    else if(goal->cmd == 3){
        geometry_msgs::Pose current_pose = move_group_interface->getCurrentPose().pose;
        geometry_msgs::Pose target_pose;

        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(current_pose);

        int num = goal->num;

        for(int i = 0; i < num; i++){
            target_pose = current_pose;
            target_pose.position.x = goal->value[i*6 + 0];
            target_pose.position.y = goal->value[i*6 + 1];
            target_pose.position.z = goal->value[i*6 + 2];
            waypoints.push_back(target_pose);
        }

        moveit_msgs::RobotTrajectory trajectory_pick;
        double jump_threshold = 0.0;
        double eef_step = 0.01;
        double fraction = move_group_interface->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_pick);
        ROS_INFO_NAMED(PLANNING_GROUP.c_str(), "Visualizing plan (Cartesian path to pick) (%.2f%% achieved)", fraction * 100.0);

        // movel(base, result_mat);
        if(fraction >= 0.7){
            move_group_interface->asyncExecute(trajectory_pick);
            ros::Duration().sleep();
        }
    }
    else if(goal->cmd == 4){
        move_group_interface->attachObject("metal", "link6");
    }
    else if(goal->cmd == 5){
        move_group_interface->detachObject("metal");
    }

    int count = 0;
    bool moving = false;
    while(ros::ok()){
        r.sleep();
        ROS_INFO("Current robot state : %d", robot_state);

        if (as.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", action_name.c_str());
            // set the action state to preempted
            as.setPreempted();
            success = false;
            // Stop();
            break;
        }

        feedback.sequence.push_back(robot_state);
        as.publishFeedback(feedback);

        if(robot_state == 1) count++;
        if(count >= 30) break;

        if(robot_state == 2) moving = true;
        if(robot_state == 1 && moving) break;

        if(goal->cmd == 4 || goal->cmd == 5) break;
    }

    if (success)
    {
        result.sequence.push_back(1);
        ROS_INFO("%s: Succeeded", action_name.c_str());
        // set the action state to succeeded
        as.setSucceeded(result);
    }
}