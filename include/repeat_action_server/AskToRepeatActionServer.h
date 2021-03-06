#ifndef __REPEAT_ACTION_SERVER__ASK_TO_REPEAT_ACTION_SERVER_H__
#define __REPEAT_ACTION_SERVER__ASK_TO_REPEAT_ACTION_SERVER_H__

#include<actionlib/server/simple_action_server.h>
#include<actionlib/client/simple_action_client.h>
#include<move_base_msgs/MoveBaseAction.h>
#include<actionlib/client/terminal_state.h>
#include<repeat_action_server/AskToRepeatAction.h>

#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<nav_msgs/Path.h>
#include<nav_msgs/Odometry.h>
#include<control_msgs/PointHeadAction.h>

class AskToRepeatActionServer {
  protected:
    ros::NodeHandle nh_;
    tf::TransformListener TFlistener_;

    std::string actionName_;
    actionlib::SimpleActionServer<repeat_action_server::AskToRepeatAction> actionServer_;
    repeat_action_server::AskToRepeatFeedback feedback_;
    repeat_action_server::AskToRepeatResult result_;
    
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> baseClient_;
    actionlib::SimpleActionClient<control_msgs::PointHeadAction> headClient_;

    ros::Subscriber odometrySub_;
    nav_msgs::Odometry currentOdom_;
    ros::ServiceClient client_;

    void robotOdometryCallback(const nav_msgs::Odometry message);
    geometry_msgs::Pose getPose(const std::string point, const std::string origin);
    void moveHead();
    void resetHead();
    nav_msgs::Path getPlan();

  public:
    AskToRepeatActionServer();
    ~AskToRepeatActionServer();
    void executeCallback(const repeat_action_server::AskToRepeatGoalConstPtr &goal);
};

#endif