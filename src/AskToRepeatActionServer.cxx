#include<repeat_action_server/AskToRepeatActionServer.h>
#include<repeat_action_server/util.h>
#include<std_msgs/String.h>
#include<nav_msgs/GetPlan.h>
#include<exception>

void AskToRepeatActionServer::robotOdometryCallback(const nav_msgs::Odometry message){
    currentOdom_ = message;
}

nav_msgs::Path AskToRepeatActionServer::getPlan(){
    nav_msgs::GetPlan srv;
    srv.request.start.header.frame_id = "map";

    tf::StampedTransform humanTF;
    TFlistener_.lookupTransform(getParamValue<std::string>("human_tf"), "map", ros::Time(0), humanTF);
    tf::Transform humanTFinversed = humanTF.inverse();
    srv.request.goal.header.frame_id = "map";
    srv.request.goal.pose.position.x = humanTFinversed.getOrigin().x();
    srv.request.goal.pose.position.y = humanTFinversed.getOrigin().y();
    srv.request.goal.pose.position.z = humanTFinversed.getOrigin().z();
    srv.request.goal.pose.orientation.x = humanTFinversed.getRotation().x();
    srv.request.goal.pose.orientation.y = humanTFinversed.getRotation().y();
    srv.request.goal.pose.orientation.z = humanTFinversed.getRotation().z();
    srv.request.goal.pose.orientation.w = humanTFinversed.getRotation().w();

    tf::StampedTransform robotTF;
    TFlistener_.lookupTransform("base_link", "map", ros::Time(0), robotTF);
    tf::Transform robotTFinversed = robotTF.inverse();
    srv.request.start.header.frame_id = "map";
    srv.request.start.pose.position.x = robotTFinversed.getOrigin().x();
    srv.request.start.pose.position.y = robotTFinversed.getOrigin().y();
    srv.request.start.pose.position.z = robotTFinversed.getOrigin().z();
    srv.request.start.pose.orientation.x = robotTFinversed.getRotation().x();
    srv.request.start.pose.orientation.y = robotTFinversed.getRotation().y();
    srv.request.start.pose.orientation.z = robotTFinversed.getRotation().z();
    srv.request.start.pose.orientation.w = robotTFinversed.getRotation().w();

    srv.request.tolerance = 0.5;

    if (client_.call(srv)){
        ROS_INFO("Plan received");
        srv.response.plan.header.frame_id = "map";
        planPublisher_.publish(srv.response.plan);
        return srv.response.plan;
    }
    else {
        ROS_ERROR("Failed to call service %s", getParamValue<std::string>("get_plan_service_name").c_str());
        throw std::exception();
    }
}

AskToRepeatActionServer::AskToRepeatActionServer() :
    actionServer_(nh_, getParamValue<std::string>("served_action_name"), boost::bind(&AskToRepeatActionServer::executeCallback, this, _1), false)
{   
        odometrySub_ = nh_.subscribe(getParamValue<std::string>("odometry_topic"), 1000, &AskToRepeatActionServer::robotOdometryCallback, this);
    client_ = nh_.serviceClient<nav_msgs::GetPlan>(getParamValue<std::string>("get_plan_service_name"));
    planPublisher_ = nh_.advertise<nav_msgs::Path>("my_plan", 1000);
    
    actionServer_.start();
    ROS_INFO("%s server ready", getParamValue<std::string>("served_action_name").c_str());
}

AskToRepeatActionServer::~AskToRepeatActionServer(){}

void AskToRepeatActionServer::executeCallback(const repeat_action_server::AskToRepeatGoalConstPtr &goal){
    nav_msgs::Path plan = getPlan();

    std_msgs::String resultStatus;
    resultStatus.data = "finished";
    result_.status = resultStatus;
    actionServer_.setSucceeded(result_);
    ROS_INFO("%s: Succeeded", actionName_.c_str());
}
