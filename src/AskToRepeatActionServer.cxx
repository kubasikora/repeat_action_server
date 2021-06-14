#include<repeat_action_server/AskToRepeatActionServer.h>
#include<repeat_action_server/util.h>
#include<std_msgs/String.h>
#include<nav_msgs/GetPlan.h>
#include<exception>
#include<math.h>

void AskToRepeatActionServer::robotOdometryCallback(const nav_msgs::Odometry message){
    currentOdom_ = message;
}

geometry_msgs::Pose AskToRepeatActionServer::getPose(const std::string point, const std::string origin){
    tf::StampedTransform transformation;
    TFlistener_.lookupTransform(point, origin, ros::Time(0), transformation);
    tf::Transform inversedTransformation = transformation.inverse();

    geometry_msgs::Pose pose;
    pose.position.x = inversedTransformation.getOrigin().x();
    pose.position.y = inversedTransformation.getOrigin().y();
    pose.position.z = inversedTransformation.getOrigin().z();
    pose.orientation.x = inversedTransformation.getRotation().x();
    pose.orientation.y = inversedTransformation.getRotation().y();
    pose.orientation.z = inversedTransformation.getRotation().z();
    pose.orientation.w = inversedTransformation.getRotation().w();

    return pose;
}

nav_msgs::Path AskToRepeatActionServer::getPlan(){
    nav_msgs::GetPlan srv;
    srv.request.start.header.frame_id = "map";
    srv.request.start.pose = getPose("base_link", "map");
    srv.request.goal.header.frame_id = "map";
    srv.request.goal.pose = getPose(getParamValue<std::string>("human_tf"), "map");
    srv.request.tolerance = 0.5;

    if (client_.call(srv)){
        srv.response.plan.header.frame_id = "map";
    
        const geometry_msgs::Pose humanPose = srv.request.goal.pose;
        std::vector<geometry_msgs::PoseStamped> finalPosesList;
        for(geometry_msgs::PoseStamped pose : srv.response.plan.poses){
            const double distance = std::sqrt(std::pow(pose.pose.position.x - humanPose.position.x, 2) + std::pow(pose.pose.position.y - humanPose.position.y, 2));
            if(distance >= 1.0){
                finalPosesList.push_back(pose);
            }
        }
        srv.response.plan.poses = finalPosesList;
        return srv.response.plan;
    }
    else {
        ROS_ERROR("Failed to call service %s", getParamValue<std::string>("get_plan_service_name").c_str());
        throw std::exception();
    }
}

AskToRepeatActionServer::AskToRepeatActionServer() :
    actionServer_(nh_, getParamValue<std::string>("served_action_name"), boost::bind(&AskToRepeatActionServer::executeCallback, this, _1), false),
    actionClient_("move_base", true)
{   
    odometrySub_ = nh_.subscribe(getParamValue<std::string>("odometry_topic"), 1000, &AskToRepeatActionServer::robotOdometryCallback, this);
    client_ = nh_.serviceClient<nav_msgs::GetPlan>(getParamValue<std::string>("get_plan_service_name"));
    actionClient_.waitForServer();

    actionServer_.start();
    ROS_INFO("%s server ready", getParamValue<std::string>("served_action_name").c_str());
}

AskToRepeatActionServer::~AskToRepeatActionServer(){}

void AskToRepeatActionServer::executeCallback(const repeat_action_server::AskToRepeatGoalConstPtr &goal){
    nav_msgs::Path plan = getPlan();
    geometry_msgs::PoseStamped goalPose = plan.poses[plan.poses.size() - 1];

    move_base_msgs::MoveBaseGoal navigationGoal;
    navigationGoal.target_pose = goalPose;
    actionClient_.sendGoal(navigationGoal);

    std_msgs::String resultStatus;
    resultStatus.data = "finished";
    result_.status = resultStatus;
    actionServer_.setSucceeded(result_);
    ROS_INFO("%s: Succeeded", actionName_.c_str());
}
