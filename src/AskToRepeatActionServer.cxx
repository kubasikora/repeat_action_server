#include<repeat_action_server/AskToRepeatActionServer.h>
#include<repeat_action_server/util.h>
#include<std_msgs/String.h>
#include<nav_msgs/GetPlan.h>
#include<exception>
#include<math.h>

void AskToRepeatActionServer::resetHead(){
    control_msgs::PointHeadGoal goal; 
    goal.target.header.frame_id = getParamValue<std::string>("base_link");
    goal.target.point.x = 1.0; goal.target.point.y = 0.0; goal.target.point.z = 1.0;
    goal.pointing_axis.x        = 1.0; 
    goal.pointing_axis.y        = 0.0; 
    goal.pointing_axis.z        = 0.0;
    goal.pointing_frame         = getParamValue<std::string>("/head_controller/point_head_action/tilt_link");
    goal.max_velocity           = getParamValue<double>("head_turning_velocity");
    headClient_.sendGoal(goal);
}

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
        const double distanceToHuman = getParamValue<double>("distance_to_human");
        for(geometry_msgs::PoseStamped pose : srv.response.plan.poses){
            const double distance = std::sqrt(std::pow(pose.pose.position.x - humanPose.position.x, 2) + std::pow(pose.pose.position.y - humanPose.position.y, 2));
            if(distance >= distanceToHuman){
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

void AskToRepeatActionServer::moveHead(){
    control_msgs::PointHeadGoal goal;
    geometry_msgs::Pose pose    = getPose(getParamValue<std::string>("human_tf"), getParamValue<std::string>("base_link"));
    goal.target.header.frame_id = getParamValue<std::string>("base_link");
    goal.target.point.x         = pose.position.x; 
    goal.target.point.y         = pose.position.y; 
    goal.target.point.z         = pose.position.z;
    goal.pointing_axis.x = 1.0; goal.pointing_axis.y = 0.0; goal.pointing_axis.z = 0.0;
    goal.pointing_frame = getParamValue<std::string>("/head_controller/point_head_action/tilt_link");
    goal.max_velocity = getParamValue<double>("head_turning_velocity");
    headClient_.sendGoal(goal);
    
    while(ros::ok()){
        actionlib::SimpleClientGoalState state = headClient_.getState();
        if(state.isDone()){
            return;
        }
    }
}

AskToRepeatActionServer::AskToRepeatActionServer() :
    actionServer_(nh_, getParamValue<std::string>("served_action_name"), boost::bind(&AskToRepeatActionServer::executeCallback, this, _1), false),
    baseClient_(getParamValue<std::string>("move_base_action_name"), true),
    headClient_(getParamValue<std::string>("move_head_action_name"), true)
{   
    odometrySub_ = nh_.subscribe(getParamValue<std::string>("odometry_topic"), 1000, &AskToRepeatActionServer::robotOdometryCallback, this);
    client_ = nh_.serviceClient<nav_msgs::GetPlan>(getParamValue<std::string>("get_plan_service_name"));
    baseClient_.waitForServer();
    headClient_.waitForServer();
    
    actionServer_.start();
    ROS_INFO("%s server ready", getParamValue<std::string>("served_action_name").c_str());
}

AskToRepeatActionServer::~AskToRepeatActionServer(){}

void AskToRepeatActionServer::executeCallback(const repeat_action_server::AskToRepeatGoalConstPtr &goal){
    resetHead();
    nav_msgs::Path plan = getPlan();
    if(!plan.poses.empty()){
        geometry_msgs::PoseStamped goalPose = plan.poses[plan.poses.size() - 1];
        move_base_msgs::MoveBaseGoal navigationGoal;
        navigationGoal.target_pose = goalPose;
        baseClient_.sendGoal(navigationGoal);
        baseClient_.waitForResult();
        moveHead();
    }

    std_msgs::String resultStatus;
    resultStatus.data = "finished";
    result_.status = resultStatus;
    actionServer_.setSucceeded(result_);
    ROS_INFO("%s: Succeeded", actionName_.c_str());
}
