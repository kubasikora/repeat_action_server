#include<repeat_action_server/AskToRepeatActionServer.h>
#include<repeat_action_server/util.h>
#include<std_msgs/String.h>

AskToRepeatActionServer::AskToRepeatActionServer() :
    actionServer_(nh_, getParamValue<std::string>("served_action_name"), boost::bind(&AskToRepeatActionServer::executeCallback, this, _1), false)
{
    actionServer_.start();
    ROS_INFO("%s server ready", getParamValue<std::string>("served_action_name").c_str());
}

AskToRepeatActionServer::~AskToRepeatActionServer(){}

void AskToRepeatActionServer::executeCallback(const repeat_action_server::AskToRepeatGoalConstPtr &goal){
    std_msgs::String resultStatus;
    resultStatus.data = "finished";
    result_.status = resultStatus;
    actionServer_.setSucceeded(result_);
    ROS_INFO("%s: Succeeded", actionName_.c_str());
}
