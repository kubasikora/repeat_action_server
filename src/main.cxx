#include<ros/ros.h>
#include<repeat_action_server/AskToRepeatActionServer.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "repeat");
    ROS_INFO("Action server ready to roll");
    ros::spin();
    return 0;
}