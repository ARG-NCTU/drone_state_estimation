#include <iostream>
#include "math.h" 
#include "ros/ros.h"
#include "mavros_msgs/State.h"
#include <behavior_tree/behavior_tree.h>
using namespace std; 

class Arm{
    private:
        ros::NodeHandle n;
        ros::Subscriber msg_sub_state;
    public:
        bt::Condition condition_arm;
        Arm();
        void conditionSet(bool state);
        void stateCallback(const mavros_msgs::State::ConstPtr& msg);
};

Arm :: Arm() : condition_arm("Is Armed"){
    msg_sub_state = n.subscribe("mavros/state", 1,  &Arm::stateCallback, this);
}

void Arm :: conditionSet(bool state){
    condition_arm.set(state);
    condition_arm.publish();
    return;
}

void Arm :: stateCallback(const mavros_msgs::State::ConstPtr& msg){
    if(msg->armed == true){
        ROS_INFO("Arm success");
        conditionSet(true);
    }else{
        ROS_INFO("Arm failed");
        conditionSet(false);
    }
    return;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "state_arm");
    Arm state_arm;
    ros::spin();
    return 0;
}