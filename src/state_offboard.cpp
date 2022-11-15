#include <iostream>
#include "math.h" 
#include "ros/ros.h"
#include "mavros_msgs/State.h"
#include "geometry_msgs/PoseStamped.h"
#include <behavior_tree/behavior_tree.h>
using namespace std; 

class Offboard{
    private:
        ros::NodeHandle n;
        ros::Subscriber msg_sub_state;
        bool mutex_lock = false;
        float current_altitude = 0;
    public:
        bt::Condition condition;
        Offboard();
        void conditionSet(bool state);
        void stateCallback(const mavros_msgs::State::ConstPtr& msg);
        void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
};

Offboard :: Offboard() : condition("state_offboard"){
    msg_sub_state = n.subscribe("mavros/state", 1,  &Offboard::stateCallback, this);
}

void Offboard :: conditionSet(bool state){
    condition.set(state);
    condition.publish();
    return;
}

void Offboard :: stateCallback(const mavros_msgs::State::ConstPtr& msg){
    if(msg->mode == "OFFBOARD"){
        ROS_INFO("Offboard success");
        conditionSet(true);
    }else{
        ROS_INFO("Offboard failed");
        conditionSet(false);
    }
    return;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "state_offboard");
    Offboard state_offboard;
    ros::spin();
    return 0;
}
