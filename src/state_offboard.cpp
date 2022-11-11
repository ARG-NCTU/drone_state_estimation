#include <iostream>
#include "math.h" 
#include "ros/ros.h"
#include "mavros_msgs/State.h"
#include "geometry_msgs/PoseStamped.h"
#include <behavior_tree/behavior_tree.h>
using namespace std; 

class Land{
    private:
        ros::NodeHandle n;
        ros::Subscriber msg_sub_state;
        bool mutex_lock = false;
        float current_altitude = 0;
    public:
        bt::Condition condition_land;
        Land();
        void conditionSet(bool state);
        void stateCallback(const mavros_msgs::State::ConstPtr& msg);
        void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
};

Land :: Land() : condition_land("state_offboard"){
    msg_sub_state = n.subscribe("mavros/state", 1,  &Land::stateCallback, this);
}

void Land :: conditionSet(bool state){
    condition_land.set(state);
    condition_land.publish();
    return;
}

void Land :: stateCallback(const mavros_msgs::State::ConstPtr& msg){
    if(mutex_lock == false){
        return;
    }else{
        mutex_lock = false;
        if(msg->mode == "OFFBOARD" || current_altitude < 0.1){
            ROS_INFO("Land success");
            conditionSet(true);
        }else{
            ROS_INFO("Land failed");
            conditionSet(false);
        }
        return;
    }
}

void Land :: positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    if(mutex_lock == false){
        mutex_lock = true;
        current_altitude = msg->pose.position.z;
        return;
    }else{
        return;
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "state_offboard");
    Land state_land;
    ros::spin();
    return 0;
}
