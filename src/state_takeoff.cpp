#include <iostream>
#include "math.h" 
#include "ros/ros.h"
#include "mavros_msgs/State.h"
#include "geometry_msgs/PoseStamped.h"
#include <behavior_tree/behavior_tree.h>
using namespace std; 

class Takeoff{
    private:
        ros::NodeHandle n;
        ros::Subscriber msg_sub_state;
        ros::Subscriber msg_sub_pose;
        bool mutex_lock = false;
        float current_altitude = 0;
        float desired_height = 2.0;
    public:
        bt::Condition condition_takeoff;
        Takeoff();
        void conditionSet(bool state);
        void stateCallback(const mavros_msgs::State::ConstPtr& msg);
        void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
};

Takeoff :: Takeoff() : condition_takeoff("Takeoff Complete"){
    msg_sub_state = n.subscribe("mavros/state", 1,  &Takeoff::stateCallback, this);
    msg_sub_pose = n.subscribe("mavros/local_position/pose", 1,  &Takeoff::positionCallback, this);
}

void Takeoff :: conditionSet(bool state){
    condition_takeoff.set(state);
    condition_takeoff.publish();
    return;
}

void Takeoff :: stateCallback(const mavros_msgs::State::ConstPtr& msg){
    if(mutex_lock == false){
        return;
    }else{
        mutex_lock = false;
        if(msg->mode == "AUTO.TAKEOFF" || current_altitude > desired_height){
            ROS_INFO("Takeoff success");
            conditionSet(true);
        }else{
            ROS_INFO("Takeoff failed");
            conditionSet(false);
        }
        return;
    }
}

void Takeoff :: positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    if(mutex_lock == false){
        mutex_lock = true;
        current_altitude = msg->pose.position.z;
        return;
    }else{
        return;
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "state_takeoff");
    Takeoff state_takeoff;
    ros::spin();
    return 0;
}
