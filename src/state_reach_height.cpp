#include <iostream>
#include "math.h" 
#include "ros/ros.h"
#include "mavros_msgs/State.h"
#include "geometry_msgs/PoseStamped.h"
#include <behavior_tree/behavior_tree.h>
using namespace std; 

class State{
    private:
        ros::NodeHandle n;
        ros::Subscriber msg_sub_pose;
        bool mutex_lock = false;
        float current_altitude = 0;
        float desired_height = 0;
    public:
        bt::Condition condition_takeoff;
        State();
        void conditionSet(bool state);
        void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
};

State :: State() : condition_takeoff("Reach Desired Height"){
    msg_sub_pose = n.subscribe("mavros/local_position/pose", 1,  &State::positionCallback, this);
}

void State :: conditionSet(bool state){
    condition_takeoff.set(state);
    condition_takeoff.publish();
    return;
}


void State :: positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_altitude = msg->pose.position.z;
    n.getParam("drone/desired_height", desired_height);

    if(abs(current_altitude - desired_height) < 0.5){
        ROS_INFO("Takeoff success");
        conditionSet(true);
    }else{
        ROS_INFO("Takeoff failed");
        conditionSet(false);
    }
    return;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "state_takeoff");
    State state_takeoff;
    ros::spin();
    return 0;
}
