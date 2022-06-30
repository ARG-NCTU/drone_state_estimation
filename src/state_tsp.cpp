#include <iostream>
#include "math.h" 
#include "ros/ros.h"
#include "mavros_msgs/State.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"
#include <behavior_tree/behavior_tree.h>
using namespace std; 

class State{
    private:
        ros::NodeHandle n;
        ros::Subscriber msg_sub_state;
    public:
        bt::Condition condition_tsp;
        State();
        void conditionSet(bool state);
        void stateCallback(const std_msgs::Bool::ConstPtr& msg);
};

State :: State() : condition_tsp("TSP Finish"){
    msg_sub_state = n.subscribe("tsp_finished", 1,  &State::stateCallback, this);
}

void State :: conditionSet(bool state){
    condition_tsp.set(state);
    condition_tsp.publish();
    return;
}


void State :: stateCallback(const std_msgs::Bool::ConstPtr& msg){
    if(msg->data){
        conditionSet(true);
    }else{
        conditionSet(false);
    }
    return;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "state_tsp");
    State state_tsp;
    ros::spin();
    return 0;
}
