#include <iostream>
#include <math.h> 
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <behavior_tree/behavior_tree.h>
using namespace std; 

class State{
    private:
        ros::NodeHandle n;
        ros::Subscriber msg_sub_state;
        float landing_height = 1.0;
    public:
        bt::Condition condition;
        State();
        void getParam();
        void conditionSet(bool state);
        void stateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
};

State :: State() : condition("state_tag_land"){
    msg_sub_state = n.subscribe("tag_pose", 1,  &State::stateCallback, this);
}

void State :: conditionSet(bool state){
    condition.set(state);
    condition.publish();
    return;
}

void State :: getParam(){
    string node_ns = ros::this_node::getName();
    n.getParam("/" + node_ns + "/landing_height", landing_height);
    cout << "SETTING: Tag height -> " << landing_height << endl;
    return;
}



void State :: stateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    if(msg->pose.position.z < landing_height){
        conditionSet(true);
    }else{
        conditionSet(false);
    }
    return;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "state_tag_land");
    State state_tag_land;
    state_tag_land.getParam();
    ros::spin();
    return 0;
}
