#include <iostream>
#include <math.h> 
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <behavior_tree/behavior_tree.h>
using namespace std; 

class State{
    private:
        ros::NodeHandle n;
        ros::Subscriber msg_sub_state;
        float warning_voltage = 50.0;
        float current_voltage = 0;
    public:
        bt::Condition condition;
        State();
        void getParam();
        void setState();
        void conditionSet(bool state);
        void stateCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
};

State :: State() : condition("state_power"){
    msg_sub_state = n.subscribe("mavros/battery", 1,  &State::stateCallback, this);
}

void State :: conditionSet(bool state){
    condition.set(state);
    condition.publish();
    return;
}

void State :: getParam(){
    string node_ns = ros::this_node::getName();
    n.getParam("/" + node_ns + "/warning_voltage", warning_voltage);
    cout << "SETTING: Voltage warning -> " << warning_voltage << endl;
    return;
}

void State :: setState(){
    
    if(current_voltage < warning_voltage){
        conditionSet(true);
        cout << "WARN WARN WARN" << endl;
    }else{
        conditionSet(false);
    }
    return;
}


void State :: stateCallback(const sensor_msgs::BatteryState::ConstPtr& msg){
    current_voltage = msg->voltage;
    cout << "Voltage: " << current_voltage << endl;
    return;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "state_power");
    State state_power;
    state_power.getParam();
    while(ros::ok()){
        state_power.setState();
        ros::spinOnce();
    }
    ros::spin();
    return 0;
}
