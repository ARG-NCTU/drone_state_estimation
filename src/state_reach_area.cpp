#include <iostream>
#include "math.h" 
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <behavior_tree/behavior_tree.h>
using namespace std; 

class ReachArea{
    private:
        ros::NodeHandle n;
        ros::Subscriber msg_sub_state;
        ros::Subscriber msg_sub_pose;
        float desired_point[2] = {-16, 1};
        float margin = 1.0;
    public:
        bt::Condition condition_reach_area;
        ReachArea();
        void conditionSet(bool state);
        void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
};

ReachArea :: ReachArea() : condition_reach_area("At Landing Area"){
    msg_sub_pose = n.subscribe("mavros/local_position/pose", 1000,  &ReachArea::positionCallback, this);
}

void ReachArea :: conditionSet(bool state){
    condition_reach_area.set(state);
    condition_reach_area.publish();
    return;
}


void ReachArea :: positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    if(sqrt(pow(abs(desired_point[0] - msg->pose.position.x), 2) + 
                pow(abs(desired_point[1] - msg->pose.position.y), 2)) < margin){
        conditionSet(true);
    }else{
        conditionSet(false);
    }
    return;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "state_reach_area");
    ReachArea state_reach_area;
    ros::spin();
    return 0;
}
