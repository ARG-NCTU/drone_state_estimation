#include <iostream>
#include <string> 
#include <ros/ros.h>
#include <behavior_tree/behavior_tree.h>
using namespace std; 

string appendString(const string &s_body, const string &s_suffix){
    std::string origin = s_body;
    std::string later = s_suffix;
    origin.append(later);
    return origin;
}
class NavAction{
    private:
        ros::NodeHandle n;
        // ros::Subscriber sub_example;
        // ros::Publisher pub_example;

        float **waypoint_list;
        int waypoint_num, waypoint_dim;
        XmlRpc::XmlRpcValue xml_waypoint;
    public:
        bt::Action action;
        NavAction();
        void getParam();
        void actionSet(int state);
};

NavAction :: NavAction() : action(appendString(ros::this_node::getName(), (string)"")){
    
}

void NavAction :: getParam(){
    string node_ns = ros::this_node::getName();
    n.getParam("/" + node_ns + "/waypoint", xml_waypoint);
    waypoint_num = xml_waypoint.size();
    waypoint_dim = xml_waypoint[0].size();
    waypoint_list = new float *[waypoint_num];
    for (int i = 0; i < waypoint_num; i++){ waypoint_list[i] = new float[waypoint_dim]; }
    for (int i = 0; i < waypoint_num; i++){
        for (int j = 0; j < waypoint_dim; j++){
            try{
                std::ostringstream ostr;
                ostr << xml_waypoint[i][j];
                std::istringstream istr(ostr.str());
                istr >> waypoint_list[i][j];
            }
            catch(...){
                throw;
            }
        }
    }
    // debug
    for (int i = 0; i < waypoint_num; i++){
        for (int j = 0; j < waypoint_dim; j++){
            cout << waypoint_list[i][j] << " ";
        }
        cout << endl;
    }
    return;
}

void NavAction :: actionSet(int state){
    switch(state){
        case 1:
            action.set_success();
            break;
        case 0:
            action.set_running();
            break;
        case -1:
            action.set_failure();
            break;
    }
    action.publish();
    return;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "action_nav");
    NavAction action_nav;
    action_nav.getParam();

    while(ros::ok()){
        if(action_nav.action.is_active() && action_nav.action.active_has_changed()){
            // ROS_INFO("Action: Arm activiate");
        }
        if(action_nav.action.active_has_changed() && !(action_nav.action.is_active())){
            // ROS_INFO("Action: Done");
            action_nav.actionSet(1);
        }
        if(action_nav.action.is_active()){
            action_nav.actionSet(0);
        }
        ros::spinOnce();
    }
    return 0;
}