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

class NavCondition{
    private:
        ros::NodeHandle n;
        // ros::Subscriber sub_example;
        // ros::Publisher pub_example;
        float **waypoint_list;
        int waypoint_num, waypoint_dim;
        XmlRpc::XmlRpcValue xml_waypoint;
    public:
        bt::Condition condition;
        NavCondition();
        void getParam();
        void conditionSet(bool state);
        void stateCallback(const std_msgs::Bool::ConstPtr& msg);
};

NavCondition :: NavCondition() : condition(appendString(ros::this_node::getName(), (string)"")){
    // sub_example = n.subscribe("topic", 1,  &NavCondition::callback, this);
}

void NavCondition :: getParam(){
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

void NavCondition :: conditionSet(bool state){
    condition.set(state);
    condition.publish();
    return;
}

void NavCondition :: stateCallback(const std_msgs::Bool::ConstPtr& msg){
    if(msg->data == true){
        conditionSet(true);
    }else{
        conditionSet(false);
    }
    return;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "state_nav");
    NavCondition state_nav;
    state_nav.getParam();
    ros::spin();
    return 0;
}