#include <physics/physics.hh>
#include "ros/ros.h"








int main(int argc, char const *argv[])
{
    ros::Rate loop_rate(20);
    while(1){
        connected = ros::master::check();
        printf("test %d",ros::master::check());
        send_wp.publish(waypoint);
        std_msgs::Bool msg;
        msg.data = bEtat;
        etat.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
