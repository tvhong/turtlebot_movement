#include "ros/ros.h"
#include "sensor_msgs/Image.h"

#include <sstream>

ros::Publisher echoer;
void callBack(const sensor_msgs::Image::ConstPtr& msg) {
    ROS_INFO("Received a frame");
    echoer.publish(msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "color_reg");
    
    ros::NodeHandle n;

    echoer = n.advertise<sensor_msgs::Image>("img_echoer", 1000);

    ros::Subscriber sub = n.subscribe("/camera/rgb/image_color", 1000, callBack);

    ros::Rate loop_rate(10);

    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
