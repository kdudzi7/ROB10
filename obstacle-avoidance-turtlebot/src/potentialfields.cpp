
    #include "sensor_msgs/LaserScan.h"
    #include "std_msgs/String.h"
    #include "iomanip"

    void poseMessageReceived(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        ROS_INFO("position=: [%f]",scan->angle_min);
    }

    int main(int argc,char **argv)
    {
        ros::init(argc,argv,"subscribe_to_scan");
        ros::NodeHandle n;

        //Create a subscriber object
        ros::Subscriber sub = n.subscribe("/Scan",1000, poseMessageReceived);

        //Let ROS take over
        ros::spin();

        return 0;
    }