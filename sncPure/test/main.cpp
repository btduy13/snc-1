#include "fetch_purepursuit/fetch_purepursuit.h"

#include <iostream>
#include <vector>
#include <cmath>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fetch_purepursuit");

    ros::NodeHandle nh;
    // ros::AsyncSpinner spinner(3);

    // spinner.start();
    ros::Rate rate(10);

    FetchPurePursuit fetch_purepursuit(nh);

    while (ros::ok)
    {
        fetch_purepursuit.operate();
        ros::spinOnce();
        rate.sleep();
    }

    // spinner.stop();
    return 0;
}