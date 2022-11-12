#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "showline");
    ros::NodeHandle n;
    ros::Publisher markerPub = n.advertise<visualization_msgs::Marker>("TEXT_VIEW_FACING", 10);
    ros::Publisher markerArrayPub = n.advertise<visualization_msgs::MarkerArray>("TEXT_VIEW_ARRAY", 10);

    ros::Rate r(1);
    int k = 0;

    while(ros::ok())
    {

        visualization_msgs::MarkerArray costCubes;
        visualization_msgs::Marker costCube;
        bool once = true;

        for (int i = -10; i < 10; ++i) {

            if (once) {
                costCube.action = 3;
                once = false;
            } else {
                costCube.action = 0;
            }

            
            costCube.header.frame_id = "map";
            costCube.header.stamp = ros::Time::now();
            costCube.id = i;
            costCube.type = visualization_msgs::Marker::CUBE;
            costCube.scale.x = 1;
            costCube.scale.y = 1;
            costCube.scale.z = 1;
            costCube.color.a = 1.0; // 透明度
            costCube.color.r = 25;
            costCube.color.g = 255;
            costCube.color.b = 0;
            costCube.pose.position.x = i;
            costCube.pose.position.y = 0.2*(i*i - 10);
            costCubes.markers.push_back(costCube);
        }

        markerArrayPub.publish(costCubes);
        r.sleep();
    }
    return 0;
}


