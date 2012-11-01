
#include "ros/ros.h"

#include <string>

#include <handle_msgs/CableTension.h>
#include <handle_msgs/Collision.h>
#include <handle_msgs/Finger.h>
#include <handle_msgs/HandleCollisions.h>
#include <handle_msgs/HandleControl.h>
#include <handle_msgs/HandleSensorsCalibrated.h>
#include <handle_msgs/HandleSensors.h>

#include <sensor_msgs/JointState.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

ros::Publisher pub;

float min_val;
float max_val;

// float get_r(float val)
// {
//     return val / 
// };

int get_global_id(int local_id, std::string link_name)
{
    if (link_name == "base_link")
        return local_id + 0;
    if (link_name == "finger[0]/proximal_link")
        return local_id + 100;
    if (link_name == "finger[1]/proximal_link")
        return local_id + 200;
    if (link_name == "finger[2]/proximal_link")
        return local_id + 300;
    if (link_name == "finger[0]/distal_link")
        return local_id + 400;
    if (link_name == "finger[1]/distal_link")
        return local_id + 500;
    if (link_name == "finger[2]/distal_link")
        return local_id + 600;
    else 
        return local_id + 1000;
}

void set_max_min(const handle_msgs::HandleCollisionsConstPtr& data)
{
    for (unsigned int i=0; i<data->collisions.size(); i++)
    {
        if (data->collisions[i].intensity > max_val)
            max_val = data->collisions[i].intensity;
        if (data->collisions[i].intensity < min_val)
            min_val = data->collisions[i].intensity;
    }
};

void callback(const handle_msgs::HandleCollisionsConstPtr& data)
{
    set_max_min(data);
    
    visualization_msgs::MarkerArray marker_array;
    
    for (unsigned int i=0; i<data->collisions.size(); i++)
    {
        visualization_msgs::Marker marker;
        
        if (data->collisions[i].intensity > 10)
        {
            // red
            marker.color.r = 1.0;
            marker.color.g = 1.0 - data->collisions[i].intensity / max_val;
            marker.color.b = 1.0 - data->collisions[i].intensity / max_val;
            marker.color.a = 1.0;
        }
        else if (data->collisions[i].intensity < 10)
        {
            // green
            marker.color.r = 1.0 - data->collisions[i].intensity / min_val;
            marker.color.g = 1.0 - data->collisions[i].intensity / min_val;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
        }
        else
            continue;
        
        marker.ns = "tactile_display";
        marker.id = get_global_id(data->collisions[i].sensor_id, data->collisions[i].frame_id);
		marker.header.frame_id = '/' + data->collisions[i].frame_id;
		marker.type = visualization_msgs::Marker::CUBE; //SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0.1);
		marker.scale.x = 0.0051; //data->collisions[i].intensity / 10000.0; //0.0051;
		marker.scale.y = 0.0051; //data->collisions[i].intensity / 10000.0; //0.0051;
		marker.scale.z = 0.0051; //data->collisions[i].intensity / 10000.0; //0.0051;
        
		marker.pose.position.x = data->collisions[i].x;
		marker.pose.position.y = data->collisions[i].y;
		marker.pose.position.z = data->collisions[i].z;
		marker.pose.orientation.x = 0.0;        
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker_array.markers.push_back(marker);
    }
    
    pub.publish(marker_array);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "collisions_visualization_publisher");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/handle/events/collisions", 1, callback);
  pub = n.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);
  
  ros::spin();
  
  return 0;
}
