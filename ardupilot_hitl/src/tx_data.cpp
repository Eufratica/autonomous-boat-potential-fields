#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
//#include <gazebo_msgs/ModelStates.h>
//#include <tf2/LinearMath/Quaternion.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mavros_msgs/GazeboMavlink.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>


std::string model_name_to_track = "migbot1";  // Replace with your model's name

mavros_msgs::GazeboMavlink msg;    

/*void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& data) {
    // Find the index of your model in the ModelStates message
    int model_index = -1;
    for (size_t i = 0; i < data->name.size(); ++i) {
        if (data->name[i] == model_name_to_track) {
            model_index = static_cast<int>(i);
            break;
        }
    }

    if (model_index != -1) {
        // Extract the pose of your model
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "world";  // Adjust the frame_id as needed
        pose_msg.pose = data->pose[model_index];

        // Rotate the pose by π radians (180 degrees) in the X (roll) axis
        tf2::Quaternion rotation_quaternion;
        rotation_quaternion.setRPY(M_PI, 0, M_PI/2);  // Roll = π radians, Pitch = 0 radians, Yaw = 0 radians

        // Apply the rotation to the orientation quaternion of the pose
        tf2::Quaternion current_orientation;
        tf2::fromMsg(pose_msg.pose.orientation, current_orientation);
        current_orientation = rotation_quaternion * current_orientation;
        pose_msg.pose.orientation = tf2::toMsg(current_orientation);

        // Publish the pose to /mavros/fake_gps/mocap/pose
        static ros::Publisher pose_publisher =
            ros::NodeHandle().advertise<geometry_msgs::PoseStamped>("/mavros/fake_gps/mocap/pose", 10);
        pose_publisher.publish(pose_msg);
    }
}*/

void imuCallback(const sensor_msgs::Imu::ConstPtr& data) {

     msg.imuLinearAccelerationXYZ[0] = data->linear_acceleration.x;
     msg.imuLinearAccelerationXYZ[1] = data->linear_acceleration.y;
     msg.imuLinearAccelerationXYZ[2] = data->linear_acceleration.z;     
     msg.imuAngularVelocityRPY[0] = data->angular_velocity.x;
     msg.imuAngularVelocityRPY[1] = data->angular_velocity.y;
     msg.imuAngularVelocityRPY[2] = data->angular_velocity.z;      
}

void magCallback(const geometry_msgs::Vector3Stamped::ConstPtr& data) {

     msg.magneticFieldXYZ[0] = data->vector.x;
     msg.magneticFieldXYZ[1] = data->vector.y;
     msg.magneticFieldXYZ[2] = data->vector.z;     
     //ROS_ERROR("magneticFieldX %f", msg.magneticFieldXYZ[0]);
     //ROS_ERROR("magneticFieldY %f", msg.magneticFieldXYZ[1]);
     //ROS_ERROR("magneticFieldZ %f", msg.magneticFieldXYZ[2]);     
  
}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& data) {

     msg.latitude = data->latitude;
     msg.longitude = data->longitude;
     msg.altitude = data->altitude;     
     //ROS_ERROR("latitude %f", msg.latitude);
     //ROS_ERROR("longitude %f", msg.longitude);
     //ROS_ERROR("altitude %f", msg.altitude);    
  
}

void gpsvelCallback(const geometry_msgs::Vector3Stamped::ConstPtr& data) {

     msg.velocityENU[0] = data->vector.x;
     msg.velocityENU[1] = data->vector.y;
     msg.velocityENU[2] = data->vector.z;     
     //ROS_ERROR("velocityE %f", msg.velocityENU[0]);
     //ROS_ERROR("velocityN %f", msg.velocityENU[1]);
     //ROS_ERROR("velocityU %f", msg.velocityENU[2]);      
  
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "tx_data_node");
    ros::NodeHandle nh;
    //mavros_msgs::GazeboMavlink msg;    

    ros::Publisher gazebo_mavlink_message_pub = nh.advertise<mavros_msgs::GazeboMavlink>("/mavros/gazebo/sensors", 10);
    // Subscribe to Gazebo's ModelStates topic
    //ros::Subscriber model_states_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, modelStatesCallback);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu", 10, imuCallback); 
    ros::Subscriber mag_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("/magnetic", 10, magCallback); 
    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/fix", 10, gpsCallback);                  
    ros::Subscriber gpsvel_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("/fix_velocity", 10, gpsvelCallback);                                    

    ros::Rate rate(10); // 10 Hz, adjust the rate as needed
    
    while (ros::ok()) {
     // Use the ROS rate to control the rate of execution
     rate.sleep();
    
     //msg.sensor_1 = 1.2;
     //msg.sensor_2 = 3.4;

     gazebo_mavlink_message_pub.publish(msg);

     ros::spinOnce(); // Process any pending ROS callbacks
    }
    return 0;
}

