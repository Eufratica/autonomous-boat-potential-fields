#ifndef _PING_PLUGIN_HH_
#define _PING_PLUGIN_HH_

#include <thread>
#include <cmath>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <ping360_sonar/SonarEcho.h>
#include <sensor_msgs/LaserScan.h>

namespace gazebo
{
  /// \brief A plugin to control the sensor.
  class PingPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: PingPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Safety check
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
        return;
      }

      // Store the model pointer for convenience.
      this->model = _model;

      // Get the first joint. We are making an assumption about the model
      // having one joint that is the rotational joint.
      for (auto joint: _model->GetJoints())
        if (joint->GetName() == "inner_disk_joint")
          this->joint = joint;

      // Setup a P-controller, with a gain of 0.1.
      this->pid = common::PID(0.1, 0, 0);

      // Apply the P-controller to the joint.
      this->model->GetJointController()->SetVelocityPID(
          this->joint->GetScopedName(), this->pid);

      // Check that the velocity element exists, then read the value
      double velocity = 0;
      if (_sdf->HasElement("velocity"))
        velocity = _sdf->Get<double>("velocity");

      // Set the joint's target velocity. This target velocity is just
      // for demonstration purposes.
      this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), velocity);

      // Start ROS related
      this->rosNode.reset(new ros::NodeHandle(""));
      this->sonarMsgPub = this->rosNode->advertise<ping360_sonar::SonarEcho>("ping360_node/sonar/data", 10);
      this->sonarMsgInfoPub = this->rosNode->advertise<sensor_msgs::LaserScan>("ping360_node/sonar/scan", 10);

      // Cast the sensor to a RaySensor and store it in a member variable
      sensors::SensorManager* sensorManager = sensors::SensorManager::Instance();
      this->sensor = sensorManager->GetSensor("sonar_sensor");
      this->raySensor = std::dynamic_pointer_cast<sensors::RaySensor>(this->sensor);
      this->updateConnection = raySensor->ConnectUpdated(std::bind(&PingPlugin::OnUpdate, this));
    }

    public: void OnUpdate()
    {
      auto rangeSensor = std::dynamic_pointer_cast<sensors::RaySensor>(this->sensor);

      // Get the latest range measurements from the sensor
      std::vector<double> ranges;
      rangeSensor->Ranges(ranges);
      // Convert to float type for filling in the message
      std::vector<float> ranges_float(ranges.size());
      for (size_t i = 0; i < ranges.size(); i++)
        ranges_float[i] = std::isinf(ranges[i]) ? 2 : static_cast<float>(ranges[i]);
      double range = 100 * *std::min_element(ranges_float.begin(), ranges_float.end()); // [cm]

      // Get the joint angle in degrees
      float angle = this->joint->Position(0) * 180/M_PI;
      int angle_ranged = angle - static_cast<int>(angle/360)*360;

      // Calculate intensity based on range
      std::vector<float> intensities(ranges_float.size());
      for (size_t i = 0; i < ranges_float.size(); i++)
      {
        if (std::isinf(ranges_float[i]))
        {
          intensities[i] = 0.0;
        }
        else
        {
          // Example calculation for intensity (you can adjust this based on your requirements)
          intensities[i] = 255.0 * (1.0 - ranges_float[i]);
        }
      }

      // Convertendo os valores de intensidade para unsigned char
      std::vector<unsigned char> intensities_char;
      intensities_char.reserve(intensities.size());
      for (size_t i = 0; i < intensities.size(); ++i) {
          intensities_char.push_back(static_cast<unsigned char>(intensities[i]));
      }


      if (abs(angle_ranged % 5) <= 0)
      {
        // Publish to ROS
        ping360_sonar::SonarEcho msg;
        msg.header.frame_id = "sonar_frame";
        msg.header.stamp = ros::Time::now();
        msg.number_of_samples = ranges_float.size();
        msg.speed_of_sound = 1499;
        msg.transmit_frequency = 740;
        msg.angle = static_cast<float>(angle_ranged);
        msg.gain = 1; 
        msg.range = 20; 
        msg.intensities = intensities_char;
        this->sonarMsgPub.publish(msg);
        
        sensor_msgs::LaserScan msg_info;
        msg_info.header = msg.header;
        msg_info.angle_max = 2*M_PI;
        msg_info.angle_increment = 0.09;
        msg_info.range_min = 0.75;
        msg_info.range_max = 20;
        msg_info.ranges = ranges_float;
        msg_info.intensities.resize(ranges_float.size(), 0);
        this->sonarMsgInfoPub.publish(msg_info);
      }
    }

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;

    /// \brief A PID controller for the joint.
    private: common::PID pid;

    /// \brief range sonar to get the measures
    private: sensors::SensorPtr sensor;
    private: sensors::RaySensorPtr raySensor;

    /// \brief connection bind for each update from the world
    private: event::ConnectionPtr updateConnection;

    /// \brief ROS node handle
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief ROS sonar message publisher
    private: ros::Publisher sonarMsgPub;
    private: ros::Publisher sonarMsgInfoPub;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(PingPlugin)
}
#endif

