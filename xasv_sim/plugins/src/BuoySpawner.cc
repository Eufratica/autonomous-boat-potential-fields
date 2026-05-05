#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <ignition/math/Pose3.hh>

#include <sstream>
#include <string>
#include <cmath>

namespace gazebo
{

class BuoySpawner : public WorldPlugin
{
public:
  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
  {
    this->world = _world;

    // Parâmetros SDF
    if (_sdf->HasElement("robot_name"))
      this->robotName = _sdf->Get<std::string>("robot_name");

    if (_sdf->HasElement("model_name"))
      this->modelName = _sdf->Get<std::string>("model_name");

    if (_sdf->HasElement("spawn_period"))
      this->spawnPeriod = _sdf->Get<double>("spawn_period");

    if (_sdf->HasElement("spawn_distance"))
      this->spawnDistance = _sdf->Get<double>("spawn_distance");

    gzdbg << "[BuoySpawner] world=" << this->world->Name()
          << " robot_name=" << this->robotName
          << " model_name=" << this->modelName
          << " period=" << this->spawnPeriod
          << " dist=" << this->spawnDistance << "\n";

    // Node + publisher para ~/factory
    this->node.reset(new transport::Node());
    this->node->Init(this->world->Name());
    this->factoryPub =
        this->node->Advertise<msgs::Factory>("~/factory");

    this->lastSpawnTime = this->world->SimTime().Double();

    // Conectar no update
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&BuoySpawner::OnUpdate, this, std::placeholders::_1));
  }

private:
  void OnUpdate(const common::UpdateInfo &_info)
  {
    const double now = _info.simTime.Double();

    // Procurar o robô se ainda não pegamos
    if (!this->robotModel)
    {
      this->robotModel = this->world->ModelByName(this->robotName);
      if (!this->robotModel)
        return;  // ainda não carregou
    }

    // Checar período de spawn
    if ((now - this->lastSpawnTime) < this->spawnPeriod)
      return;

    this->lastSpawnTime = now;

    // Pose do robô
    ignition::math::Pose3d pose = this->robotModel->WorldPose();
    double yaw = pose.Rot().Yaw();

    // Ponto à frente do robô
    double x = pose.Pos().X() + this->spawnDistance * std::cos(yaw);
    double y = pose.Pos().Y() + this->spawnDistance * std::sin(yaw);
    double z = pose.Pos().Z(); // mesmo nível do robô (ajuste se quiser)

    std::string instanceName =
        this->modelName + "_" + std::to_string(this->counter++);

    // SDF para o modelo a partir de model://trunk2_buoy
    std::ostringstream ss;
    ss << "<sdf version='1.6'>"
       << "  <model name='" << instanceName << "'>"
       << "    <include>"
       << "      <uri>model://" << this->modelName << "</uri>"
       << "    </include>"
       << "    <pose>"
       << x << " " << y << " " << z
       << " 0 0 " << yaw
       << "    </pose>"
       << "  </model>"
       << "</sdf>";

    msgs::Factory msg;
    msg.set_sdf(ss.str());
    this->factoryPub->Publish(msg);

    gzdbg << "[BuoySpawner] Spawned " << instanceName
          << " at (" << x << ", " << y << ", " << z << ")\n";
  }

private:
  physics::WorldPtr world;
  physics::ModelPtr robotModel;

  std::string robotName  = "migbot1";
  std::string modelName  = "trunk2_buoy";
  double spawnPeriod     = 5.0;  // segundos
  double spawnDistance   = 8.0;  // metros à frente

  double lastSpawnTime   = 0.0;
  unsigned int counter   = 0;

  transport::NodePtr node;
  transport::PublisherPtr factoryPub;
  event::ConnectionPtr updateConnection;
};

GZ_REGISTER_WORLD_PLUGIN(BuoySpawner)

} // namespace gazebo

