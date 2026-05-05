// WorldGpsAlignMavlink.cc
//
// "World-bender" alinhado com o ArduPilot:
//  - NÃO mexe na pose do robô.
//  - Gira/translada o(s) modelo(s) de cenário para que a geometria
//    robô <-> marcadores em Gazebo fique igual à do mapa/QGC.
//
// Usa:
//   * robot_model        : nome do modelo do robô no Gazebo (ex: migbot1)
//   * environment_model  : modelo principal onde estão os ref_circle_*
//   * extra_model        : (opcional, repetível) outros modelos de cenário
//                           ex: ocean_waves, ground_plane, etc.
//   * gps_topic          : NavSatFix do ArduPilot (via MAVROS)
//
// Passos:
//   1) Espera um NavSatFix do ArduPilot.
//   2) Usa (lat_r, lon_r) do robô como origem ENU (robô = (0,0)).
//   3) Converte lat/lon dos ref_circle para ENU relativos ao robô.
//   4) Define posições alvo dos ref_circle em Gazebo:
//        P_target_i = P_robot_gz + Q_i.
//   5) Ajusta R,t (rotação + translação) que leva P_source_i -> P_target_i.
//   6) Aplica R,t na pose de environment_model + extra_model(s) com SetWorldPose.
//   7) Robô fica parado; cenário “encaixa” no ground truth do QGC.
//
// Depende de ROS (para assinar o NavSatFix do SITL).

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

#include <Eigen/Core>
#include <Eigen/SVD>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <functional>

namespace gazebo
{

class WorldGpsAlignMavlink : public WorldPlugin
{
public:
  WorldGpsAlignMavlink() : WorldPlugin() {}
  virtual ~WorldGpsAlignMavlink() {}

  struct RefPoint
  {
    std::string linkName;
    double latDeg;
    double lonDeg;
  };

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
  {
    this->world_ = _world;

    if (!_sdf)
    {
      gzerr << "[WorldGpsAlignMavlink] SDF pointer is null.\n";
      return;
    }

    // ===== parâmetros obrigatórios =====
    if (!_sdf->HasElement("environment_model"))
    {
      gzerr << "[WorldGpsAlignMavlink] Missing <environment_model>.\n";
      return;
    }
    this->envModelName_ = _sdf->Get<std::string>("environment_model");

    if (!_sdf->HasElement("robot_model"))
    {
      gzerr << "[WorldGpsAlignMavlink] Missing <robot_model>.\n";
      return;
    }
    this->robotModelName_ = _sdf->Get<std::string>("robot_model");

    // modelos extras de cenário (ex: ocean_waves, ground_plane...)
    if (_sdf->HasElement("extra_model"))
    {
      sdf::ElementPtr em = _sdf->GetElement("extra_model");
      while (em)
      {
        std::string name = em->Get<std::string>();
        extraModelNames_.push_back(name);
        if (em->GetNextElement("extra_model"))
          em = em->GetNextElement("extra_model");
        else
          break;
      }
    }

    // tópico de GPS (NavSatFix) vindo do ArduPilot (via MAVROS normalmente)
    this->gpsTopic_ = _sdf->Get<std::string>("gps_topic",
                                             std::string("/mavros/global_position/global")).first;

    // raio da Terra para ENU plano
    this->earthRadius_ = _sdf->Get<double>("earth_radius", 6378137.0).first;

    // ler referências
    if (!_sdf->HasElement("ref"))
    {
      gzerr << "[WorldGpsAlignMavlink] No <ref> blocks found. Need at least 2.\n";
      return;
    }

    sdf::ElementPtr refElem = _sdf->GetElement("ref");
    while (refElem)
    {
      if (!refElem->HasElement("link_name") ||
          !refElem->HasElement("lat")       ||
          !refElem->HasElement("lon"))
      {
        gzerr << "[WorldGpsAlignMavlink] <ref> precisa de <link_name>, <lat>, <lon>.\n";
      }
      else
      {
        RefPoint rp;
        rp.linkName = refElem->Get<std::string>("link_name");
        rp.latDeg   = refElem->Get<double>("lat");
        rp.lonDeg   = refElem->Get<double>("lon");
        this->refs_.push_back(rp);
      }

      if (refElem->GetNextElement("ref"))
        refElem = refElem->GetNextElement("ref");
      else
        break;
    }

    if (this->refs_.size() < 2)
    {
      gzerr << "[WorldGpsAlignMavlink] Precisa de pelo menos 2 referências; "
            << "encontradas " << this->refs_.size() << ".\n";
      return;
    }

    gzmsg << "[WorldGpsAlignMavlink] Loaded. envModel=" << this->envModelName_
          << " robotModel=" << this->robotModelName_
          << " refs=" << this->refs_.size()
          << " gps_topic=" << this->gpsTopic_ << "\n";

    if (!extraModelNames_.empty())
    {
      gzmsg << "[WorldGpsAlignMavlink] Extra models:";
      for (auto &n : extraModelNames_)
        gzmsg << " " << n;
      gzmsg << "\n";
    }

    // ===== ROS =====
    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = nullptr;
      ros::init(argc, argv, "world_gps_align_mavlink_worldbender",
                ros::init_options::NoSigintHandler);
    }

    if (!this->nh_)
      this->nh_.reset(new ros::NodeHandle("~"));

    this->gpsSub_ = this->nh_->subscribe(
        this->gpsTopic_, 10,
        &WorldGpsAlignMavlink::GpsCallback, this);

    // ===== conectar update =====
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&WorldGpsAlignMavlink::OnUpdate, this, std::placeholders::_1));
  }

private:
  // callback do GPS do ArduPilot
  void GpsCallback(const sensor_msgs::NavSatFixConstPtr &msg)
  {
    this->lastFix_ = *msg;
    this->haveGps_ = true;
  }

  void OnUpdate(const common::UpdateInfo &)
  {
    if (this->aligned_)
      return;               // já alinhou uma vez

    if (!this->haveGps_)
      return;               // ainda não recebeu GPS

    if (!this->world_)
      return;

    // pegar modelos (lazy, porque podem nascer depois)
    if (!this->envModel_)
    {
      this->envModel_ = this->world_->ModelByName(this->envModelName_);
      if (!this->envModel_)
      {
        static bool warnedEnv = false;
        if (!warnedEnv)
        {
          gzmsg << "[WorldGpsAlignMavlink] Aguardando modelo de ambiente '"
                << this->envModelName_ << "'.\n";
          warnedEnv = true;
        }
        return;
      }
    }

    if (!this->robotModel_)
    {
      this->robotModel_ = this->world_->ModelByName(this->robotModelName_);
      if (!this->robotModel_)
      {
        static bool warnedRobot = false;
        if (!warnedRobot)
        {
          gzmsg << "[WorldGpsAlignMavlink] Aguardando modelo do robô '"
                << this->robotModelName_ << "'.\n";
          warnedRobot = true;
        }
        return;
      }
    }

    // resolver extra models
    if (extraModels_.size() != extraModelNames_.size())
    {
      extraModels_.clear();
      for (const auto &name : extraModelNames_)
      {
        auto m = this->world_->ModelByName(name);
        if (!m)
        {
          gzmsg << "[WorldGpsAlignMavlink] Aviso: extra_model '" << name
                << "' ainda não encontrado.\n";
        }
        extraModels_.push_back(m); // pode ter nullptr, tratamos depois
      }
    }

    // tudo pronto → faz alinhamento uma vez
    if (this->AlignEnvironment())
    {
      this->aligned_ = true;
      gzmsg << "[WorldGpsAlignMavlink] Alinhamento do mundo concluído.\n";
    }
  }

  bool AlignEnvironment()
  {
    // posição do robô em Gazebo (NÃO será modificada)
    ignition::math::Pose3d poseR = this->robotModel_->WorldPose();
    ignition::math::Vector3d posR = poseR.Pos();
    double xr = posR.X();
    double yr = posR.Y();

    double lat_r = this->lastFix_.latitude;
    double lon_r = this->lastFix_.longitude;

    gzmsg << "[WorldGpsAlignMavlink] Robot Gazebo=(" << xr << ", " << yr
          << "), GPS=(" << lat_r << ", " << lon_r << ")\n";

    auto latLonToEnuRelRobot = [this, lat_r, lon_r](double latDeg, double lonDeg)
    {
      double lat0 = lat_r * M_PI / 180.0;
      double lon0 = lon_r * M_PI / 180.0;
      double lat  = latDeg * M_PI / 180.0;
      double lon  = lonDeg * M_PI / 180.0;

      double dLat = lat - lat0;
      double dLon = lon - lon0;

      double x_enu = this->earthRadius_ * std::cos(lat0) * dLon; // East
      double y_enu = this->earthRadius_ * dLat;                  // North

      return Eigen::Vector2d(x_enu, y_enu);
    };

    std::vector<Eigen::Vector2d> P;       // posições atuais dos marcadores (Gazebo)
    std::vector<Eigen::Vector2d> Ptarget; // onde deveriam estar (Gazebo), rel. ao robô

    P.reserve(this->refs_.size());
    Ptarget.reserve(this->refs_.size());

    for (const auto &rp : this->refs_)
    {
      auto link = this->envModel_->GetLink(rp.linkName);
      if (!link)
      {
        gzerr << "[WorldGpsAlignMavlink] Link '" << rp.linkName
              << "' não encontrado em '" << this->envModelName_ << "'.\n";
        continue;
      }

      ignition::math::Pose3d poseL = link->WorldPose();
      ignition::math::Vector3d posL = poseL.Pos();
      double xl = posL.X();
      double yl = posL.Y();

      Eigen::Vector2d q_rel = latLonToEnuRelRobot(rp.latDeg, rp.lonDeg);
      Eigen::Vector2d p_target(xr + q_rel.x(), yr + q_rel.y());

      P.emplace_back(xl, yl);
      Ptarget.emplace_back(p_target);

      gzmsg << "[WorldGpsAlignMavlink] Ref '" << rp.linkName << "': "
            << "P_source=(" << xl << ", " << yl << "), "
            << "P_target=(" << p_target.x() << ", " << p_target.y()
            << ") (relativo ao robô)\n";
    }

    if (P.size() < 2)
    {
      gzerr << "[WorldGpsAlignMavlink] Menos de 2 refs válidas para alinhar ("
            << P.size() << ").\n";
      return false;
    }

    // ===== Procrustes 2D SEM escala: Ptarget ≈ R * P + t =====
    const size_t M = P.size();
    Eigen::Vector2d centroidP(0.0, 0.0);
    Eigen::Vector2d centroidQ(0.0, 0.0);

    for (size_t i = 0; i < M; ++i)
    {
      centroidP += P[i];
      centroidQ += Ptarget[i];
    }
    centroidP /= static_cast<double>(M);
    centroidQ /= static_cast<double>(M);

    Eigen::Matrix2d Sigma = Eigen::Matrix2d::Zero();
    for (size_t i = 0; i < M; ++i)
    {
      Eigen::Vector2d p = P[i] - centroidP;
      Eigen::Vector2d q = Ptarget[i] - centroidQ;
      Sigma += p * q.transpose(); // mapeando P -> Ptarget
    }

    Eigen::JacobiSVD<Eigen::Matrix2d> svd(
        Sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d U = svd.matrixU();
    Eigen::Matrix2d V = svd.matrixV();

    Eigen::Matrix2d R = V * U.transpose();

    // garantir rotação (det=+1)
    double detR = R(0,0) * R(1,1) - R(0,1) * R(1,0);
    if (detR < 0.0)
    {
      V.col(1) *= -1.0;
      R = V * U.transpose();
      detR = R(0,0) * R(1,1) - R(0,1) * R(1,0);
    }

    Eigen::Vector2d t = centroidQ - R * centroidP;

    gzmsg << "[WorldGpsAlignMavlink] Transformação mundo (P -> Ptarget):\n";
    gzmsg << "  R =\n" << R << "\n";
    gzmsg << "  det(R) = " << detR << "\n";
    gzmsg << "  t = (" << t.x() << ", " << t.y() << ")\n";

    for (size_t i = 0; i < M; ++i)
    {
      Eigen::Vector2d pred = R * P[i] + t;
      Eigen::Vector2d err  = pred - Ptarget[i];
      gzmsg << "[WorldGpsAlignMavlink] Erro ref " << i
            << " depois da transformação (mundo) = ("
            << err.x() << ", " << err.y() << ") m\n";
    }

    // yaw da rotação
    double deltaYaw = std::atan2(R(1,0), R(0,0));

    // aplicar em todos os modelos de cenário (principal + extras)
    ApplyTransformToModel(this->envModel_, R, t, deltaYaw, "envModel");

    for (size_t i = 0; i < extraModels_.size(); ++i)
    {
      auto &m = extraModels_[i];
      const std::string &name = (i < extraModelNames_.size()
                                   ? extraModelNames_[i] : std::string("extra?"));
      if (!m)
      {
        gzmsg << "[WorldGpsAlignMavlink] extra_model '" << name
              << "' ainda é nullptr; ignorando.\n";
        continue;
      }
      ApplyTransformToModel(m, R, t, deltaYaw, name);
    }

    // robô NÃO é transformado

    return true;
  }

  void ApplyTransformToModel(physics::ModelPtr model,
                             const Eigen::Matrix2d &R,
                             const Eigen::Vector2d &t,
                             double deltaYaw,
                             const std::string &label)
  {
    if (!model)
      return;

    ignition::math::Pose3d oldPose = model->WorldPose();
    ignition::math::Vector3d oldPos = oldPose.Pos();
    ignition::math::Quaterniond oldRot = oldPose.Rot();

    Eigen::Vector2d posOldXY(oldPos.X(), oldPos.Y());
    Eigen::Vector2d posNewXY = R * posOldXY + t;

    ignition::math::Vector3d newPos(posNewXY.x(), posNewXY.y(), oldPos.Z());
    ignition::math::Quaterniond rotDelta(0.0, 0.0, deltaYaw);
    ignition::math::Quaterniond newRot = rotDelta * oldRot;

    ignition::math::Pose3d newPose(newPos, newRot);
    model->SetWorldPose(newPose);

    gzmsg << "[WorldGpsAlignMavlink] Model '" << model->GetName()
          << "' (" << label << ") movido:\n";
    gzmsg << "  oldPos = (" << oldPos.X() << ", " << oldPos.Y()
          << ", " << oldPos.Z() << ")\n";
    gzmsg << "  newPos = (" << newPos.X() << ", " << newPos.Y()
          << ", " << newPos.Z() << ")\n";
    gzmsg << "  deltaYaw = " << deltaYaw << " rad\n";
  }

private:
  physics::WorldPtr world_;
  physics::ModelPtr envModel_;
  physics::ModelPtr robotModel_;
  std::vector<physics::ModelPtr> extraModels_;

  event::ConnectionPtr updateConnection_;

  std::string envModelName_;
  std::string robotModelName_;
  std::vector<std::string> extraModelNames_;
  std::string gpsTopic_;

  double earthRadius_{6378137.0};

  std::vector<RefPoint> refs_;

  // ROS
  static std::unique_ptr<ros::NodeHandle> nh_;
  ros::Subscriber gpsSub_;
  sensor_msgs::NavSatFix lastFix_;
  bool haveGps_{false};

  bool aligned_{false};
};

// definição do static
std::unique_ptr<ros::NodeHandle> WorldGpsAlignMavlink::nh_;

GZ_REGISTER_WORLD_PLUGIN(WorldGpsAlignMavlink)

} // namespace gazebo

