#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

#include <string>
#include <random>
#include <functional>
#include <cmath>
#include <algorithm>

namespace gazebo
{
class TrunkScaler : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    this->model = _model;
    this->world = this->model->GetWorld();

    // ===== LINK OBRIGATÓRIO =====
    if (_sdf->HasElement("link_name"))
      this->linkName = _sdf->Get<std::string>("link_name");
    else
    {
      gzerr << "[TrunkScaler] link_name não especificado no SDF.\n";
      return;
    }

    this->link = this->model->GetLink(this->linkName);
    if (!this->link)
    {
      gzerr << "[TrunkScaler] Link '" << this->linkName
            << "' não encontrado no modelo '" << this->model->GetName() << "'.\n";
      return;
    }

    // ===== VISUAL NAME A PARTIR DA SDF =====
    sdf::ElementPtr modelSdf = this->model->GetSDF();
    if (modelSdf)
    {
      sdf::ElementPtr linkElem = modelSdf->GetElement("link");
      while (linkElem)
      {
        std::string name = linkElem->Get<std::string>("name");
        if (name == this->linkName)
        {
          if (linkElem->HasElement("visual"))
          {
            sdf::ElementPtr visElem = linkElem->GetElement("visual");
            if (visElem->HasAttribute("name"))
              this->visualName = visElem->Get<std::string>("name");
          }
          break;
        }
        linkElem = linkElem->GetNextElement("link");
      }
    }

    if (_sdf->HasElement("visual_name"))
      this->visualName = _sdf->Get<std::string>("visual_name");

    if (this->visualName.empty())
    {
      this->visualName = "visual";  // fallback
      gzmsg << "[TrunkScaler] visual name não encontrado na SDF de '"
            << this->linkName << "'. Tentando fallback 'visual'.\n";
    }
    else
    {
      gzmsg << "[TrunkScaler] visual name para link '" << this->linkName
            << "' = '" << this->visualName << "'\n";
    }

    // ===== PARÂMETROS DE PERFIL =====
    if (_sdf->HasElement("profile"))
      this->profile = _sdf->Get<std::string>("profile");

    if (_sdf->HasElement("scale_amplitude"))
      this->scaleAmplitude = _sdf->Get<double>("scale_amplitude");

    if (_sdf->HasElement("run_duration"))
      this->runDuration = _sdf->Get<double>("run_duration");

    if (_sdf->HasElement("cycle_seconds"))
      this->cycleSeconds = _sdf->Get<double>("cycle_seconds");

    if (_sdf->HasElement("profile_exp"))
      this->profileExp = _sdf->Get<double>("profile_exp");

    if (_sdf->HasElement("noise_smoothing"))
      this->noiseSmoothing = _sdf->Get<double>("noise_smoothing");

    if (_sdf->HasElement("start_delay"))
      this->startDelay = _sdf->Get<double>("start_delay");

    // Escalar física também?
    if (_sdf->HasElement("scale_physics"))
      this->scalePhysics = _sdf->Get<bool>("scale_physics");

    // ===== AMPLITUDES POR EIXO =====
    // Começa com o valor global
    this->scaleAmpX = this->scaleAmplitude;
    this->scaleAmpY = this->scaleAmplitude;
    this->scaleAmpZ = this->scaleAmplitude;

    // Se existirem por eixo, sobrescrevem
    if (_sdf->HasElement("scale_amp_x"))
      this->scaleAmpX = _sdf->Get<double>("scale_amp_x");
    if (_sdf->HasElement("scale_amp_y"))
      this->scaleAmpY = _sdf->Get<double>("scale_amp_y");
    if (_sdf->HasElement("scale_amp_z"))
      this->scaleAmpZ = _sdf->Get<double>("scale_amp_z");

    this->startSimTime = this->world->SimTime();

    // transport para ~/visual
    this->node.reset(new transport::Node());
    this->node->Init(this->world->Name());
    this->visPub = this->node->Advertise<msgs::Visual>("~/visual");

    // nome escopado do visual (model::link::visual)
    this->scopedVisualName =
      this->model->GetScopedName() + "::" + this->linkName + "::" + this->visualName;
    this->scopedLinkName =
      this->model->GetScopedName() + "::" + this->linkName;

    gzmsg << "[TrunkScaler] Modelo=" << this->model->GetName()
          << " link=" << this->linkName
          << " scopedLink=" << this->scopedLinkName
          << " scopedVisual=" << this->scopedVisualName
          << " profile=" << this->profile
          << " scaleAmplitude(global)=" << this->scaleAmplitude
          << " scaleAmpX=" << this->scaleAmpX
          << " scaleAmpY=" << this->scaleAmpY
          << " scaleAmpZ=" << this->scaleAmpZ
          << " startDelay=" << this->startDelay
          << " scalePhysics=" << (this->scalePhysics ? "true" : "false")
          << "\n";

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&TrunkScaler::OnUpdate, this, std::placeholders::_1));
  }

private:
  void OnUpdate(const common::UpdateInfo &_info)
  {
    double t = (_info.simTime - this->startSimTime).Double();

    if (t < this->startDelay)
      return;

    if (this->runDuration > 0.0 && (t - this->startDelay) >= this->runDuration)
    {
      this->updateConnection.reset();
      gzmsg << "[TrunkScaler] run_duration atingido, parando.\n";
      return;
    }

    double elapsed = t - this->startDelay;

    double value01 = this->ComputeValue01(elapsed);

    // escala por eixo usando amplitudes diferentes
    double sx = this->ValueToScaleAxis(value01, this->scaleAmpX);
    double sy = this->ValueToScaleAxis(value01, this->scaleAmpY);
    double sz = this->ValueToScaleAxis(value01, this->scaleAmpZ);

    ignition::math::Vector3d scaleVec(sx, sy, sz);

    // 1) Física / colisão: OPCIONAL
    if (this->scalePhysics)
    {
      this->link->SetScale(scaleVec);
    }

    // 2) Visual na GUI: só altera a ESCALA, não a pose
    if (this->visPub)
    {
      msgs::Visual msg;
      msg.set_name(this->scopedVisualName);
      msg.set_parent_name(this->scopedLinkName);
      msg.set_is_static(false);
      msg.set_transparency(0.0);
      msg.set_visible(true);

      // NÃO seta pose -> não desloca o visual
      msgs::Set(msg.mutable_scale(), scaleVec);

      this->visPub->Publish(msg);

      if (!this->haveVisual)
      {
        this->haveVisual = true;
        gzmsg << "[TrunkScaler] Publicando visual para '"
              << this->scopedVisualName << "'\n";
      }
    }

    gzmsg << "[TrunkScaler] t=" << elapsed
          << " value01=" << value01
          << " scaleVec=(" << sx << ", " << sy << ", " << sz << ")\n";
  }

  double ComputeValue01(double elapsed)
  {
    double v = 0.5;

    if (this->profile == "oscillating")
    {
      double phase = 2.0 * M_PI * std::fmod(elapsed, this->cycleSeconds) / this->cycleSeconds;
      v = 0.5 + 0.5 * std::sin(phase);
    }
    else if (this->profile == "linear_up")
    {
      double dur = (this->runDuration > 0.0) ? this->runDuration : this->cycleSeconds;
      if (dur <= 0.0) dur = 1.0;
      v = std::min(elapsed / dur, 1.0);
    }
    else if (this->profile == "linear_down")
    {
      double dur = (this->runDuration > 0.0) ? this->runDuration : this->cycleSeconds;
      if (dur <= 0.0) dur = 1.0;
      v = std::max(1.0 - elapsed / dur, 0.0);
    }
    else if (this->profile == "triangle")
    {
      double p = std::fmod(elapsed, this->cycleSeconds) / this->cycleSeconds;
      if (p < 0.5)
        v = 2.0 * p;
      else
        v = 2.0 * (1.0 - p);
    }
    else if (this->profile == "saw")
    {
      v = std::fmod(elapsed, this->cycleSeconds) / this->cycleSeconds;
    }
    else if (this->profile == "noise")
    {
      double target = this->rand01();
      double alpha = this->noiseSmoothing;
      v = (1.0 - alpha) * this->lastValue01 + alpha * target;
    }
    else
    {
      v = 0.5;
    }

    v = std::max(0.0, std::min(1.0, v));

    if (this->profileExp > 0.0 && this->profileExp != 1.0)
      v = std::pow(v, this->profileExp);

    this->lastValue01 = v;
    return v;
  }

  // value_01 em [0,1] -> escala no eixo com amplitude específica
  double ValueToScaleAxis(double v, double amp) const
  {
    // 0 => 1 - amp ; 1 => 1 + amp
    return (1.0 - amp) + v * (2.0 * amp);
  }

  double rand01()
  {
    return double(this->rng() % 1000000) / 1000000.0;
  }

private:
  physics::ModelPtr model;
  physics::WorldPtr world;
  physics::LinkPtr  link;
  event::ConnectionPtr updateConnection;

  transport::NodePtr node;
  transport::PublisherPtr visPub;

  std::string linkName;
  std::string visualName;
  std::string scopedLinkName;
  std::string scopedVisualName;
  std::string profile{"oscillating"};

  // amplitudes globais e por eixo
  double scaleAmplitude{0.5};
  double scaleAmpX{0.5};
  double scaleAmpY{0.5};
  double scaleAmpZ{0.5};

  double runDuration{-1.0};
  double cycleSeconds{6.0};
  double profileExp{1.0};
  double noiseSmoothing{0.3};
  double startDelay{2.0};

  bool   scalePhysics{false};

  double lastValue01{0.5};
  common::Time startSimTime;

  bool haveVisual{false};

  std::mt19937 rng{12345};
};

GZ_REGISTER_MODEL_PLUGIN(TrunkScaler)
}  // namespace gazebo

