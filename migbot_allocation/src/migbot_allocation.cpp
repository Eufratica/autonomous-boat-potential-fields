// migbot_allocation_node.cpp
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Wrench.h>
#include <cmath>

// ====================== Constantes do modelo ======================
static constexpr float GRAVITY_MSS = 9.80665f;

// Forças máximas (cada motor)
static float FM1 = GRAVITY_MSS * 2.1f;
static float FM2 = GRAVITY_MSS * 2.1f;
static float FM3 = GRAVITY_MSS * 2.1f;
static float FM4 = GRAVITY_MSS * 2.1f;
static float FM5 = GRAVITY_MSS * 2.1f;
static float FM6 = GRAVITY_MSS * 2.1f;

static float Fmax = FM1 + FM2 + FM3 + FM4 + FM5 + FM6; // força total

// Geometria
static float L  = 2.3f;
static float Lx = L * std::cos(M_PI/4.0);
static float Ly = L * std::cos(M_PI/4.0);

// PWM “faixa de trabalho”
static float Pwmmax = 1001.0f;
static float Pwmmin = 1.0f;

// Ganhos por motor (N por unidade de PWM)
static float k1 = (FM1)/(Pwmmax - Pwmmin);
static float k2 = (FM2)/(Pwmmax - Pwmmin);
static float k3 = (FM3)/(Pwmmax - Pwmmin);
static float k4 = (FM4)/(Pwmmax - Pwmmin);
static float k5 = (FM5)/(Pwmmax - Pwmmin);
static float k6 = (FM6)/(Pwmmax - Pwmmin);

static float Nmax = L * Fmax;

// ====================== Estado do Wrench ======================
static float Fx_cmd = 0.0f;
static float Fy_cmd = 0.0f; // zerado no algoritmo
static float Tn_cmd = 0.0f;

// ====================== Utils ======================
inline float constrain_float(float v, float lo, float hi) {
  if (std::isnan(v)) return 0.5f*(lo+hi);
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

inline float PWMtoNorm(float pwm) {
  // [Pwmmin..Pwmmax] -> [0..1]
  float V = (pwm - Pwmmin) / (Pwmmax - Pwmmin);
  return constrain_float(V, 0.0f, 1.0f);
}

inline float NormtoPWM(float val01) {
  // [0..1] -> [Pwmmin..Pwmmax]
  return val01 * (Pwmmax - Pwmmin) + Pwmmin;
}

// Alocação simplificada: usa FX e TN (FY=0) para 6 propulsores
static void allocatePWM(float FX, float /*FY*/, float TN,
                        float &Pwm1, float &Pwm2, float &Pwm3,
                        float &Pwm4, float &Pwm5, float &Pwm6)
{
  // Normaliza comandos de entrada
  FX = constrain_float(FX, -1.0f, 1.0f);
  TN = constrain_float(TN, -1.0f, 1.0f);

  // Mapeia para unidades físicas
  float FXN = FX * Fmax;   // N
  float TNN = TN * Nmax;   // N*m

  // Métrica de “esforço total” para threshold (mesma ideia do seu FT)
  float FT = std::sqrt((TNN/L)*(TNN/L) + FXN*FXN);
  if (FT < 0.02f*Fmax) {
    // envia zero
    Pwm1 = Pwm2 = Pwm3 = Pwm4 = Pwm5 = Pwm6 = 0.0f; // normalizado [0..1]
    return;
  }

  // Fórmulas (iguais às do seu código original, sem FY)
  // Primeiro calculamos PWM em UNIDADES DE PULSO (não normalizado)
  float pwm1 = FXN/(4*k1) - TNN/(4*Ly*k1);
  float pwm2 = FXN/(4*k2) + TNN/(4*Ly*k2);
  float pwm3 = FXN/(4*k3) + TNN/(4*Ly*k3);
  float pwm4 = FXN/(4*k4) - TNN/(4*Ly*k4);
  float pwm5 = -FXN/(4*k5) - TNN/(4*Ly*k5);
  float pwm6 = -FXN/(4*k6) + TNN/(4*Ly*k6);

  // Saturação na faixa de PWM física
  pwm1 = constrain_float(pwm1, Pwmmin, Pwmmax);
  pwm2 = constrain_float(pwm2, Pwmmin, Pwmmax);
  pwm3 = constrain_float(pwm3, Pwmmin, Pwmmax);
  pwm4 = constrain_float(pwm4, Pwmmin, Pwmmax);
  pwm5 = constrain_float(pwm5, Pwmmin, Pwmmax);
  pwm6 = constrain_float(pwm6, Pwmmin, Pwmmax);

  // Converte para [0..1] p/ publicar “potência” normalizada
  Pwm1 = PWMtoNorm(pwm1);
  Pwm2 = PWMtoNorm(pwm2);
  Pwm3 = PWMtoNorm(pwm3);
  Pwm4 = PWMtoNorm(pwm4);
  Pwm5 = PWMtoNorm(pwm5);
  Pwm6 = PWMtoNorm(pwm6);
}

// ====================== ROS ======================
static void wrench_cb(const geometry_msgs::Wrench& msg) {
  Fx_cmd = msg.force.x;
  Fy_cmd = msg.force.y;     // não entra na alocação (mantido para futura extensão)
  Tn_cmd = msg.torque.z;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "migbot_allocation_node");
  ros::NodeHandle nh;          // tópicos RELATIVOS ao namespace do node
  ros::NodeHandle nh_priv("~");

  // (Opcional) permitir override via parâmetros
  nh_priv.param("L", L, L);
  Lx = L * std::cos(M_PI/4.0);
  Ly = L * std::cos(M_PI/4.0);
  nh_priv.param("Pwmmin", Pwmmin, Pwmmin);
  nh_priv.param("Pwmmax", Pwmmax, Pwmmax);

  // recomputa ganhos se PWM foi alterado por parâmetro
  k1 = (FM1)/(Pwmmax - Pwmmin);
  k2 = (FM2)/(Pwmmax - Pwmmin);
  k3 = (FM3)/(Pwmmax - Pwmmin);
  k4 = (FM4)/(Pwmmax - Pwmmin);
  k5 = (FM5)/(Pwmmax - Pwmmin);
  k6 = (FM6)/(Pwmmax - Pwmmin);
  Fmax = FM1 + FM2 + FM3 + FM4 + FM5 + FM6;
  Nmax = L * Fmax;

  // Publishers (relativos): resolvem para <ns>/Engine_helice_X_effort_controller/command
  auto pub_PWM1 = nh.advertise<std_msgs::Float64>("Engine_helice_1_effort_controller/command", 10);
  auto pub_PWM2 = nh.advertise<std_msgs::Float64>("Engine_helice_2_effort_controller/command", 10);
  auto pub_PWM3 = nh.advertise<std_msgs::Float64>("Engine_helice_3_effort_controller/command", 10);
  auto pub_PWM4 = nh.advertise<std_msgs::Float64>("Engine_helice_4_effort_controller/command", 10);
  auto pub_PWM5 = nh.advertise<std_msgs::Float64>("Engine_helice_5_effort_controller/command", 10);
  auto pub_PWM6 = nh.advertise<std_msgs::Float64>("Engine_helice_6_effort_controller/command", 10);

  // Subscriber (relativo): <ns>/Wrench
  auto sub = nh.subscribe("Wrench", 10, wrench_cb);

  ros::Rate loop_rate(100);

  while (ros::ok()) {
    float p1, p2, p3, p4, p5, p6;
    allocatePWM(Fx_cmd, Fy_cmd, Tn_cmd, p1, p2, p3, p4, p5, p6);

    // Mantém o mesmo mapeamento de sinais que você usava
    std_msgs::Float64 m1; m1.data =  100.0 * p1;
    std_msgs::Float64 m2; m2.data =  100.0 * p2;
    std_msgs::Float64 m3; m3.data = -100.0 * p3;
    std_msgs::Float64 m4; m4.data = -100.0 * p4;
    std_msgs::Float64 m5; m5.data = -100.0 * p5; // to check
    std_msgs::Float64 m6; m6.data = -100.0 * p6; // to check

    pub_PWM1.publish(m1);
    pub_PWM2.publish(m2);
    pub_PWM3.publish(m3);
    pub_PWM4.publish(m4);
    pub_PWM5.publish(m5);
    pub_PWM6.publish(m6);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

