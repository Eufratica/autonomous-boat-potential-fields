# Blender -> URDF (.urdf.xacro) para ROS Noetic / Gazebo Classic
# - Cores fiéis (PHONG + diffuse(color) -> PNG 1x1 + retarget)
# - Geometria “lisinha”: Triangulate (+Weighted Normal) + Shade Smooth
# - Só VISÍVEIS; 1 helice.dae; colisão do base_link a partir de cubos col0..colN
# - Export sem “desmontar”: bake ROT+SCALE no mesh; location NÃO aplicada
# - Xacro: migbot2.urdf.xacro

import bpy, os, re, math, copy, xml.etree.ElementTree as ET
from xml.dom.minidom import parseString
from mathutils import Vector
from bpy_extras.io_utils import ExportHelper
from bpy.props import StringProperty, FloatProperty, BoolProperty
from bpy.types import Operator

# ================== CONFIG ==================
MODEL_NAME           = "migbot2"
XACRO_FILENAME       = "migbot2.urdf.xacro"

DENSITY_DEFAULT      = 90.0    # kg/m³ (massa ~ densidade * volume AABB)
VISUAL_AXIS_FIX_FOR_COLLADA = False  # -90° em X no VISUAL (se aparecer deitado no Gazebo)

# Aparência PHONG
PHONG_AMBIENT        = 1.00
PHONG_EMISSION       = 0.45
PHONG_SPECULAR       = 0.10
PHONG_SHININESS      = 15.0

# Cor -> PNG 1x1
EXPOSURE_GAIN        = 1.35
PNG_WRITE_SRGB       = True

VERBOSE              = True
COL_RE               = re.compile(r"^col\d+$")  # colisão: col0..colN

# Namespaces / xacro
XACRO_NS = "http://www.ros.org/wiki/xacro"
ET.register_namespace("xacro", XACRO_NS)

# Bloco extra de XACRO / Gazebo / ArduPilot a ser injetado no final do <robot>
EXTRA_XACRO = """
  <!-- =========================
       MOTORES (motor_model)
       ========================= -->
  <xacro:macro name="migbot_motor" params="idx turn motor_topic_idx">
    <gazebo>
      <plugin name="motor${idx}_model" filename="libgazebo_motor_model.so">
        <robotNamespace>${namespace}</robotNamespace>
        <jointName>Engine_helice_${idx}</jointName>
        <linkName>Helice_${idx}</linkName>
        <turningDirection>${turn}</turningDirection>
        <timeConstantUp>0.0125</timeConstantUp>
        <timeConstantDown>0.025</timeConstantDown>
        <maxRotVelocity>110000</maxRotVelocity>
        <motorConstant>1e-02</motorConstant>
        <momentConstant>0.01</momentConstant>
        <commandSubTopic>/gazebo/command/motor_speed</commandSubTopic>
        <motorNumber>1</motorNumber>
        <rotorDragCoefficient>0.000806428</rotorDragCoefficient>
        <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
        <motorSpeedPubTopic>/motor_speed/${motor_topic_idx}</motorSpeedPubTopic>
        <rotorVelocitySlowdownSim>1</rotorVelocitySlowdownSim>
      </plugin>
    </gazebo>
    
    <!-- transmissions só quando o controle é via ROS (não via ArduPilot) -->
    <xacro:unless value="${ardupilot}">
      <transmission name="Engine_helice_${idx}_trans">
        <type>transmission_interface/SimpleTransmission</type>
        <actuator name="Engine_helice_${idx}_act">
          <mechanicalReduction>1</mechanicalReduction>
        </actuator>
        <joint name="Engine_helice_${idx}">
          <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        </joint>
      </transmission>
    </xacro:unless>
  </xacro:macro>

  <!-- Instâncias dos 6 motores (ajuste os 'turn' se precisar) -->
  <xacro:migbot_motor idx="1" turn="cw"  motor_topic_idx="1"/>
  <xacro:migbot_motor idx="2" turn="cw"  motor_topic_idx="2"/>
  <xacro:migbot_motor idx="3" turn="ccw" motor_topic_idx="3"/>
  <xacro:migbot_motor idx="4" turn="ccw" motor_topic_idx="4"/>
  <xacro:migbot_motor idx="5" turn="cw"  motor_topic_idx="5"/>
  <xacro:migbot_motor idx="6" turn="ccw" motor_topic_idx="6"/>

  <!-- =========================
       SENSORES (IMU / MAG / GPS)
       ========================= -->
  <link name="imu_link">
    <inertial>
      <mass value="${imu_mass}"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="0.00532259139" ixy="0" ixz="0"
               iyy="0.00363239675" iyz="-0.00067637070" izz="0.00234468738"/>
    </inertial>
    <sensor name="imu_sensor_inline" type="imu">
      <pose>0 0 0 3.141593 0 0</pose>
    </sensor>
  </link>

  <joint name="imu_joint" type="revolute">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin rpy="0 0 0" xyz="-0.01 0.002 0.082"/>
    <axis xyz="1 0 0"/>
    <limit effort="0.0" lower="0" upper="0" velocity="0"/>
    <dynamics damping="0.7" friction="0" spring_reference="0" spring_stiffness="0"/>
  </joint>

  <gazebo reference="imu_link">
    <gravity>true</gravity>
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>1000</update_rate>
      <visualize>true</visualize>
      <topic>__default_topic__</topic>
      <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
        <topicName>imu_data</topicName>
        <bodyName>imu_link</bodyName>
        <updateRateHZ>10.0</updateRateHZ>
        <gaussianNoise>0.0</gaussianNoise>
        <xyzOffset>0 0 0</xyzOffset>
        <rpyOffset>0 0 0</rpyOffset>
        <frameName>imu_link</frameName>
        <initialOrientationAsReference>false</initialOrientationAsReference>
      </plugin>
      <pose>0 0 0 3.141593 0 0</pose>
    </sensor>
  </gazebo>

  <gazebo>
    <plugin name="quadrotor_imu_sim" filename="libhector_gazebo_ros_imu.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>100.0</updateRate>
      <bodyName>base_link</bodyName>
      <topicName>/imu</topicName>
      <gaussianNoise>1.3e-5 1.3e-5 1.3e-5</gaussianNoise>
    </plugin>
  </gazebo>

  <gazebo>
    <plugin name="quadrotor_magnetic_sim" filename="libhector_gazebo_ros_magnetic.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>100.0</updateRate>
      <referenceHeading>-90</referenceHeading>
      <declination>-13.9794</declination>
      <inclination>-2.0879</inclination>
      <bodyName>base_link</bodyName>
      <topicName>/magnetic</topicName>
      <magnitude>1000.0</magnitude>
      <offset>0 0 0</offset>
      <drift>0.0 0.0 0.0</drift>
      <gaussianNoise>1.3e-5 1.3e-5 1.3e-5</gaussianNoise>
    </plugin>
  </gazebo>

  <gazebo>
    <plugin name="quadrotor_gps_sim" filename="libhector_gazebo_ros_gps.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>10.0</updateRate>
      <bodyName>base_link</bodyName>
      <topicName>/fix</topicName>
      <velocityTopicName>/fix_velocity</velocityTopicName>
      <drift>0.0 0.0 0.0</drift>
      <gaussianNoise>0.01 0.01 0.01</gaussianNoise>
      <velocityDrift>0 0 0</velocityDrift>
      <velocityGaussianNoise>0.01 0.01 0.01</velocityGaussianNoise>
      <referenceLatitude>${ref_lat}</referenceLatitude>
      <referenceLongitude>${ref_lon}</referenceLongitude>
      <referenceHeading>-90</referenceHeading>
      <referenceAltitude>${ref_alt}</referenceAltitude>
    </plugin>
  </gazebo>

  <link name="mag_link">
    <inertial>
      <origin xyz="0 0 0" rpy="3.141593 0 0"/>
      <mass value="${mag_mass}"/>
      <inertia ixx="1e-05" ixy="0" ixz="0" iyy="1e-05" iyz="0" izz="1e-05"/>
    </inertial>
    <visual name="mag_vis">
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="0.01 0.01 0.01"/></geometry>
    </visual>
    <collision name="mag_col">
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="0.01 0.01 0.01"/></geometry>
    </collision>
  </link>
  <joint name="mag_joint" type="fixed">
    <parent link="base_link"/>
    <child link="mag_link"/>
    <origin xyz="0.1 0.1 0.1" rpy="0 0 0"/>
  </joint>

  <!-- =========================
       DINÂMICA USV / BOIÂNCIA / FOLLOW
       ========================= -->
  <gazebo>
    <plugin name="usv_dynamics_plugin" filename="libgazebo_usv_dynamics_plugin.so">
      <bodyName>base_link</bodyName>
      <waterLevel>-0.15</waterLevel>
      <waterDensity>${fluid_density}</waterDensity>
      <xDotU>0.0</xDotU>
      <yDotV>0.0</yDotV>
      <nDotR>0.0</nDotR>
      <xU>31.3</xU>
      <xUU>42.4</xUU>
      <yV>40.0</yV>
      <yVV>0.0</yVV>
      <zW>480.0</zW>
      <kP>50.0</kP>
      <mQ>50.0</mQ>
      <nR>380.0</nR>
      <nRR>0.0</nRR>
      <hullRadius>${hull_radius}</hullRadius>
      <boatWidth>${boat_width}</boatWidth>
      <boatLength>${boat_length}</boatLength>
      <length_n>2</length_n>
      <wave_model>${wave_model}</wave_model>
    </plugin>
  </gazebo>

  <gazebo reference="base_link">
    <plugin name="MigbotBuoyancyPlugin" filename="libbuoyancy_gazebo_plugin.so">
      <wave_model>${wave_model}</wave_model>
      <fluid_density>${fluid_density}</fluid_density>
      <fluid_level>${fluid_level}</fluid_level>
      <linear_drag>1.0</linear_drag>
      <angular_drag>1.0</angular_drag>
      <buoyancy name="buoyancy_box">
        <link_name>base_link</link_name>
        <pose>0 0 0.1 0 0 0</pose>
        <geometry>
          <box>
            <size>${0.2*S} ${0.1*S} ${0.05*S}</size>
          </box>
        </geometry>
      </buoyancy>
    </plugin>
  </gazebo>

  <gazebo>
    <plugin name="MigbotFollowPlugin" filename="libfollow_plugin.so">
      <link_name>base_link</link_name>
      <loop_forever>1</loop_forever>
      <line>
        <direction>0</direction>
        <length>1</length>
        <force>0</force>
        <torque>0</torque>
      </line>
      <frame>world</frame>
    </plugin>
  </gazebo>

  <!-- =========================
       ROS CONTROL / JOINT STATES
       ========================= -->
  <gazebo>
    <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
      <robotNamespace>${namespace}</robotNamespace>
      <jointName>
        Engine_helice_1,
        Engine_helice_2,
        Engine_helice_3,
        Engine_helice_4,
        Engine_helice_5,
        Engine_helice_6
      </jointName>
    </plugin>
  </gazebo>
  
  <xacro:unless value="${ardupilot}">
    <gazebo>
      <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
        <robotNamespace>${namespace}</robotNamespace>
        <legacyModeNS>true</legacyModeNS>
        <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
      </plugin>
    </gazebo>
  </xacro:unless>  

  <!-- Ardupilot plugin -->
  <xacro:if value="${ardupilot}">
   <gazebo>
    <plugin name="arducopter_plugin" filename="libArduPilotPlugin.so">
      <fdm_addr>127.0.0.1</fdm_addr> 
      <fdm_port_in>9002</fdm_port_in>
      <fdm_port_out>9003</fdm_port_out>   
           
      <modelXYZToAirplaneXForwardZDown>0 0  0 3.141593 0 0</modelXYZToAirplaneXForwardZDown>
      <gazeboXYZToNED>0 0 0 3.141593 0 -1.57075</gazeboXYZToNED>
      <imuName>imu_sensor</imuName> 
      <gpsName>gpssensor</gpsName>   
      <magName>magsensor</magName> 
      <!-- <barName>barsensor</barName>  -->
        
      <connectionTimeoutMaxCount>50</connectionTimeoutMaxCount>
      
      <control channel="0">
        <type>EFFORT</type>
        <offset>0</offset>
        <p_gain>0.20</p_gain>
        <i_gain>0</i_gain>
        <d_gain>0</d_gain>
        <i_max>0</i_max>
        <i_min>0</i_min>
        <cmd_max>2.5</cmd_max>
        <cmd_min>-2.5</cmd_min>
        <jointName>Engine_helice_1</jointName>
        <multiplier>100</multiplier>
        <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
      </control>
      <control channel="1">
        <type>EFFORT</type>
        <offset>0</offset>
        <p_gain>0.20</p_gain>
        <i_gain>0</i_gain>
        <d_gain>0</d_gain>
        <i_max>0</i_max>
        <i_min>0</i_min>
        <cmd_max>2.5</cmd_max>
        <cmd_min>-2.5</cmd_min>
        <jointName>Engine_helice_2</jointName>
        <multiplier>100</multiplier>
        <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
      </control>
      <control channel="2">
        <type>EFFORT</type>
        <offset>0</offset>
        <p_gain>0.20</p_gain>
        <i_gain>0</i_gain>
        <d_gain>0</d_gain>
        <i_max>0</i_max>
        <i_min>0</i_min>
        <cmd_max>2.5</cmd_max>
        <cmd_min>-2.5</cmd_min>
        <jointName>Engine_helice_3</jointName>
        <multiplier>-100</multiplier>
        <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
      </control>
      <control channel="3">
        <type>EFFORT</type>
        <offset>0</offset>
        <p_gain>0.20</p_gain>
        <i_gain>0</i_gain>
        <d_gain>0</d_gain>
        <i_max>0</i_max>
        <i_min>0</i_min>
        <cmd_max>2.5</cmd_max>
        <cmd_min>-2.5</cmd_min>
        <jointName>Engine_helice_4</jointName>
        <multiplier>-100</multiplier>
        <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
      </control>
      <control channel="4">
        <type>EFFORT</type>
        <offset>0</offset>
        <p_gain>0.20</p_gain>
        <i_gain>0</i_gain>
        <d_gain>0</d_gain>
        <i_max>0</i_max>
        <i_min>0</i_min>
        <cmd_max>2.5</cmd_max>
        <cmd_min>-2.5</cmd_min>
        <jointName>Engine_helice_5</jointName>
        <multiplier>100</multiplier>
        <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
      </control>
      <control channel="5">
        <type>EFFORT</type>
        <offset>0</offset>
        <p_gain>0.20</p_gain>
        <i_gain>0</i_gain>
        <d_gain>0</d_gain>
        <i_max>0</i_max>
        <i_min>0</i_min>
        <cmd_max>2.5</cmd_max>
        <cmd_min>-2.5</cmd_min>
        <jointName>Engine_helice_6</jointName>
        <multiplier>-100</multiplier>
        <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
      </control>       
    </plugin> 
   </gazebo>
  </xacro:if>
"""

def append_extra_xacro(robot_elem):
    """Anexa o bloco EXTRA_XACRO como filhos adicionais do <robot>."""
    if not EXTRA_XACRO.strip():
        return
    wrapper = f'<root xmlns:xacro="{XACRO_NS}">{EXTRA_XACRO}</root>'
    root = ET.fromstring(wrapper)
    for child in list(root):
        robot_elem.append(child)

# ============== Utils / Seleção ==============
def ensure_dir(p):
    os.makedirs(p, exist_ok=True)
    return p

def pretty_xml(e):
    return parseString(ET.tostring(e)).toprettyxml(indent="  ")

def visible_meshes():
    bpy.context.view_layer.update()
    return [o for o in bpy.context.scene.objects if o.type == 'MESH' and o.visible_get()]

def base_link_object(objs):
    for o in objs:
        if o.name == "base_link":
            return o
    for o in objs:
        if o.name.startswith("base_link"):
            return o
    return None

def aabb_local(obj):
    sx, sy, sz = obj.scale
    bb = [(v[0]*sx, v[1]*sy, v[2]*sz) for v in obj.bound_box]
    xs = [p[0] for p in bb]
    ys = [p[1] for p in bb]
    zs = [p[2] for p in bb]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    zmin, zmax = min(zs), max(zs)
    cx = (xmin + xmax) / 2.0
    cy = (ymin + ymax) / 2.0
    cz = (zmin + zmax) / 2.0
    sx = (xmax - xmin)
    sy = (ymax - ymin)
    sz = (zmax - zmin)
    return (cx, cy, cz, sx, sy, sz)

def principal_axis_from_extents(sx, sy, sz):
    if sx >= sy and sx >= sz:
        return 'X'
    if sy >= sx and sy >= sz:
        return 'Y'
    return 'Z'

# ============== Inércia simples ==============
def inertia_box(sx, sy, sz, density):
    vol = max(sx * sy * sz, 1e-9)
    m = density * vol
    ixx = (1/12.0) * m * (sy*sy + sz*sz)
    iyy = (1/12.0) * m * (sx*sx + sz*sz)
    izz = (1/12.0) * m * (sx*sx + sy*sy)
    return m, ixx, iyy, izz

def inertia_cylinder_axis(axis, sx, sy, sz, density):
    if axis == 'X':
        L = max(sx, 1e-6)
        R = 0.5 * max(sy, sz)
        vol = math.pi * R * R * L
        m = density * vol
        Ixx = 0.5 * m * R * R
        Iyy = (1/12.0) * m * (3*R*R + L*L)
        Izz = Iyy
        return m, Ixx, Iyy, Izz, R, L
    if axis == 'Y':
        L = max(sy, 1e-6)
        R = 0.5 * max(sx, sz)
        vol = math.pi * R * R * L
        m = density * vol
        Iyy = 0.5 * m * R * R
        Ixx = (1/12.0) * m * (3*R*R + L*L)
        Izz = Ixx
        return m, Ixx, Iyy, Izz, R, L
    L = max(sz, 1e-6)
    R = 0.5 * max(sx, sy)
    vol = math.pi * R * R * L
    m = density * vol
    Izz = 0.5 * m * R * R
    Ixx = (1/12.0) * m * (3*R*R + L*L)
    Iyy = Ixx
    return m, Izz, Ixx, Iyy, R, L  # ordem cuidada

# ============== Namespaces/XML ==============
def _ns_of(tag):
    if tag and tag.startswith('{'):
        return tag[1:].split('}')[0]
    return ''

def _q(ns, tag):
    return f'{{{ns}}}{tag}' if ns else tag

def _write_collada(tree, dae_path):
    root = tree.getroot()
    ns = _ns_of(root.tag)
    if ns:
        try:
            ET.register_namespace('', ns)
        except Exception:
            pass
    tree.write(dae_path, encoding="utf-8", xml_declaration=True)

# ============== Collada exporter compat ==============
def collada_export_safe(**params):
    """
    Chama bpy.ops.wm.collada_export só com os argumentos suportados pela versão atual do Blender.
    Evita erros do tipo 'keyword ... unrecognized'.
    """
    op = bpy.ops.wm.collada_export
    try:
        props = {p.identifier for p in op.get_rna_type().properties}
    except Exception:
        props = set()

    kwargs = {}
    # essenciais
    kwargs['filepath'] = params.get('filepath', '/tmp/out.dae')
    if 'selected' in props:
        kwargs['selected'] = params.get('selected', True)
    if 'check_existing' in props:
        kwargs['check_existing'] = params.get('check_existing', False)

    # comuns, se existirem
    for k, v in (
        ('apply_modifiers', True),
        ('triangulate', True),
        ('use_blender_profile', False),
        ('include_uv_textures', True),
        ('include_material_textures', True),
    ):
        if k in props:
            kwargs[k] = v

    try:
        op(**kwargs)
    except TypeError:
        # fallback mínimo
        op(filepath=kwargs['filepath'])

# ============== PHONG + color->texture ==============
def fix_dae_material_shader(dae_path):
    """lambert/blinn → phong; ajusta ambient/emission/specular/shininess; preserva diffuse/texture."""
    try:
        tree = ET.parse(dae_path)
        root = tree.getroot()
        ns = _ns_of(root.tag)
        for eff in root.findall(f'.//{_q(ns,"effect")}'):
            prof = eff.find(f'.//{_q(ns,"profile_COMMON")}')
            tech = prof and prof.find(f'.//{_q(ns,"technique")}')
            if tech is None:
                continue
            lb = tech.find(f'./{_q(ns,"lambert")}') or tech.find(f'./{_q(ns,"blinn")}')
            if lb is None:
                phong = tech.find(f'./{_q(ns,"phong")}')
                if phong is None:
                    continue
                amb = phong.find(f'./{_q(ns,"ambient")}') or ET.SubElement(phong, _q(ns, "ambient"))
                (amb.find(f'./{_q(ns,"color")}') or ET.SubElement(amb, _q(ns, "color"))).text = f"{PHONG_AMBIENT} {PHONG_AMBIENT} {PHONG_AMBIENT} 1.0"
                emi = phong.find(f'./{_q(ns,"emission")}') or ET.SubElement(phong, _q(ns, "emission"))
                (emi.find(f'./{_q(ns,"color")}') or ET.SubElement(emi, _q(ns, "color"))).text = f"{PHONG_EMISSION} {PHONG_EMISSION} {PHONG_EMISSION} 1.0"
                spec = phong.find(f'./{_q(ns,"specular")}') or ET.SubElement(phong, _q(ns, "specular"))
                (spec.find(f'./{_q(ns,"color")}') or ET.SubElement(spec, _q(ns, "color"))).text = f"{PHONG_SPECULAR} {PHONG_SPECULAR} {PHONG_SPECULAR} 1.0"
                shiny = phong.find(f'./{_q(ns,"shininess")}') or ET.SubElement(phong, _q(ns, "shininess"))
                (shiny.find(f'./{_q(ns,"float")}') or ET.SubElement(shiny, _q(ns, "float"))).text = f"{PHONG_SHININESS}"
                continue

            phong = ET.Element(_q(ns, "phong"))
            # emission
            emi = lb.find(f'./{_q(ns,"emission")}') or ET.Element(_q(ns, "emission"))
            (emi.find(f'./{_q(ns,"color")}') or ET.SubElement(emi, _q(ns, "color"))).text = f"{PHONG_EMISSION} {PHONG_EMISSION} {PHONG_EMISSION} 1.0"
            phong.append(copy.deepcopy(emi))
            # ambient
            amb = lb.find(f'./{_q(ns,"ambient")}') or ET.Element(_q(ns, "ambient"))
            (amb.find(f'./{_q(ns,"color")}') or ET.SubElement(amb, _q(ns, "color"))).text = f"{PHONG_AMBIENT} {PHONG_AMBIENT} {PHONG_AMBIENT} 1.0"
            phong.append(copy.deepcopy(amb))
            # diffuse
            diff_new = ET.SubElement(phong, _q(ns, "diffuse"))
            diff_old = lb.find(f'./{_q(ns,"diffuse")}')
            if diff_old is not None:
                tex = diff_old.find(f'.//{_q(ns,"texture")}')
                col = diff_old.find(f'.//{_q(ns,"color")}')
                if tex is not None:
                    diff_new.append(copy.deepcopy(tex))
                else:
                    if col is None:
                        col = ET.Element(_q(ns, "color"))
                        col.text = "0.8 0.8 0.8 1.0"
                    diff_new.append(copy.deepcopy(col))
            else:
                ET.SubElement(diff_new, _q(ns, "color")).text = "0.8 0.8 0.8 1.0"
            # spec/shiny mais suaves
            spec = ET.SubElement(phong, _q(ns, "specular"))
            ET.SubElement(spec, _q(ns, "color")).text = f"{PHONG_SPECULAR} {PHONG_SPECULAR} {PHONG_SPECULAR} 1.0"
            shiny = ET.SubElement(phong, _q(ns, "shininess"))
            ET.SubElement(shiny, _q(ns, "float")).text = f"{PHONG_SHININESS}"
            tech.remove(lb)
            tech.append(phong)
        _write_collada(tree, dae_path)
        if VERBOSE:
            print(f"[PHONG] {os.path.basename(dae_path)}")
    except Exception as e:
        print(f"[PHONG][ERRO] {dae_path}: {e}")

def _lin_to_srgb(c: float) -> float:
    c = max(0.0, min(1.0, c))
    return 12.92*c if c <= 0.0031308 else 1.055*(c**(1/2.4)) - 0.055

def promote_color_diffuse_to_texture(dae_path, textures_dir):
    """Transforma diffuse(color) em texture (PNG 1×1) para o Gazebo (OGRE) respeitar as cores."""
    try:
        tree = ET.parse(dae_path)
        root = tree.getroot()
        ns = _ns_of(root.tag)
        os.makedirs(textures_dir, exist_ok=True)
        lib_imgs = root.find(f'./{_q(ns,"library_images")}') or ET.SubElement(root, _q(ns, "library_images"))

        def png_name_from_rgba(rgba):
            r = int(max(0, min(1, rgba[0]*EXPOSURE_GAIN)) * 255)
            g = int(max(0, min(1, rgba[1]*EXPOSURE_GAIN)) * 255)
            b = int(max(0, min(1, rgba[2]*EXPOSURE_GAIN)) * 255)
            a = int(max(0, min(1, rgba[3] if len(rgba) > 3 else 1)) * 255)
            if PNG_WRITE_SRGB:
                r, g, b = int(_lin_to_srgb(r/255.0)*255), int(_lin_to_srgb(g/255.0)*255), int(_lin_to_srgb(b/255.0)*255)
            return f"flat_{r:02x}{g:02x}{b:02x}{a:02x}.png"

        def ensure_png(rgba):
            fn = png_name_from_rgba(rgba)
            full = os.path.join(textures_dir, fn)
            if not os.path.exists(full):
                img = bpy.data.images.new(fn, width=1, height=1, alpha=True, float_buffer=False)
                try:
                    img.colorspace_settings.name = 'sRGB'
                    img.colorspace_settings.is_data = False
                except Exception:
                    pass
                img.alpha_mode = 'STRAIGHT'
                img.pixels = [rgba[0], rgba[1], rgba[2], rgba[3] if len(rgba) > 3 else 1.0]
                img.filepath_raw = full
                img.file_format = 'PNG'
                img.save()
                bpy.data.images.remove(img)
            return fn

        edits = 0
        for eff in root.findall(f'.//{_q(ns,"effect")}'):
            prof = eff.find(f'./{_q(ns,"profile_COMMON")}')
            tech = prof and prof.find(f'./{_q(ns,"technique")}')
            if tech is None:
                continue
            shader = (tech.find(f'./{_q(ns,"phong")}')
                      or tech.find(f'./{_q(ns,"lambert")}')
                      or tech.find(f'./{_q(ns,"blinn")}'))
            if shader is None:
                continue
            diff = shader.find(f'./{_q(ns,"diffuse")}')
            if diff is None or diff.find(f'./{_q(ns,"texture")}') is not None:
                continue
            col = diff.find(f'./{_q(ns,"color")}')
            if col is None or not (col.text or "").strip():
                continue

            try:
                rgba = [float(x) for x in col.text.strip().split()]
                while len(rgba) < 4:
                    rgba.append(1.0)
            except Exception:
                rgba = [0.8, 0.8, 0.8, 1.0]

            png_name = ensure_png(rgba)
            tex_id = os.path.splitext(png_name)[0]

            if lib_imgs.find(f'./{_q(ns,"image")}[@id="{tex_id}"]') is None:
                im = ET.SubElement(lib_imgs, _q(ns, "image"), id=tex_id, name=tex_id)
                ET.SubElement(im, _q(ns, "init_from")).text = png_name

            np_surf = ET.SubElement(prof, _q(ns, "newparam"), sid=f"{tex_id}-surface")
            surf = ET.SubElement(np_surf, _q(ns, "surface"), type="2D")
            ET.SubElement(surf, _q(ns, "init_from")).text = tex_id

            np_samp = ET.SubElement(prof, _q(ns, "newparam"), sid=f"{tex_id}-sampler")
            samp = ET.SubElement(np_samp, _q(ns, "sampler2D"))
            ET.SubElement(samp, _q(ns, "source")).text = f"{tex_id}-surface"

            diff.remove(col)
            ET.SubElement(diff, _q(ns, "texture"), texture=f"{tex_id}-sampler", texcoord="CHANNEL0")
            edits += 1

        if edits and VERBOSE:
            print(f"[FLAT→TEX] {edits} em {os.path.basename(dae_path)}")
        _write_collada(tree, dae_path)
    except Exception as e:
        print(f"[FLAT→TEX][ERRO] {dae_path}: {e}")

def _images_used_by_object(obj) -> set:
    imgs = set()
    if obj.type != 'MESH':
        return imgs
    for mat in obj.data.materials:
        if not mat or not mat.use_nodes:
            continue
        for n in mat.node_tree.nodes:
            if getattr(n, "type", "") == "TEX_IMAGE" and getattr(n, "image", None):
                imgs.add(n.image)
    return imgs

def _save_image_as(img: bpy.types.Image, out_dir: str, basename: str) -> str:
    os.makedirs(out_dir, exist_ok=True)
    dst = os.path.join(out_dir, basename)
    tmp = img.copy()
    try:
        ext = os.path.splitext(basename)[1].lower()
        tmp.file_format = 'PNG' if ext not in ('.png', '.jpg', '.jpeg', '.tga', '.bmp') else ('PNG' if ext == '.png' else 'JPEG')
        if tmp.file_format == 'JPEG' and ext not in ('.jpg', '.jpeg'):
            dst = os.path.splitext(dst)[0] + ".jpg"
        tmp.filepath_raw = dst
        tmp.save()
    except Exception:
        tmp.file_format = 'PNG'
        dst = os.path.splitext(dst)[0] + ".png"
        tmp.filepath_raw = dst
        tmp.save()
    finally:
        bpy.data.images.remove(tmp)
    return os.path.basename(dst)

def _retarget_dae_images_strict(dae_path, obj):
    """Garante que cada <image><init_from> exista ao lado do .dae (sem duplicar nomes tipo .001)."""
    try:
        tree = ET.parse(dae_path)
        root = tree.getroot()
        ns = _ns_of(root.tag)
        dae_dir = os.path.dirname(dae_path)
        used_imgs = _images_used_by_object(obj)
        by_clean = {re.sub(r'\W+', '', (im.name or '').lower()): im for im in used_imgs}
        changed = 0
        for img in root.findall(f'.//{_q(ns,"library_images")}/{_q(ns,"image")}'):
            init = img.find(f'./{_q(ns,"init_from")}')
            if init is None or not (init.text or "").strip():
                continue
            want_basename = os.path.basename(init.text.strip())
            target = os.path.join(dae_dir, want_basename)
            if not os.path.exists(target):
                key = re.sub(r'\W+', '', os.path.splitext(want_basename)[0].lower())
                cand = by_clean.get(key) or (next(iter(used_imgs)) if used_imgs else None)
                if cand:
                    _save_image_as(cand, dae_dir, want_basename)
                    changed += 1
            tex_id = os.path.splitext(want_basename)[0]
            img.set("id", tex_id)
            img.set("name", tex_id)
            init.text = want_basename
        if changed and VERBOSE:
            print(f"[IMG RETARGET] {os.path.basename(dae_path)} ({changed} salvas)")
        _write_collada(tree, dae_path)
    except Exception as e:
        print(f"[IMG RETARGET][ERRO] {dae_path}: {e}")

# ============== Export DAE (bake R+S, sem L) ==============
def export_object_at_origin(obj, dae_path, textures_dir):
    """
    Duplica o objeto, aplica ROT+SCALE no mesh (location NÃO),
    move a cópia para a origem e exporta com modifiers.
    Pós-processa: PHONG + diffuse(color)->PNG 1×1 + retarget imagens.
    """
    # cria cópia temporária
    tmp = obj.copy()
    tmp.data = obj.data.copy()
    bpy.context.collection.objects.link(tmp)

    prev_active = bpy.context.view_layer.objects.active
    try:
        # preparar sombreamento (sem “enrugado”)
        bpy.context.view_layer.objects.active = tmp
        try:
            # Triangulate consistente
            tri = tmp.modifiers.new(name="__Tri__", type='TRIANGULATE')
            tri.keep_custom_normals = True
            tri.quad_method = 'SHORTEST_DIAGONAL'
            tri.ngon_method = 'BEAUTY'
        except Exception:
            pass
        # (Opcional) Weighted Normal ajuda em planos
        try:
            wn = tmp.modifiers.new(name="__WN__", type='WEIGHTED_NORMAL')
            wn.keep_sharp = True
        except Exception:
            pass
        # Shade Smooth + Auto Smooth 60°
        try:
            bpy.ops.object.shade_smooth()
        except Exception:
            pass
        try:
            tmp.data.use_auto_smooth = True
            tmp.data.auto_smooth_angle = math.radians(60.0)
        except Exception:
            pass

        # bake ROT+SCALE no mesh (location NÃO)
        bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
        # exporta na origem (URDF/joints cuidam da pose)
        tmp.location = (0.0, 0.0, 0.0)
        tmp.rotation_euler = (0.0, 0.0, 0.0)

        bpy.ops.object.select_all(action='DESELECT')
        tmp.select_set(True)
        collada_export_safe(filepath=dae_path, selected=True, check_existing=False)
    finally:
        # limpa cópia
        try:
            bpy.data.objects.remove(tmp, do_unlink=True)
        except Exception:
            pass
        if prev_active:
            bpy.context.view_layer.objects.active = prev_active

    # Pós-processamento Collada (ordem importa)
    fix_dae_material_shader(dae_path)
    promote_color_diffuse_to_texture(dae_path, textures_dir)
    _retarget_dae_images_strict(dae_path, obj)

# ============== URDF helpers ==============
def urdf_origin(elem, xyz, rpy):
    o = ET.SubElement(elem, "origin")
    o.set("xyz", f"{xyz[0]} {xyz[1]} {xyz[2]}")
    o.set("rpy", f"{rpy[0]} {rpy[1]} {rpy[2]}")
    return o

def urdf_inertial(parent, m, ixx, iyy, izz, com_xyz):
    it = ET.SubElement(parent, "inertial")
    urdf_origin(it, com_xyz, (0, 0, 0))
    ET.SubElement(it, "mass").set("value", f"{m:.6f}")
    I = ET.SubElement(it, "inertia")
    I.set("ixx", f"{ixx:.6f}")
    I.set("ixy", "0.0")
    I.set("ixz", "0.0")
    I.set("iyy", f"{iyy:.6f}")
    I.set("iyz", "0.0")
    I.set("izz", f"{izz:.6f}")

def make_link_visual(parent, name, mesh_abs):
    link = ET.SubElement(parent, "link", {"name": name})
    if mesh_abs:
        vis = ET.SubElement(link, "visual")
        if VISUAL_AXIS_FIX_FOR_COLLADA:
            urdf_origin(vis, (0, 0, 0), (-math.pi/2, 0, 0))
        else:
            urdf_origin(vis, (0, 0, 0), (0, 0, 0))
        g = ET.SubElement(vis, "geometry")
        m = ET.SubElement(g, "mesh")
        m.set("filename", "file://" + mesh_abs)
    return link

def box_pose_size_in_parent(parent_obj, box_obj):
    # AABB no frame local do cubo → transforma para base_link
    cx, cy, cz, sx, sy, sz = aabb_local(box_obj)
    Tb = parent_obj.matrix_world.copy()
    Tb_inv = Tb.copy()
    Tb_inv.invert()
    To = box_obj.matrix_world.copy()
    Trel = Tb_inv @ To
    center_local = Vector((cx, cy, cz, 1.0))
    center_in_base = Trel @ center_local
    rpy = Trel.to_euler('XYZ')
    return (center_in_base.x, center_in_base.y, center_in_base.z), (rpy.x, rpy.y, rpy.z), (sx, sy, sz)

def cyl_collision_for_helice(link_elem, obj_for_aabb, density):
    cx, cy, cz, sx, sy, sz = aabb_local(obj_for_aabb)
    axis = principal_axis_from_extents(sx, sy, sz)
    m, ixx, iyy, izz, R, L = inertia_cylinder_axis(axis, sx, sy, sz, density)
    col = ET.SubElement(link_elem, "collision")
    if axis == 'X':
        rpy = (0.0, math.pi/2.0, 0.0)
        axis_vec = "1 0 0"
    elif axis == 'Y':
        rpy = (-math.pi/2.0, 0.0, 0.0)
        axis_vec = "0 1 0"
    else:
        rpy = (0.0, 0.0, 0.0)
        axis_vec = "0 0 1"
    urdf_origin(col, (cx, cy, cz), rpy)
    g = ET.SubElement(col, "geometry")
    c = ET.SubElement(g, "cylinder")
    c.set("radius", f"{R}")
    c.set("length", f"{L}")
    urdf_inertial(link_elem, m, ixx, iyy, izz, (cx, cy, cz))
    return axis_vec

# ============== xacro:arg e xacro:property ==============
def add_xacro_args(robot_elem):
    """Define apenas argumentos xacro "externos" (podem vir do launch)."""
    def add_arg(name, default):
        ET.SubElement(robot_elem, f"{{{XACRO_NS}}}arg",
                      {"name": name, "default": str(default)})

    # básicos que você pode querer sobrescrever pelo launch
    add_arg("namespace", MODEL_NAME)
    add_arg("ardupilot", "true")

def add_xacro_properties(robot_elem):
    """Define propriedades xacro usadas como variáveis em ${...}."""
    def prop(name, value):
        ET.SubElement(robot_elem, f"{{{XACRO_NS}}}property",
                      {"name": name, "value": str(value)})

    # sensores
    prop("imu_mass", 0.002)
    prop("mag_mass", 0.001)

    # ambiente / dinâmica
    prop("fluid_density", 1025.0)
    prop("fluid_level", 0.0001)
    prop("hull_radius", 10.6)
    prop("boat_width", 1.848)
    prop("boat_length", 2.3)
    prop("wave_model", 0)

    # referência GPS
    prop("ref_lat", -8.798638)
    prop("ref_lon", -63.952087)
    prop("ref_alt", 58)

    # escala S (usada na boiância)
    prop("S", 1.0)

# ============== Export principal ==============
def export_xacro(out_dir, density=DENSITY_DEFAULT):
    if not hasattr(bpy.ops.wm, "collada_export"):
        raise RuntimeError("Ative o Add-on 'Import-Export: Collada (Default)'.")

    meshes_dir = ensure_dir(os.path.join(out_dir, "meshes"))

    objs = visible_meshes()
    base = base_link_object(objs)
    if base is None:
        raise RuntimeError("Não encontrei 'base_link' VISÍVEL.")

    # cubos de colisão
    cubes = [o for o in objs if COL_RE.match(o.name)]

    # hélices: qualquer objeto cujo nome começa com "helice" (case-insensitive)
    helice_objs = [o for o in objs if o.name.lower().startswith("helice")]
    helice_objs = sorted(helice_objs, key=lambda o: o.name)

    # outros links visuais (exceto col# e hélices)
    other_objs = [o for o in objs if (o not in helice_objs and not COL_RE.match(o.name))]

    # Exporta malha única para hélices (reuso)
    helice_mesh_abs = None
    if helice_objs:
        helice_mesh_abs = os.path.join(meshes_dir, "helice.dae")
        export_object_at_origin(helice_objs[0], helice_mesh_abs, meshes_dir)

    # link_info: nome_canônico -> dict com objeto e flags
    link_info = {}

    # hélices com nomes canônicos Helice_1, Helice_2, ...
    for idx, obj in enumerate(helice_objs, start=1):
        link_name = f"Helice_{idx}"
        link_info[link_name] = {
            "obj": obj,
            "is_helice": True,
            "idx": idx,
            "axis_vec": None,
            "mesh_abs": helice_mesh_abs,
        }

    # demais links (nome = nome do objeto)
    for obj in other_objs:
        link_name = obj.name
        dae_abs = os.path.join(meshes_dir, f"{link_name}.dae")
        export_object_at_origin(obj, dae_abs, meshes_dir)
        link_info[link_name] = {
            "obj": obj,
            "is_helice": False,
            "idx": None,
            "axis_vec": None,
            "mesh_abs": dae_abs,
        }

    # ---------- URDF/XACRO ----------
    robot = ET.Element("robot", {"name": MODEL_NAME})

    # declara os xacro:arg e xacro:property usados no bloco EXTRA_XACRO
    add_xacro_args(robot)
    add_xacro_properties(robot)

    # Links + colisões
    for link_name, info in link_info.items():
        obj = info["obj"]
        is_helice = info["is_helice"]
        mesh_abs = info["mesh_abs"]

        link = make_link_visual(robot, link_name, mesh_abs)

        if link_name == "base_link":
            # colisões a partir de col0..N
            if cubes:
                xs, ys, zs = [], [], []
                for c in sorted(cubes, key=lambda x: x.name):
                    xyz, rpy, size = box_pose_size_in_parent(base, c)
                    col = ET.SubElement(link, "collision")
                    urdf_origin(col, xyz, rpy)
                    g = ET.SubElement(col, "geometry")
                    b = ET.SubElement(g, "box")
                    b.set("size", f"{size[0]} {size[1]} {size[2]}")
                    cx, cy, cz = xyz
                    sx, sy, sz = size
                    xs += [cx - sx/2.0, cx + sx/2.0]
                    ys += [cy - sy/2.0, cy + sy/2.0]
                    zs += [cz - sz/2.0, cz + sz/2.0]
                # inércia do envelope dos cubos
                sx = (max(xs) - min(xs))
                sy = (max(ys) - min(ys))
                sz = (max(zs) - min(zs))
                cx = (max(xs) + min(xs)) / 2.0
                cy = (max(ys) + min(ys)) / 2.0
                cz = (max(zs) + min(zs)) / 2.0
                m, ixx, iyy, izz = inertia_box(sx, sy, sz, density)
                urdf_inertial(link, m, ixx, iyy, izz, (cx, cy, cz))
            else:
                # fallback: AABB do base_link
                cx, cy, cz, sx, sy, sz = aabb_local(obj)
                col = ET.SubElement(link, "collision")
                urdf_origin(col, (cx, cy, cz), (0, 0, 0))
                g = ET.SubElement(col, "geometry")
                b = ET.SubElement(g, "box")
                b.set("size", f"{sx} {sy} {sz}")
                m, ixx, iyy, izz = inertia_box(sx, sy, sz, density)
                urdf_inertial(link, m, ixx, iyy, izz, (cx, cy, cz))

        elif is_helice:
            # colisão: cilindro pelo AABB; guarda eixo para a junta
            axis_vec = cyl_collision_for_helice(link, obj, density)
            info["axis_vec"] = axis_vec

        else:
            cx, cy, cz, sx, sy, sz = aabb_local(obj)
            col = ET.SubElement(link, "collision")
            urdf_origin(col, (cx, cy, cz), (0, 0, 0))
            g = ET.SubElement(col, "geometry")
            b = ET.SubElement(g, "box")
            b.set("size", f"{sx} {sy} {sz}")
            m, ixx, iyy, izz = inertia_box(sx, sy, sz, density)
            urdf_inertial(link, m, ixx, iyy, izz, (cx, cy, cz))

    # Juntas (base_link -> demais)
    Tbase = base.matrix_world.copy()
    Tbinv = Tbase.copy()
    Tbinv.invert()

    for link_name, info in link_info.items():
        if link_name == "base_link":
            continue

        obj = info["obj"]
        is_helice = info["is_helice"]
        idx = info["idx"]
        axis_vec = info["axis_vec"]

        Trel = Tbinv @ obj.matrix_world
        t = Trel.to_translation()
        rpy = Trel.to_euler('XYZ')

        if is_helice and idx is not None:
            joint_name = f"Engine_helice_{idx}"
            jtype = "continuous"
        else:
            joint_name = f"{link_name}_joint"
            jtype = "fixed"

        j = ET.SubElement(robot, "joint", {"name": joint_name, "type": jtype})
        ET.SubElement(j, "parent").set("link", "base_link")
        ET.SubElement(j, "child").set("link", link_name)
        urdf_origin(j, (t.x, t.y, t.z), (rpy.x, rpy.y, rpy.z))
        if axis_vec:
            ax = ET.SubElement(j, "axis")
            ax.set("xyz", axis_vec)
            dyn = ET.SubElement(j, "dynamics")
            dyn.set("damping", "0.05")
            dyn.set("friction", "0.05")

    # Anexa o bloco xacro/sensores/dinâmica/ArduPilot
    append_extra_xacro(robot)

    # Salva XACRO
    xacro_path = os.path.join(out_dir, XACRO_FILENAME)
    with open(xacro_path, "w") as f:
        f.write(pretty_xml(robot))
    print(f"[ok] salvo: {xacro_path}")
    print(f"[ok] malhas: {meshes_dir}")

# ============== Operador Blender ==============
class OT_ExportMigbotXacro(Operator, ExportHelper):
    bl_idname = "export_scene.migbot2_xacro"
    bl_label = "Export Xacro (migbot2_ardupilot)"
    bl_options = {'REGISTER', 'UNDO'}
    filename_ext = ".xacro"
    filter_glob: StringProperty(default="*.xacro", options={'HIDDEN'})
    density: FloatProperty(name="Densidade (kg/m³)", default=DENSITY_DEFAULT,
                           min=10.0, max=50000.0)
    axis_fix: BoolProperty(name="Visual axis fix (-90° X)",
                           default=VISUAL_AXIS_FIX_FOR_COLLADA)

    def execute(self, context):
        global VISUAL_AXIS_FIX_FOR_COLLADA
        VISUAL_AXIS_FIX_FOR_COLLADA = bool(self.axis_fix)
        if not hasattr(bpy.ops.wm, "collada_export"):
            self.report({'ERROR'}, "Ative o Add-on 'Import-Export: Collada (Default)'.")
            return {'CANCELLED'}
        out_dir = os.path.dirname(self.filepath)
        if not os.path.isdir(out_dir):
            self.report({'ERROR'}, f"Diretório inválido: {out_dir}")
            return {'CANCELLED'}
        try:
            export_xacro(out_dir, density=self.density)
        except Exception as e:
            self.report({'ERROR'}, f"Falha: {e}")
            return {'CANCELLED'}
        self.report({'INFO'}, f"Export concluído em: {out_dir}")
        return {'FINISHED'}

def menu_func(self, context):
    self.layout.operator(OT_ExportMigbotXacro.bl_idname,
                         text="Export Xacro (migbot2_ardupilot)")

def register():
    bpy.utils.register_class(OT_ExportMigbotXacro)
    bpy.types.TOPBAR_MT_file_export.append(menu_func)
    print("[XACRO] File > Export > Export Xacro (migbot2_ardupilot)")

def unregister():
    bpy.types.TOPBAR_MT_file_export.remove(menu_func)
    bpy.utils.unregister_class(OT_ExportMigbotXacro)

if __name__ == "__main__":
    register()
    bpy.ops.export_scene.migbot2_xacro('INVOKE_DEFAULT')

