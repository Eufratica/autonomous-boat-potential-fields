#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rosgps2mission.py

Ler um rosbag com mensagens de GPS e gerar um arquivo de missão
para ArduPilot em formato .waypoints (QGC WPL 110) ou .plan (QGC JSON).

Exemplos de uso:

  # 20 pontos, saída .waypoints
  ./rosgps2mission.py \
      --bag traj.bag \
      --topic /mavros/global_position/global \
      --points 20 \
      --format waypoint \
      --output mission.waypoints \
      --alt 20

  # 50 pontos, saída .plan
  ./rosgps2mission.py \
      --bag traj.bag \
      --topic /mavros/global_position/global \
      --points 50 \
      --format plan \
      --output mission.plan
"""

import argparse
import json
import sys

try:
    import rosbag
except ImportError:
    print("Erro: não foi possível importar rosbag. "
          "Certifique-se de estar em um ambiente ROS (ex: source /opt/ros/noetic/setup.bash).",
          file=sys.stderr)
    sys.exit(1)


def extract_gps_points(bag_path, topic):
    """
    Lê o rosbag e extrai (lat, lon, alt) da mensagem indicada.
    Suporta mensagens com campos (latitude, longitude, altitude)
    ou (lat, lon, alt).
    """
    points = []
    bag = rosbag.Bag(bag_path)
    try:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            lat = lon = alt = None

            # Tentativa 1: sensor_msgs/NavSatFix (latitude, longitude, altitude)
            if hasattr(msg, "latitude") and hasattr(msg, "longitude"):
                lat = float(msg.latitude)
                lon = float(msg.longitude)
                alt = float(getattr(msg, "altitude", 0.0))

            # Tentativa 2: mensagens com lat/lon/alt
            elif hasattr(msg, "lat") and hasattr(msg, "lon"):
                lat = float(msg.lat)
                lon = float(msg.lon)
                alt = float(getattr(msg, "alt", 0.0))

            if lat is not None and lon is not None:
                points.append((lat, lon, alt if alt is not None else 0.0))
    finally:
        bag.close()

    if not points:
        raise RuntimeError(
            f"Nenhum ponto GPS encontrado em {bag_path} no tópico {topic}"
        )

    return points


def select_n_points(points, n):
    """
    Seleciona N pontos distribuídos ao longo da trajetória.
    Se houver menos pontos que N, retorna todos.
    """
    if n >= len(points):
        return points

    if n <= 1:
        # só o primeiro
        return [points[0]]

    selected = []
    last_index = len(points) - 1
    for i in range(n):
        idx = round(i * last_index / (n - 1))
        selected.append(points[idx])
    return selected


def write_waypoints_file(points, output_path, alt_override=None):
    """
    Escreve arquivo texto QGC WPL 110 (.waypoints) com NAV_WAYPOINT (cmd 16).
    Formato: QGC WPL 110 + linhas com TAB, conforme MAVLink docs.
    """
    version = "110"
    coord_frame = 3           # MAV_FRAME_GLOBAL_RELATIVE_ALT
    command = 16              # MAV_CMD_NAV_WAYPOINT
    param1 = param2 = param3 = param4 = 0.0
    autocontinue = 1

    with open(output_path, "w", encoding="utf-8") as f:
        f.write(f"QGC WPL {version}\n")
        for idx, (lat, lon, alt) in enumerate(points):
            alt_use = float(alt_override) if alt_override is not None else float(alt)
            current_wp = 1 if idx == 0 else 0
            line = (
                f"{idx}\t"
                f"{current_wp}\t"
                f"{coord_frame}\t"
                f"{command}\t"
                f"{param1}\t"
                f"{param2}\t"
                f"{param3}\t"
                f"{param4}\t"
                f"{lat:.8f}\t"
                f"{lon:.8f}\t"
                f"{alt_use:.2f}\t"
                f"{autocontinue}\n"
            )
            f.write(line)


def write_plan_file(points, output_path, alt_override=None):
    """
    Escreve arquivo JSON .plan (QGroundControl).
    Cria itens simples NAV_WAYPOINT (cmd 16) com frame GLOBAL_RELATIVE_ALT.
    """
    if not points:
        raise RuntimeError("Lista de pontos vazia para escrever .plan.")

    items = []
    for i, (lat, lon, alt) in enumerate(points, start=1):
        alt_use = float(alt_override) if alt_override is not None else float(alt)
        item = {
            "AMSLAltAboveTerrain": None,
            "Altitude": alt_use,
            # 1 = altitude relativa ao home (GLOBAL_RELATIVE_ALT)
            "AltitudeMode": 1,
            "autoContinue": True,
            "command": 16,  # MAV_CMD_NAV_WAYPOINT
            "doJumpId": i,
            "frame": 3,     # MAV_FRAME_GLOBAL_RELATIVE_ALT
            "params": [
                0,          # param1 (hold)
                0,          # param2 (aceitação)
                0,          # param3 (raio)
                None,       # param4 (heading)
                lat,        # x
                lon,        # y
                alt_use     # z
            ],
            "type": "SimpleItem",
        }
        items.append(item)

    first_lat, first_lon, first_alt = points[0]
    home_alt = float(alt_override) if alt_override is not None else float(first_alt)

    plan = {
        "fileType": "Plan",
        "geoFence": {
            "circles": [],
            "polygons": [],
            "version": 2
        },
        "groundStation": "QGroundControl",
        "mission": {
            "cruiseSpeed": 1,
            # 3 = MAV_AUTOPILOT_ARDUPILOTMEGA
            "firmwareType": 3,
            # 1 = altitudes relativas por padrão
            "globalPlanAltitudeMode": 1,
            "hoverSpeed": 1,
            "items": items,
            # [lat, lon, alt] do home planejado
            "plannedHomePosition": [first_lat, first_lon, home_alt],
            # 10 = MAV_TYPE_GROUND_ROVER, 11 = BOAT; ajuste se quiser
            "vehicleType": 10,
            "version": 2
        },
        "rallyPoints": {
            "points": [],
            "version": 2
        },
        "version": 1
    }

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(plan, f, indent=2, sort_keys=False)


def main():
    parser = argparse.ArgumentParser(
        description="Converter GPS de um rosbag em arquivo de missão ArduPilot."
    )
    parser.add_argument(
        "--bag", required=True,
        help="Caminho do arquivo .bag"
    )
    parser.add_argument(
        "--topic", default="/mavros/global_position/global",
        help="Tópico de GPS (default: /mavros/global_position/global)"
    )
    parser.add_argument(
        "--points", "-n", type=int, required=True,
        help="Número de pontos (waypoints) desejados"
    )
    parser.add_argument(
        "--format", "-f", choices=["waypoint", "plan"], required=True,
        help="Formato de saída: 'waypoint' (QGC WPL 110) ou 'plan' (QGC .plan JSON)"
    )
    parser.add_argument(
        "--output", "-o", required=True,
        help="Arquivo de saída (ex: mission.waypoints ou mission.plan)"
    )
    parser.add_argument(
        "--alt", type=float, default=None,
        help="Altitude fixa (m). Se não informar, usa altitude do GPS."
    )

    args = parser.parse_args()

    try:
        points_all = extract_gps_points(args.bag, args.topic)
    except Exception as e:
        print(f"Erro ao ler rosbag: {e}", file=sys.stderr)
        sys.exit(1)

    if args.points <= 0:
        print("Erro: --points deve ser > 0", file=sys.stderr)
        sys.exit(1)

    points_sel = select_n_points(points_all, args.points)

    if args.format == "waypoint":
        write_waypoints_file(points_sel, args.output, alt_override=args.alt)
    else:
        write_plan_file(points_sel, args.output, alt_override=args.alt)

    print(f"Missão gerada com {len(points_sel)} pontos em: {args.output}")


if __name__ == "__main__":
    main()

