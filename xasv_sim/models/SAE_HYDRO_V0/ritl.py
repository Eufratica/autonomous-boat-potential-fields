#!/usr/bin/env python3
import time
import math
import random
import os
import xml.etree.ElementTree as ET

# ===== CONFIG =====

# Perfil de tempo/escala
PROFILE_MODE     = "linear_up"
# opções: "oscillating", "linear_up", "linear_down", "triangle", "saw", "noise"
PROFILE_EXP      = 1.0    # 1.0 = direto; >1 puxa pra baixo; <1 puxa pra cima

CYCLE_SECONDS    = 6.0    # período dos perfis oscilantes/saw/triangle
NOISE_SMOOTHING  = 0.3    # 0..1 quanto o "noise" reage ao novo valor

# Amplitude de escala
# escala final = 1.0 +/- SCALE_AMPLITUDE
SCALE_AMPLITUDE  = 0.5    # 0.5 => escala vai de 0.5 até 1.5

UPDATE_INTERVAL  = 0.5    # segundos entre iterações do loop
EXPORT_INTERVAL  = 2.0    # segundos entre exports DAE
RUN_DURATION     = 10.0   # None = roda indefinido; ex.: 20.0 para parar

# Caminhos (baseado na pasta deste script)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ORIGINAL_DAE_PATH = os.path.join(BASE_DIR, "meshes", "trunk.dae")
DAE_EXPORT_PATH   = os.path.join(BASE_DIR, "meshes", "trunk_runtime.dae")

_state = {
    "start_time": None,
    "last_export_time": None,
    "last_value_01": 0.5,
}


# ===== PERFIL: gera value_01 em [0,1] conforme o tempo =====
def calcular_value_01(elapsed_total):
    """Retorna value_01 em [0,1] conforme PROFILE_MODE."""
    if PROFILE_MODE == "oscillating":
        phase = 2.0 * math.pi * (elapsed_total % CYCLE_SECONDS) / CYCLE_SECONDS
        v = 0.5 + 0.5 * math.sin(phase)

    elif PROFILE_MODE == "linear_up":
        dur = RUN_DURATION if RUN_DURATION is not None else CYCLE_SECONDS
        if dur <= 0:
            dur = 1.0
        v = min(elapsed_total / dur, 1.0)

    elif PROFILE_MODE == "linear_down":
        dur = RUN_DURATION if RUN_DURATION is not None else CYCLE_SECONDS
        if dur <= 0:
            dur = 1.0
        v = max(1.0 - elapsed_total / dur, 0.0)

    elif PROFILE_MODE == "triangle":
        p = (elapsed_total % CYCLE_SECONDS) / CYCLE_SECONDS
        if p < 0.5:
            v = 2.0 * p
        else:
            v = 2.0 * (1.0 - p)

    elif PROFILE_MODE == "saw":
        v = (elapsed_total % CYCLE_SECONDS) / CYCLE_SECONDS

    elif PROFILE_MODE == "noise":
        prev = _state.get("last_value_01", 0.5)
        target = random.random()
        alpha = NOISE_SMOOTHING
        v = (1.0 - alpha) * prev + alpha * target

    else:
        v = 0.5

    v = max(0.0, min(1.0, v))
    if PROFILE_EXP != 1.0 and PROFILE_EXP > 0.0:
        v = v ** PROFILE_EXP

    _state["last_value_01"] = v
    return v


# ===== ESCALA UNIFORME =====
def value_to_scale(value_01):
    """
    value_01 em [0,1] -> escala em torno de 1.0.
    0 => 1 - SCALE_AMPLITUDE
    1 => 1 + SCALE_AMPLITUDE
    """
    s = (1.0 - SCALE_AMPLITUDE) + value_01 * (2.0 * SCALE_AMPLITUDE)
    return s


# ===== APLICAR ESCALA NO .DAE =====
def scale_positions_in_dae(in_path, out_path, scale):
    """
    Lê um .dae, encontra arrays de posição e multiplica (x,y,z) por 'scale'.
    Salva em out_path.
    """
    if not os.path.isfile(in_path):
        print(f"[ERRO] Arquivo DAE de entrada não encontrado: {in_path}")
        return

    print(f"[EXPORT] Lendo DAE base: {in_path}")
    tree = ET.parse(in_path)
    root = tree.getroot()

    # Collada usa namespace; vamos tratar tags por 'endswith'
    def tag_endswith(elem, name):
        return elem.tag.endswith(name)

    count_scaled_arrays = 0
    count_scaled_verts = 0

    # Estratégia simples:
    #  - acha <source> cujo id contenha 'position' ou 'positions'
    #  - dentro dele, pega <float_array> e escala a cada tripla (x,y,z)
    for source in root.iter():
        if not tag_endswith(source, "source"):
            continue
        sid = source.get("id", "").lower()
        if "position" not in sid:
            continue

        float_array = None
        for child in source:
            if tag_endswith(child, "float_array"):
                float_array = child
                break

        if float_array is None or float_array.text is None:
            continue

        try:
            nums = [float(x) for x in float_array.text.split()]
        except ValueError:
            continue

        # escala a cada tripla
        for i in range(0, len(nums), 3):
            nums[i]     *= scale
            if i + 1 < len(nums):
                nums[i+1] *= scale
            if i + 2 < len(nums):
                nums[i+2] *= scale

        float_array.text = " ".join(f"{x:.6f}" for x in nums)
        count_scaled_arrays += 1
        count_scaled_verts  += len(nums) // 3

    print(f"[EXPORT] Escalou {count_scaled_arrays} arrays de posição, {count_scaled_verts} vértices.")

    # grava saída
    tree.write(out_path, encoding="utf-8", xml_declaration=True)
    print(f"[EXPORT] DAE escalado salvo em: {out_path}")


# ===== LOOP PRINCIPAL =====
def main():
    print(f"[INFO] DAE base:    {ORIGINAL_DAE_PATH}")
    print(f"[INFO] DAE runtime: {DAE_EXPORT_PATH}")
    print(f"[INFO] PROFILE_MODE={PROFILE_MODE}, RUN_DURATION={RUN_DURATION}")

    start = time.perf_counter()
    _state["start_time"] = start
    _state["last_export_time"] = start

    while True:
        now = time.perf_counter()
        elapsed_total = now - start

        # fim de execução
        if RUN_DURATION is not None and elapsed_total >= RUN_DURATION:
            print("[MAIN] Tempo limite atingido, encerrando.")
            break

        value_01 = calcular_value_01(elapsed_total)
        scale = value_to_scale(value_01)
        print(f"[TIMER] mode={PROFILE_MODE}, t={elapsed_total:.2f}s, value_01={value_01:.3f}, scale={scale:.3f}")

        # export periódico
        elapsed_since_export = now - _state["last_export_time"]
        if elapsed_since_export >= EXPORT_INTERVAL:
            scale_positions_in_dae(ORIGINAL_DAE_PATH, DAE_EXPORT_PATH, scale)
            _state["last_export_time"] = now

        time.sleep(UPDATE_INTERVAL)


if __name__ == "__main__":
    main()

