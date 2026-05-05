#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Script Blender -> Gazebo (SAE_HYDRO_V0)

- Exporta um DAE para cada objeto MESH da cena (ou de uma collection específica)
- Gera model_raw.sdf com um link por objeto
- Gera model.sdf injetando colisões simples:
    * caixas para grid0*, bigbuoygrid*, smallbuoygrid*
    * cilindro para maincolumn
    * colisão por mesh para: trunk, full_dam, vegetation, riverbed, ref_circle_1..4
- Converte materiais "só cor" em texturas PNG pequenas (sem modificar o material original)
- Gera model.config
"""

import bpy
import os
import xml.etree.ElementTree as ET
from copy import deepcopy

# ============================
# CONFIGURAÇÃO BÁSICA
# ============================

MODEL_NAME = "SAE_HYDRO_V0"

# Se quiser exportar só uma collection, coloque o nome aqui.
# Se deixar em "", exporta todos os objetos MESH da cena.
EXPORT_COLLECTION_NAME = ""  # ex: "SAE_HYDRO"

HOME = os.path.expanduser("~")
BASE_DIR = os.path.join(HOME, ".gazebo", "models", MODEL_NAME)
MESH_DIR = os.path.join(BASE_DIR, "meshes")
TEXTURE_DIR = MESH_DIR  # onde salvamos texturas planas

RAW_SDF = os.path.join(BASE_DIR, "model_raw.sdf")
FINAL_SDF = os.path.join(BASE_DIR, "model.sdf")
MODEL_CONFIG = os.path.join(BASE_DIR, "model.config")

# ============================
# TEMPLATES DE COLISÃO
# ============================

BOX_TEMPLATES = {
    "grid0": {
        "pose": "-0.012944 -0.014759 1.243475 0 0 1.242577",
        "size": "7.826429 0.328338 2.486950",
        "name_suffix": "basebox",
    },
    "bigbuoygrid": {
        "pose": "-1.857040 3.140529 2.596603 0 0 2.148923",
        "size": "14.789584 3.188850 5.200012",
        "name_suffix": "basebox",
    },
    "smallbuoygrid": {
        "pose": "0.002917 -0.005491 2.073040 0 0 2.132364",
        "size": "7.501942 2.757460 4.146080",
        "name_suffix": "basebox",
    },
}

CYL_TEMPLATES = {
    "maincolumn": {
        "pose": "0 0 3.729830 0 0.285020 -0.595138",
        "radius": "3.007000",
        "length": "6.014022",
        "name_suffix": "basecyl",
    }
}

MESH_COLLISION_LINKS = [
    "trunk",
    "full_dam",
    "vegetation",
    "riverbed",
    "ref_circle_1",
    "ref_circle_2",
    "ref_circle_3",
    "ref_circle_4",
]

# ============================
# TEXTURAS PLANAS PARA CORES
# ============================

_flat_color_cache = {}  # chave: (r,g,b,a) arredondado -> bpy.types.Image


def _color_key(col):
    r, g, b, a = col
    return (round(r, 4), round(g, 4), round(b, 4), round(a, 4))


def ensure_flat_color_image(color, base_name="flat"):
    """
    Cria (ou reutiliza) uma imagem pequena PNG com a cor RGBA dada.
    NÃO mexe em nenhum material, só gera o arquivo.
    """
    os.makedirs(TEXTURE_DIR, exist_ok=True)

    key = _color_key(color)
    if key in _flat_color_cache:
        return _flat_color_cache[key]

    # clampa 0..1
    r = max(0.0, min(1.0, color[0]))
    g = max(0.0, min(1.0, color[1]))
    b = max(0.0, min(1.0, color[2]))
    a = max(0.0, min(1.0, color[3]))

    # nome baseado na cor
    R = int(r * 255)
    G = int(g * 255)
    B = int(b * 255)
    image_name = f"{base_name}_{R:02x}{G:02x}{B:02x}.png"
    image_path = os.path.join(TEXTURE_DIR, image_name)

    # reaproveita se já existir em bpy.data.images
    img = bpy.data.images.get(image_name)
    if img is None:
        img = bpy.data.images.new(image_name, width=4, height=4)
        pixels = [r, g, b, a] * (4 * 4)
        img.pixels = pixels
        img.filepath_raw = image_path
        img.file_format = 'PNG'
        img.save()
    else:
        img.filepath_raw = image_path

    _flat_color_cache[key] = img
    return img


def ensure_mesh_uv(mesh):
    """
    Gera um UVMap trivial se não existir nenhum; todos vértices em (0.5,0.5).
    (Isso não mexe no material, só evita preto quando se usa textura.)
    """
    if not hasattr(mesh, "uv_layers"):
        return

    if mesh.uv_layers:
        return

    uv_layer = mesh.uv_layers.new(name="UVMap")
    for loop in uv_layer.data:
        loop.uv = (0.5, 0.5)


def material_has_image_texture(mat):
    """Retorna True se o material já usa alguma TEX_IMAGE."""
    if not mat.use_nodes or mat.node_tree is None:
        return False
    for n in mat.node_tree.nodes:
        if n.type == 'TEX_IMAGE' and getattr(n, "image", None) is not None:
            return True
    return False


def setup_material_flatcolor(mat_copy, mat_source=None):
    """
    Configura o material *cópia* para usar uma textura plana baseada
    na cor original. NÃO toca no material original.
    Retorna True se criou uma textura nova e conectou.
    """
    if mat_source is None:
        mat_source = mat_copy

    # Se já tem textura de imagem, não mexe
    if material_has_image_texture(mat_copy):
        return False

    # Cor base original
    base_color = None

    if mat_source.use_nodes and mat_source.node_tree is not None:
        principled_src = None
        for n in mat_source.node_tree.nodes:
            if n.type == 'BSDF_PRINCIPLED':
                principled_src = n
                break
        if principled_src is not None:
            base_color = principled_src.inputs['Base Color'].default_value[:]

    if base_color is None:
        # fallback: diffuse_color
        base_color = mat_source.diffuse_color[:]
        if len(base_color) == 3:
            base_color = (base_color[0], base_color[1], base_color[2], 1.0)

    # Garante nodes no material-cópia
    mat_copy.use_nodes = True
    nt = mat_copy.node_tree
    nt.nodes.clear()
    nodes = nt.nodes
    links = nt.links

    # Output
    output = nodes.new("ShaderNodeOutputMaterial")
    output.is_active_output = True
    output.location = (200, 0)

    # Principled
    principled = nodes.new("ShaderNodeBsdfPrincipled")
    principled.location = (0, 0)
    principled.inputs['Base Color'].default_value = base_color
    links.new(principled.outputs['BSDF'], output.inputs['Surface'])

    # Textura plana
    img = ensure_flat_color_image(base_color, base_name=mat_copy.name)

    tex_node = nodes.new("ShaderNodeTexImage")
    tex_node.image = img
    tex_node.location = (-300, 0)
    links.new(tex_node.outputs['Color'], principled.inputs['Base Color'])

    return True


# ============================
# SELEÇÃO E EXPORT
# ============================

def get_export_objects():
    """Retorna a lista de objetos MESH a exportar."""
    scene = bpy.context.scene
    if EXPORT_COLLECTION_NAME:
        col = bpy.data.collections.get(EXPORT_COLLECTION_NAME)
        if col is None:
            print(f"[SAE_HYDRO] Collection '{EXPORT_COLLECTION_NAME}' não encontrada. Exportando cena inteira.")
            return [obj for obj in scene.objects if obj.type == 'MESH']
        return [obj for obj in col.objects if obj.type == 'MESH']
    else:
        return [obj for obj in scene.objects if obj.type == 'MESH']


def export_meshes_to_dae(objs):
    """Exporta um DAE para cada objeto em objs, dentro de MESH_DIR, sem alterar a cena."""
    if not objs:
        print("[SAE_HYDRO] Nenhum objeto MESH encontrado para exportar!")
        return

    os.makedirs(MESH_DIR, exist_ok=True)

    prev_active = bpy.context.view_layer.objects.active
    prev_selected = [o for o in bpy.context.selected_objects]

    for obj in objs:
        if obj.type != 'MESH':
            continue
        if obj.name.startswith("Camera") or obj.name.startswith("Light"):
            continue

        dae_path = os.path.join(MESH_DIR, f"{obj.name}.dae")
        print(f"[SAE_HYDRO] Exportando {obj.name} -> {dae_path}")

        # ---- prepara materiais temporários ----
        backup = []
        created_any_texture = False

        for idx, slot in enumerate(obj.material_slots):
            mat_orig = slot.material
            if mat_orig is None:
                continue

            # cria cópia temporária
            mat_temp = mat_orig.copy()
            created = setup_material_flatcolor(mat_temp, mat_source=mat_orig)
            created_any_texture = created_any_texture or created

            slot.material = mat_temp
            backup.append((idx, mat_orig, mat_temp))

        if created_any_texture and obj.data is not None:
            ensure_mesh_uv(obj.data)

        # Seleciona só o objeto atual
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj

        # Exporta COLLADA só do selecionado
        bpy.ops.wm.collada_export(
            filepath=dae_path,
            check_existing=False,
            selected=True,
            apply_modifiers=True,
            include_children=False,
            include_armatures=False,
            include_animations=False,
        )

        # ---- restaura materiais originais e remove cópias ----
        for idx, mat_orig, mat_temp in backup:
            obj.material_slots[idx].material = mat_orig
            if mat_temp is not None and mat_temp.name in bpy.data.materials:
                try:
                    bpy.data.materials.remove(mat_temp, do_unlink=True)
                except TypeError:
                    # compatibilidade com versões onde não existe do_unlink
                    bpy.data.materials.remove(mat_temp)

    # restaura seleção original
    bpy.ops.object.select_all(action='DESELECT')
    for o in prev_selected:
        o.select_set(True)
    bpy.context.view_layer.objects.active = prev_active

    print("[SAE_HYDRO] Export de meshes concluído (sem alterar a cena).")


# ============================
# GERAÇÃO DO model_raw.sdf
# ============================

def generate_raw_sdf(objs, out_path):
    """Cria model_raw.sdf com um link por objeto exportado."""
    print(f"[SAE_HYDRO] Gerando SDF bruto em: {out_path}")

    sdf = ET.Element("sdf", {"version": "1.6"})
    model = ET.SubElement(sdf, "model", {"name": MODEL_NAME})

    static_el = ET.SubElement(model, "static")
    static_el.text = "true"

    pose_model = ET.SubElement(model, "pose")
    pose_model.text = "0 0 0 0 0 0"

    for obj in objs:
        if obj.type != 'MESH':
            continue
        link_name = obj.name

        link_el = ET.SubElement(model, "link", {"name": link_name})

        visual_el = ET.SubElement(link_el, "visual", {"name": link_name + "_visual"})

        pose_v = ET.SubElement(visual_el, "pose")
        pose_v.text = "0 0 0 0 0 0"

        geom_v = ET.SubElement(visual_el, "geometry")
        mesh_v = ET.SubElement(geom_v, "mesh")
        uri_v = ET.SubElement(mesh_v, "uri")
        uri_v.text = f"model://{MODEL_NAME}/meshes/{link_name}.dae"
        scale_v = ET.SubElement(mesh_v, "scale")
        scale_v.text = "1 1 1"

        # --- PHONG no SDF ---
        material_el = ET.SubElement(visual_el, "material")
        ET.SubElement(material_el, "shader", {"type": "pixel"})

    tree = ET.ElementTree(sdf)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    tree.write(out_path, encoding="utf-8", xml_declaration=True)
    print("[SAE_HYDRO] model_raw.sdf gerado.")


# ============================
# FUNÇÕES DE INJEÇÃO DE COLISÃO
# ============================

def add_box_collision(link, tpl_key):
    tpl = BOX_TEMPLATES[tpl_key]
    coll_name = f"{link.attrib['name']}_{tpl['name_suffix']}"
    coll_el = ET.SubElement(link, "collision", {"name": coll_name})

    pose_el = ET.SubElement(coll_el, "pose")
    pose_el.text = tpl["pose"]

    geom_el = ET.SubElement(coll_el, "geometry")
    box_el = ET.SubElement(geom_el, "box")
    size_el = ET.SubElement(box_el, "size")
    size_el.text = tpl["size"]

    surface_el = ET.SubElement(coll_el, "surface")
    contact_el = ET.SubElement(surface_el, "contact")
    mask_el = ET.SubElement(contact_el, "collide_bitmask")
    mask_el.text = "0x01"


def add_cylinder_collision(link, tpl_key):
    tpl = CYL_TEMPLATES[tpl_key]
    coll_name = f"{link.attrib['name']}_{tpl['name_suffix']}"
    coll_el = ET.SubElement(link, "collision", {"name": coll_name})

    pose_el = ET.SubElement(coll_el, "pose")
    pose_el.text = tpl["pose"]

    geom_el = ET.SubElement(coll_el, "geometry")
    cyl_el = ET.SubElement(geom_el, "cylinder")
    r_el = ET.SubElement(cyl_el, "radius")
    r_el.text = tpl["radius"]
    l_el = ET.SubElement(cyl_el, "length")
    l_el.text = tpl["length"]

    surface_el = ET.SubElement(coll_el, "surface")
    contact_el = ET.SubElement(surface_el, "contact")
    mask_el = ET.SubElement(contact_el, "collide_bitmask")
    mask_el.text = "0x01"


def add_mesh_collision_from_visual(link):
    visual = link.find("visual")
    if visual is None:
        return
    geom = visual.find("geometry")
    if geom is None:
        return

    coll_el = ET.SubElement(link, "collision", {"name": link.attrib["name"] + "_collision"})
    new_geom = ET.SubElement(coll_el, "geometry")
    for child in geom:
        new_geom.append(deepcopy(child))


def inject_collisions(in_path, out_path):
    print(f"[SAE_HYDRO] Injetando colisões em: {in_path}")
    tree = ET.parse(in_path)
    root = tree.getroot()

    for link in root.findall(".//link"):
        name = link.attrib.get("name", "")

        if link.find("collision") is not None:
            continue

        if name.startswith("grid0"):
            add_box_collision(link, "grid0")
            continue

        if name.startswith("bigbuoygrid"):
            add_box_collision(link, "bigbuoygrid")
            continue

        if name.startswith("smallbuoygrid"):
            add_box_collision(link, "smallbuoygrid")
            continue

        if name == "maincolumn":
            add_cylinder_collision(link, "maincolumn")
            continue

        if name in MESH_COLLISION_LINKS:
            add_mesh_collision_from_visual(link)
            continue

    tree.write(out_path, encoding="utf-8", xml_declaration=True)
    print(f"[SAE_HYDRO] model.sdf com colisões gerado em: {out_path}")


# ============================
# model.config
# ============================

def write_model_config():
    os.makedirs(BASE_DIR, exist_ok=True)
    with open(MODEL_CONFIG, "w", encoding="utf-8") as f:
        f.write('<?xml version="1.0"?>\n')
        f.write('<model>\n')
        f.write(f'  <name>{MODEL_NAME}</name>\n')
        f.write('  <version>1.0</version>\n')
        f.write('  <sdf version="1.6">model.sdf</sdf>\n')
        f.write('  <author>\n')
        f.write('    <name>Auto-generated by Blender exporter</name>\n')
        f.write('    <email></email>\n')
        f.write('  </author>\n')
        f.write('  <description>SAE_HYDRO_V0 environment exported from Blender.</description>\n')
        f.write('</model>\n')
    print(f"[SAE_HYDRO] model.config gerado em: {MODEL_CONFIG}")


# ============================
# FUNÇÃO PRINCIPAL
# ============================

def exportar_sae_hydro():
    print("[SAE_HYDRO] Iniciando export Blender -> Gazebo...")
    os.makedirs(BASE_DIR, exist_ok=True)
    os.makedirs(MESH_DIR, exist_ok=True)

    objs = get_export_objects()
    print(f"[SAE_HYDRO] Objetos MESH a exportar: {[o.name for o in objs]}")

    export_meshes_to_dae(objs)
    generate_raw_sdf(objs, RAW_SDF)
    inject_collisions(RAW_SDF, FINAL_SDF)
    write_model_config()

    print("[SAE_HYDRO] Exportação completa! Verifique ~/.gazebo/models/SAE_HYDRO_V0")


if __name__ == "__main__":
    exportar_sae_hydro()

