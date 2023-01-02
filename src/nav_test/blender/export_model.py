import bpy
import os
import sys
from xml.dom import minidom
from xml.etree import ElementTree

# Features
#   - Handle models with multiple materials 

if "--" in sys.argv:
    argv = sys.argv[sys.argv.index("--") + 1:]
else:
    argv = []

print(f"argv {argv}")
if len(argv) > 0:
    SEED = int(argv[0])
else:
    SEED = 0
print(f"Using seed: {SEED}")

MODEL_NAME = "Field"
PATH_MODEL = bpy.path.abspath(f"//../models/{MODEL_NAME}")
PATH_COLLISION = os.path.join(PATH_MODEL, 'assets/collision.obj')
PATH_VISUAL = os.path.join(PATH_MODEL, 'assets/visual.obj')
PATH_SDF = os.path.join(PATH_MODEL, 'model.sdf')
PATH_CONFIG = os.path.join(PATH_MODEL, 'model.config')
SDF_VERSION = '1.9'

# select the model we want to export by name
bpy.ops.object.select_all(action='DESELECT')
bpy.data.objects[MODEL_NAME].select_set(True)
obj = bpy.data.objects[MODEL_NAME]

def duplicate(obj, add_to_scene=True):
    obj_copy = obj.copy()
    obj_copy.data = obj_copy.data.copy()
    if add_to_scene:
        bpy.context.scene.collection.objects.link(obj_copy)
    return obj_copy

def apply_modifiers(obj, is_collider: bool):
    ctx = bpy.context.copy()
    ctx['object'] = obj
    for modifier in obj.modifiers:
        ctx['modifier'] = modifier
        for input in modifier.node_group.inputs:
            if input.name == "is_exporting":
                modifier[input.identifier] = True
            if input.name == "is_collider":
                modifier[input.identifier] = is_collider
            if input.name == "seed":
                modifier[input.identifier] = SEED
        bpy.ops.object.modifier_apply(ctx, modifier=modifier.name)
            
    for i in reversed(range(len(obj.data.attributes))):
        attr = obj.data.attributes[i]
        obj.data.attributes.active_index = i
        
        if attr.name == "UVMap":
            bpy.ops.geometry.attribute_convert(ctx, mode='UV_MAP')

    for m in obj.modifiers:
        obj.modifiers.remove(m)
        
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)

def write_xml(xml: ElementTree.Element, filepath: str):
    xml_string = minidom.parseString(
        ElementTree.tostring(xml, encoding="unicode")
    ).toprettyxml(indent="  ")
    file = open(filepath, "w")
    file.write(xml_string)
    file.close()
    print(f"Saved: {filepath}")

def generate_sdf():
    sdf = ElementTree.Element("sdf", attrib={"version": SDF_VERSION})
    model = ElementTree.SubElement(sdf, "model", attrib={"name": MODEL_NAME})
    statit_xml = ElementTree.SubElement(model, "static")
    statit_xml.text = str(True)
    link = ElementTree.SubElement(
        model, "link", attrib={"name": f"{MODEL_NAME}_link"}
    )

    # Visual geometry
    visual = ElementTree.SubElement(
        link, "visual", attrib={"name": f"{MODEL_NAME}_visual"}
    )
    visual_geometry = ElementTree.SubElement(visual, "geometry")
    visual_mesh = ElementTree.SubElement(visual_geometry, "mesh")
    visual_mesh_uri = ElementTree.SubElement(visual_mesh, "uri")
    visual_mesh_uri.text = os.path.relpath(PATH_VISUAL, os.path.dirname(PATH_SDF))

    # Collider geometry
    collision = ElementTree.SubElement(
        link, "collision", attrib={"name": f"{MODEL_NAME}_collision"}
    )
    collision_geometry = ElementTree.SubElement(collision, "geometry")
    collision_mesh = ElementTree.SubElement(collision_geometry, "mesh")
    collision_mesh_uri = ElementTree.SubElement(collision_mesh, "uri")
    collision_mesh_uri.text = os.path.relpath(PATH_COLLISION, os.path.dirname(PATH_SDF))

    write_xml(sdf, PATH_SDF)

# Generate a minimal config file. For more options, see: https://github.com/gazebosim/gz-sim/blob/a738dec47ae4f5c18f48a6d4d4b0edb500a490fa/examples/scripts/blender/procedural_dataset_generator.py#L1161-L1211
def generate_config():
    model_config = ElementTree.Element("model")
    name = ElementTree.SubElement(model_config, "name")
    name.text = MODEL_NAME

    # Path to SDF
    sdf_tag = ElementTree.SubElement(
        model_config, "sdf", attrib={"version": SDF_VERSION}
    )
    sdf_tag.text = os.path.relpath(PATH_SDF, os.path.dirname(PATH_CONFIG))

    write_xml(model_config, PATH_CONFIG)

def export_selection(filepath: str, with_materials: bool):
    os.makedirs(name=os.path.dirname(filepath), exist_ok=True)
    bpy.ops.export_scene.obj(
        filepath=filepath,
        check_existing=False,
        axis_forward="Y",
        axis_up="Z",
        use_selection=True,
        # use_animation=False,
        # use_mesh_modifiers=True,
        # use_edges=True,
        # use_smooth_groups=False,
        # use_smooth_groups_bitflags=False,
        # use_normals=True,
        # use_uvs=True,
        use_materials=with_materials,
        use_triangles=True,
        # use_nurbs=False,
        # use_vertex_groups=False,
        # use_blen_objects=True,
        # group_by_object=False,
        # group_by_material=False,
        # keep_vertex_order=False,
        # global_scale=1,
        path_mode="COPY",
    )
    print(f"Saved {filepath}")

copied_obj = duplicate(obj) 
apply_modifiers(copied_obj, is_collider=False)
export_selection(PATH_VISUAL, with_materials=True)
#TODO separate meshes and collect texture info
bpy.ops.object.delete()

copied_obj = duplicate(obj) 
apply_modifiers(copied_obj, is_collider=True)
export_selection(PATH_COLLISION, with_materials=False)
bpy.ops.object.delete()


generate_sdf()
generate_config()

