import bpy
import os
import sys
from xml.dom import minidom
from xml.etree import ElementTree

if "--" in sys.argv:
    argv = sys.argv[sys.argv.index("--") + 1:]
else:
    argv = []

# Try to read the SEED from arguments
# TODO read start-goal positions from arguments
print(f"argv {argv}")
if len(argv) > 0:
    SEED = int(argv[0])
else:
    SEED = 0
print(f"Using seed: {SEED}")

MODEL_NAME = "Field"
# "//" is converted to the directory of the .blend file
PATH_MODEL = bpy.path.abspath(f"//../models/{MODEL_NAME}")
PATH_COLLISION = os.path.join(PATH_MODEL, 'assets/collision.obj')
PATH_VISUAL = os.path.join(PATH_MODEL, 'assets/visual.obj')
PATH_SDF = os.path.join(PATH_MODEL, 'model.sdf')
PATH_CONFIG = os.path.join(PATH_MODEL, 'model.config')
SDF_VERSION = '1.9'

# Select the model we want to export by name
bpy.ops.object.select_all(action='DESELECT')
bpy.data.objects[MODEL_NAME].select_set(True)
obj = bpy.data.objects[MODEL_NAME]

def separate_meshes_by_material(obj):
    all_objects_before = list(bpy.data.objects)
    ctx = bpy.context.copy()
    ctx['object'] = obj
    bpy.ops.mesh.separate(ctx, type='MATERIAL')
    new_objects = [o for o in bpy.data.objects if o not in all_objects_before]
    parts = [obj] + new_objects
    return parts

# Apply all modifiers on the selected Blender object
def apply_modifiers(obj, is_collider: bool):
    ctx = bpy.context.copy()
    ctx['object'] = obj
    for modifier in obj.modifiers:
        ctx['modifier'] = modifier
        # Set the Geometry Node inputs
        for input in modifier.node_group.inputs:
            if input.name == "is_exporting":
                modifier[input.identifier] = True
            if input.name == "is_collider":
                modifier[input.identifier] = is_collider
            if input.name == "seed":
                modifier[input.identifier] = SEED
        bpy.ops.object.modifier_apply(ctx, modifier=modifier.name)

    # Applying geometry nodes won't preserve the UV maps of the instanced meshes.
    #  Lucliky, the UV data is still available in the attributes, so we can convert
    #  back UV maps. For more info see: https://developer.blender.org/T85962#1375353

    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)

    parts = separate_meshes_by_material(obj)

    for obj in parts:
        print(f"Restoring UV map for object '{obj.name}'")
        material = obj.material_slots[0].material
        found_uv_map = False
        
        for i in reversed(range(len(obj.data.attributes))):
            attr = obj.data.attributes[i]
            obj.data.attributes.active_index = i
            
            # Use the UV map with the same name as the material 
            if attr.name == material.name:
                with bpy.context.temp_override(object=obj): # type: ignore
                    bpy.ops.geometry.attribute_convert(mode='UV_MAP')
        
        if found_uv_map:
            print(f"Restored UV map '{material.name}'")
        else:
            print(f"Warning: failed to find UV map '{material.name}'. Make sure each material has a corresponding UV map (with the same name) attrubute on the generated object")
            attribute_names = [attr.name for attr in obj.data.attributes]
            print(f"Available attributes are: {attribute_names}")

        for m in obj.modifiers:
            obj.modifiers.remove(m)

    # Select the parts for the following steps 
    bpy.ops.object.select_all(action='DESELECT')
    for obj in parts:
        obj.select_set(True)

# Util for duplicating a Blender object
def duplicate(obj, add_to_scene=True):
    obj_copy = obj.copy()
    obj_copy.data = obj_copy.data.copy()
    if add_to_scene:
        bpy.context.scene.collection.objects.link(obj_copy)
    return obj_copy

# Util for saving an XML file
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
        # Use ROS coordinate frame
        axis_forward="Y",
        axis_up="Z",
        use_selection=True,
        use_materials=with_materials,
        use_triangles=True,
        # copy all the texture images next to the model
        path_mode="COPY",
    )
    print(f"Saved {filepath}")

# Generate the visible mesh

# Duplicate the object before any modification
copied_obj = duplicate(obj) 
# Apply the Geometry Node (and any other) modifiers
apply_modifiers(copied_obj, is_collider=False)
# Export mesh
export_selection(PATH_VISUAL, with_materials=True)
#TODO separate meshes and collect texture info
# Delete the duplicated object
bpy.ops.object.delete()

# Generate the collision mesh
copied_obj = duplicate(obj) 
# Set is_collider True to turn on mesh optimizations
apply_modifiers(copied_obj, is_collider=True)
# Turn off materials for the OBJ export
export_selection(PATH_COLLISION, with_materials=False)
bpy.ops.object.delete()

# Create the model.sdf file
generate_sdf()
# Create the model.config file
generate_config()

