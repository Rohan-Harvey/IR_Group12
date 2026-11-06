"""Generate original, locally bundled skinned COLLADA people and walking scripts.

The block-style humanoid uses one rigid bone per body part, animated hips,
knees, shoulders and elbows, and forward root motion for interpolate_x.
Actors are visual / GPU-sensor obstacles, not physics collision bodies.
"""

import math
from pathlib import Path
import xml.etree.ElementTree as ET


NS = "http://www.collada.org/2005/11/COLLADASchema"
BONES = [
    ("Root", None, (0, 0, 0)), ("Hips", "Root", (0, 0, 0.92)),
    ("Spine", "Hips", (0, 0, 0.05)), ("Head", "Spine", (0, 0, 0.58)),
    ("LeftArm", "Spine", (0, 0.29, 0.40)), ("LeftForearm", "LeftArm", (0, 0, -0.31)),
    ("RightArm", "Spine", (0, -0.29, 0.40)), ("RightForearm", "RightArm", (0, 0, -0.31)),
    ("LeftThigh", "Hips", (0, 0.13, -0.02)), ("LeftShin", "LeftThigh", (0, 0, -0.40)),
    ("RightThigh", "Hips", (0, -0.13, -0.02)), ("RightShin", "RightThigh", (0, 0, -0.40)),
]
PROFILES = {"blue": (0.08, 0.30, 0.72), "orange": (0.93, 0.36, 0.08), "green": (0.12, 0.55, 0.31)}
ROUTES = [
    {"name": "pedestrian_1", "start": [-15, 18.3], "end": [15, 18.3], "ground_z_m": 0.12, "speed": 1.0, "delay": 0, "shirt": "blue"},
    {"name": "pedestrian_2", "start": [12, -18.3], "end": [-12, -18.3], "ground_z_m": 0.12, "speed": 0.9, "delay": 2, "shirt": "orange"},
    {"name": "pedestrian_3", "start": [-19, -5.5], "end": [-19, -12.5], "speed": 0.8, "delay": 5, "shirt": "green"},
    {"name": "pedestrian_4", "start": [19.5, 13], "end": [19.5, 5], "speed": 0.9, "delay": 3, "shirt": "orange"},
    {"name": "pedestrian_5", "start": [-12, -9], "end": [12, -9], "speed": 1.0, "delay": 0, "shirt": "blue"},
    {"name": "pedestrian_6", "start": [12, 9], "end": [-12, 9], "speed": 1.1, "delay": 1, "shirt": "green"},
]


def values(items):
    return " ".join(f"{v:.8f}" if isinstance(v, (float, int)) else str(v) for v in items)


def matrix(position=(0, 0, 0), pitch=0):
    c, s = math.cos(pitch), math.sin(pitch)
    x, y, z = position
    # COLLADA XML writes the mathematical column-vector matrix row by row.
    return [c, 0, s, x, 0, 1, 0, y, -s, 0, c, z, 0, 0, 0, 1]


def write_person(path, shirt):
    def node(parent, tag, text=None, **attrs):
        item = ET.SubElement(parent, "{" + NS + "}" + tag, attrs)
        if text is not None:
            item.text = str(text)
        return item

    ET.register_namespace("", NS)
    root = ET.Element("{" + NS + "}COLLADA", version="1.4.1")
    asset = node(root, "asset")
    node(asset, "created", "2026-11-06T00:00:00Z")
    node(asset, "modified", "2026-11-06T00:00:00Z")
    node(asset, "unit", name="meter", meter="1")
    node(asset, "up_axis", "Z_UP")
    colors = {"shirt": PROFILES[shirt], "pants": (0.12, 0.15, 0.20),
              "skin": (0.66, 0.42, 0.27), "shoes": (0.045, 0.05, 0.06), "hair": (0.09, 0.065, 0.04)}
    effects = node(root, "library_effects")
    materials = node(root, "library_materials")
    for name, rgb in colors.items():
        effect = node(effects, "effect", id=name + "_effect")
        phong = node(node(node(effect, "profile_COMMON"), "technique", sid="common"), "phong")
        node(node(phong, "diffuse"), "color", values([*rgb, 1]))
        node(node(phong, "ambient"), "color", values([*rgb, 1]))
        node(node(phong, "specular"), "color", "0.05 0.05 0.05 1")
        material = node(materials, "material", id=name + "_material", name=name)
        node(material, "instance_effect", url="#" + name + "_effect")
    positions, normals, weights = [], [], []
    groups = {name: [] for name in colors}
    bone_index = {name: i for i, (name, _, _) in enumerate(BONES)}

    def box(center, size, bone, material):
        x, y, z = center
        a, b, c = [v / 2 for v in size]
        corners = [(x-a,y-b,z-c),(x+a,y-b,z-c),(x+a,y+b,z-c),(x-a,y+b,z-c),
                   (x-a,y-b,z+c),(x+a,y-b,z+c),(x+a,y+b,z+c),(x-a,y+b,z+c)]
        faces = [((0,3,2,1),(0,0,-1)),((4,5,6,7),(0,0,1)),((0,1,5,4),(0,-1,0)),
                 ((1,2,6,5),(1,0,0)),((2,3,7,6),(0,1,0)),((3,0,4,7),(-1,0,0))]
        for face, normal in faces:
            for triangle in ((face[0],face[1],face[2]),(face[0],face[2],face[3])):
                for vertex in triangle:
                    index = len(positions) // 3
                    positions.extend(corners[vertex])
                    normals.extend(normal)
                    weights.append(bone_index[bone])
                    groups[material].extend([index, index])
    box((0,0,0.92),(0.25,0.36,0.18),"Hips","pants")
    box((0,0,1.19),(0.27,0.44,0.46),"Spine","shirt")
    box((0,0,1.49),(0.13,0.15,0.13),"Head","skin")
    box((0,0,1.64),(0.24,0.22,0.26),"Head","skin")
    box((0,0,1.77),(0.25,0.23,0.055),"Head","hair")
    box((0.14,0,1.64),(0.07,0.085,0.09),"Head","skin")
    for side, y in (("Left",0.29),("Right",-0.29)):
        box((0,y,1.215),(0.14,0.14,0.31),side+"Arm","shirt")
        box((0,y,0.91),(0.12,0.12,0.30),side+"Forearm","skin")
        box((0,y,0.745),(0.13,0.12,0.09),side+"Forearm","skin")
    for side, y in (("Left",0.13),("Right",-0.13)):
        box((0,y,0.70),(0.18,0.18,0.40),side+"Thigh","pants")
        box((0,y,0.30),(0.16,0.16,0.40),side+"Shin","pants")
        box((0.075,y,0.085),(0.32,0.19,0.15),side+"Shin","shoes")

    def source(parent, name, data, params, stride=1, array_type="float_array", param_type="float"):
        item = node(parent, "source", id=name)
        node(item, array_type, values(data), id=name+"_array", count=str(len(data)))
        accessor = node(node(item, "technique_common"), "accessor", source="#"+name+"_array",
                        count=str(len(data)//stride), stride=str(stride))
        for param in params:
            node(accessor, "param", name=param, type=param_type)
    mesh = node(node(node(root,"library_geometries"),"geometry",id="person_mesh",name="person"),"mesh")
    source(mesh,"positions",positions,["X","Y","Z"],3)
    source(mesh,"normals",normals,["X","Y","Z"],3)
    vertices = node(mesh,"vertices",id="person_vertices")
    node(vertices,"input",semantic="POSITION",source="#positions")
    for material, indices in groups.items():
        triangles = node(mesh,"triangles",material=material,count=str(len(indices)//6))
        node(triangles,"input",semantic="VERTEX",source="#person_vertices",offset="0")
        node(triangles,"input",semantic="NORMAL",source="#normals",offset="1")
        node(triangles,"p"," ".join(map(str,indices)))
    skin = node(node(node(root,"library_controllers"),"controller",id="person_skin"),"skin",source="#person_mesh")
    node(skin,"bind_shape_matrix",values(matrix()))
    source(skin,"joint_names",list(bone_index),["JOINT"],array_type="Name_array",param_type="Name")
    world_positions = {}
    inverse = []
    for name,parent,xyz in BONES:
        parent_xyz = world_positions.get(parent,(0,0,0))
        world_positions[name] = tuple(a+b for a,b in zip(parent_xyz,xyz))
        inverse.extend(matrix(tuple(-v for v in world_positions[name])))
    source(skin,"inverse_bind",inverse,["TRANSFORM"],16,param_type="float4x4")
    source(skin,"weights",[1.0],["WEIGHT"])
    joints = node(skin,"joints")
    node(joints,"input",semantic="JOINT",source="#joint_names")
    node(joints,"input",semantic="INV_BIND_MATRIX",source="#inverse_bind")
    vertex_weights = node(skin,"vertex_weights",count=str(len(weights)))
    node(vertex_weights,"input",semantic="JOINT",source="#joint_names",offset="0")
    node(vertex_weights,"input",semantic="WEIGHT",source="#weights",offset="1")
    node(vertex_weights,"vcount"," ".join("1" for _ in weights))
    node(vertex_weights,"v"," ".join(f"{i} 0" for i in weights))
    animations = node(root,"library_animations")
    times = [i*1.2/24 for i in range(25)]
    for name,parent,xyz in BONES:
        animation = node(animations,"animation",id=name+"_animation",name=name)
        transforms = []
        for t in times:
            phase = math.tau*t/1.2
            pitch = 0.0
            translation = xyz
            if name == "Root":
                translation = (t,0,0.008*(1-math.cos(2*phase)))
            if name in ("LeftThigh","RightThigh","LeftArm","RightArm"):
                side = 1 if name.startswith("Left") else -1
                pitch = side*math.sin(phase)*(0.40 if "Thigh" in name else -0.46)
            if name in ("LeftShin","RightShin"):
                side = 1 if name.startswith("Left") else -1
                pitch = -0.60*max(0,side*math.sin(phase))
            if "Forearm" in name:
                pitch = -0.12
            transforms.extend(matrix(translation,pitch))
        source(animation,name+"_times",times,["TIME"])
        source(animation,name+"_transforms",transforms,["TRANSFORM"],16,param_type="float4x4")
        source(animation,name+"_interpolation",["LINEAR"]*len(times),["INTERPOLATION"],
               array_type="Name_array",param_type="Name")
        sampler = node(animation,"sampler",id=name+"_sampler")
        for semantic,suffix in (("INPUT","times"),("OUTPUT","transforms"),("INTERPOLATION","interpolation")):
            node(sampler,"input",semantic=semantic,source="#"+name+"_"+suffix)
        node(animation,"channel",source="#"+name+"_sampler",target=name+"/transform")
    clip = node(node(root,"library_animation_clips"),"animation_clip",id="walk",name="walk",start="0",end="1.2")
    for name,_,_ in BONES:
        node(clip,"instance_animation",url="#"+name+"_animation")
    scene = node(node(root,"library_visual_scenes"),"visual_scene",id="Scene",name="Scene")
    bone_nodes = {}
    for name,parent,xyz in BONES:
        bone = node(bone_nodes[parent] if parent else scene,"node",id=name,sid=name,name=name,type="JOINT")
        node(bone,"matrix",values(matrix(xyz)),sid="transform")
        bone_nodes[name] = bone
    instance = node(node(scene,"node",id="Person",name="Person"),"instance_controller",url="#person_skin")
    node(instance,"skeleton","#Root")
    bindings = node(node(instance,"bind_material"),"technique_common")
    for material in colors:
        node(bindings,"instance_material",symbol=material,target="#"+material+"_material")
    node(node(root,"scene"),"instance_visual_scene",url="#Scene")
    ET.indent(root,space="  ")
    path.parent.mkdir(parents=True,exist_ok=True)
    ET.ElementTree(root).write(path,encoding="utf-8",xml_declaration=True)


def add_people(builder, enabled=True,routes=None):
    if not enabled:
        return
    def sub(parent,tag,value=None,**attrs):
        item=ET.SubElement(parent,tag,attrs)
        if value is not None:
            item.text=str(value)
        return item
    for route in ROUTES if routes is None else routes:
        start, end = route["start"], route["end"]
        dx,dy = end[0]-start[0],end[1]-start[1]
        heading = math.atan2(dy,dx)
        duration = math.hypot(dx,dy)/route["speed"]
        actor = sub(builder.world,"actor",name=route["name"])
        sub(actor,"pose",builder.pose((*start,route.get("ground_z_m",0))))
        asset = f"model://walking_person/meshes/walk_{route['shirt']}.dae"
        skin=sub(actor,"skin")
        sub(skin,"filename",asset)
        sub(skin,"scale",builder.scale)
        animation=sub(actor,"animation",name="walk")
        sub(animation,"filename",asset)
        sub(animation,"scale",builder.scale)
        sub(animation,"interpolate_x","true")
        script=sub(actor,"script")
        sub(script,"loop","true")
        sub(script,"auto_start","true")
        sub(script,"delay_start",route["delay"])
        trajectory=sub(script,"trajectory",id="0",type="walk",tension="1.0")
        # Waypoints are relative to the initial actor pose, not world positions.
        for t,x,y,yaw in [(0,0,0,heading),(duration,dx,dy,heading),
                          (duration+1,dx,dy,heading+math.pi),
                          (2*duration+1,0,0,heading+math.pi),(2*duration+2,0,0,heading)]:
            waypoint=sub(trajectory,"waypoint")
            sub(waypoint,"time",t)
            sub(waypoint,"pose",builder.pose((x,y,0),(0,0,yaw)))


def write_people_assets(root):
    for shirt in PROFILES:
        write_person(Path(root)/"models"/"walking_person"/"meshes"/f"walk_{shirt}.dae",shirt)
