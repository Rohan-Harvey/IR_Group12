#!/usr/bin/env python3
"""Inspect the bundled COLLADA skin at three gait phases; optional numpy/matplotlib."""

from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


ROOT=Path(__file__).resolve().parents[1]


def render(root=ROOT):
    ns={"c":"http://www.collada.org/2005/11/COLLADASchema"}
    mesh=ET.parse(root/"models/walking_person/meshes/walk_blue.dae").getroot()
    def array(name):
        return np.fromstring(mesh.find(f".//c:source[@id='{name}']/c:float_array",ns).text,sep=" ")
    vertices=array("positions").reshape(-1,3)
    inverse=array("inverse_bind").reshape(-1,4,4)
    weights=np.fromstring(mesh.find(".//c:vertex_weights/c:v",ns).text,sep=" ",dtype=int).reshape(-1,2)[:,0]
    names=mesh.find(".//c:Name_array[@id='joint_names_array']",ns).text.split()
    parents={}
    def walk(node,parent=None):
        name=node.get("id")
        if node.get("type")=="JOINT":
            parents[name]=parent
            parent=name
        for child in node.findall("c:node",ns):
            walk(child,parent)
    scene=mesh.find(".//c:visual_scene",ns)
    for node in scene.findall("c:node",ns):
        walk(node)
    triangle_groups=[]
    for group in mesh.findall(".//c:triangles",ns):
        indices=np.fromstring(group.find("c:p",ns).text,sep=" ",dtype=int).reshape(-1,2)[:,0].reshape(-1,3)
        material=group.get("material")
        rgb=np.fromstring(mesh.find(f".//c:effect[@id='{material}_effect']//c:diffuse/c:color",ns).text,sep=" ")
        triangle_groups.append((indices,rgb[:3]))
    homogeneous=np.column_stack([vertices,np.ones(len(vertices))])
    figure=plt.figure(figsize=(11,6),facecolor="#f6f7f7")
    for column,frame in enumerate([0,6,12]):
        globals={}
        for name in names:
            local=array(name+"_transforms").reshape(-1,4,4)[frame]
            globals[name]=globals[parents[name]]@local if parents[name] else local
        transformed=np.empty_like(vertices)
        for i,name in enumerate(names):
            mask=weights==i
            transformed[mask]=(globals[name]@inverse[i]@homogeneous[mask].T).T[:,:3]
        transformed[:,0]-=globals['Root'][0,3]
        assert np.all(np.isfinite(transformed)) and transformed[:,2].max()<2.0
        ax=figure.add_subplot(1,3,column+1,projection="3d")
        for indices,rgb in triangle_groups:
            collection=Poly3DCollection(transformed[indices],facecolors=[rgb],edgecolors="none")
            ax.add_collection3d(collection)
        ax.set_xlim(-0.6,0.6);ax.set_ylim(-0.6,0.6);ax.set_zlim(0,1.85)
        ax.set_box_aspect([1.0,1.0,1.8]);ax.view_init(elev=12,azim=-50)
        ax.set_axis_off();ax.set_title(["Start of stride","Quarter stride","Half stride"][column],fontsize=12,color="#4a6171")
    figure.suptitle("BUNDLED WALKING ANIMATION",fontsize=19,fontweight="bold",color="#173047")
    figure.text(0.5,0.045,"Original low-poly model · animated hips, knees, shoulders and elbows",ha="center",color="#4a6171")
    figure.savefig(root/"docs/pedestrian_preview.png",dpi=140,facecolor=figure.get_facecolor())
    plt.close(figure)


if __name__=="__main__":
    render()
