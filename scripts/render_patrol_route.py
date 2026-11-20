#!/usr/bin/env python3
"""Offline inspection of the reference route; never used by navigation."""

import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle

from patrol_route import build_route, route_lengths, transform
from scene_geometry import rectangle


def main():
    root = Path(__file__).resolve().parents[1]
    config = json.loads((root/"config/patrol.json").read_text())
    robot = json.loads((root/"config/limo.json").read_text())
    truth = json.loads((root/"config/scene_ground_truth.json").read_text())
    origin = [*config["home_world_pose_m_deg"][:2], math.radians(config["home_world_pose_m_deg"][2])]
    fig, ax = plt.subplots(figsize=(12, 8), facecolor="#f5f6f8")
    ax.set_facecolor("#e3e9df")
    ax.add_patch(Rectangle((-24,-20),48,40,color="#8f959c"))
    ax.add_patch(Rectangle((-36,-12.2),12,6.4,color="#8f959c"))
    for sign in (-1,1):
        ax.add_patch(Rectangle((-21.5,sign*18.3-.8),43,1.6,color="#c8c8bc"))
    ax.add_patch(Rectangle((-16.8,-.6),33.6,1.2,color="#8ba574"))
    for bay in truth["bays"]:
        car = bay.get("car")
        if car is None:
            continue
        shape = rectangle(car["position_m"],*car["envelope_m"][:2],car["yaw_rad"])
        color = [float(v) for v in car["rgba"].split()]
        ax.add_patch(Polygon(shape,facecolor=color,edgecolor="#303943",linewidth=.65))
    for name,a,b in [("north",(-24,20),(24,20)),("south",(-24,-20),(24,-20)),
                     ("east",(24,-20),(24,20)),("west1",(-24,-20),(-24,-12.2)),
                     ("west2",(-24,-5.8),(-24,20))]:
        ax.plot([a[0],b[0]],[a[1],b[1]],color="#555a60",lw=3)
    colors = {"entry":"#14b8a6", "loop":"#0969da", "return":"#f97316"}
    labels = {"entry":"1  Enter and merge", "loop":"2  Counterclockwise survey", "return":"3  Reverse to home"}
    for name,path in build_route(config,robot["wheelbase_m"]):
        points = [transform(p,origin) for p in path]
        if name == "return":
            ax.plot([p.x for p in points],[p.y for p in points],color=colors[name],lw=2,ls="--",label=labels[name],zorder=5)
        else:
            ax.plot([p.x for p in points],[p.y for p in points],color=colors[name],lw=2.4,label=labels[name],zorder=4)
        if name=="loop":
            for i in (35,220,420,610):
                if i+8 < len(points):
                    p,q=points[i],points[i+8]
                    ax.annotate("",xy=(q.x,q.y),xytext=(p.x,p.y),
                                arrowprops=dict(arrowstyle="-|>",color=colors[name],lw=2,mutation_scale=15))
    ax.add_patch(Polygon(rectangle(origin[:2],robot["footprint_size_m"][0],robot["footprint_size_m"][1],origin[2]),
                         facecolor="#fafafa",edgecolor="#101827",lw=1.3,zorder=6))
    ax.annotate("LIMO home\nentrance extension",xy=origin[:2],xytext=(-34,-1.7),fontsize=10,
                arrowprops=dict(arrowstyle="->",color="#172033"),color="#172033")
    speed_label = (f"{config['cruise_speed_m_s']:.2f} m/s cruise\n"
                   f"{config['corner_speed_m_s']:.2f} m/s corners")
    ax.annotate(speed_label,xy=(7,8.6),xytext=(5,21.6),fontsize=10,
                ha="center",arrowprops=dict(arrowstyle="->",color="#172033"),color="#172033")
    ax.set_xlim(-37,26)
    ax.set_ylim(-22,25)
    ax.set_aspect("equal")
    ax.set_xlabel("World x (m)")
    ax.set_ylabel("World y (m)")
    ax.set_title("Autonomous mapping reference route",loc="left",fontsize=17,fontweight="bold",pad=15)
    fig.text(.125,.02,"Reference geometry only. Nav2 replans from laser/map observations; this is not a Gazebo run.",fontsize=9,color="#465365")
    ax.legend(loc="lower left",framealpha=.96,fontsize=9)
    ax.spines[["top","right"]].set_visible(False)
    fig.tight_layout(rect=(0,.04,1,1))
    output=root/"docs/patrol_route.png"
    fig.savefig(output,dpi=160)
    print(output)


if __name__=="__main__":
    main()
