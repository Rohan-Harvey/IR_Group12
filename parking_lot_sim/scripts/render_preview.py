#!/usr/bin/env python3
"""Draw a top-down plan from actual SDF visual geometry (optional matplotlib/numpy).

This drawing is a layout preview, not a screenshot of a running Gazebo instance.
The yellow start marker is an annotation, not a simulator object.
"""

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon
import numpy as np
from scene_geometry import rectangle


ROOT = Path(__file__).resolve().parents[1]


def transform(pose):
    x, y, z, roll, pitch, yaw = [float(v) for v in (pose or "0 0 0 0 0 0").split()]
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    t = np.eye(4)
    t[:3, :3] = rz @ ry @ rx
    t[:3, 3] = [x, y, z]
    return t


def hull(points):
    points = sorted(set((round(float(x), 8), round(float(y), 8)) for x, y in points))
    if len(points) <= 2:
        return points
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def draw(root, output):
    truth = json.loads((root / "config" / "scene_ground_truth.json").read_text())
    scale = truth["scale"]
    world = ET.parse(root / "worlds" / "parking_lot.sdf").getroot().find("world")
    figure, ax = plt.subplots(figsize=(15, 12), facecolor="#f6f7f7")
    figure.subplots_adjust(left=0.035, right=0.97, top=0.86, bottom=0.09)
    figure.text(0.06, 0.955, "PARKING SCENARIO LOT", fontsize=24, fontweight="bold", color="#173047")
    figure.text(0.06, 0.920,
                f"{truth['parked_car_count']} parked cars   /   Six test cases   /   Car-sized 4WD steering robot",
                fontsize=14, color="#4a6171")
    figure.text(0.94, 0.951, "GAZEBO HARMONIC", ha="right", fontsize=11, color="#4a6171")
    figure.text(0.94, 0.922, "Layout preview from the SDF geometry", ha="right", fontsize=10, color="#6b7d87")
    visuals = []
    scenery_visuals=[]
    for model in world.findall("model"):
        model_t = transform(model.findtext("pose"))
        for link in model.findall("link"):
            link_t = model_t @ transform(link.findtext("pose"))
            for visual in link.findall("visual"):
                t = link_t @ transform(visual.findtext("pose"))
                geometry = visual.find("geometry")
                color = [float(v) for v in visual.findtext("material/diffuse").split()]
                if geometry.find("box") is not None:
                    dimensions = [float(v) / 2 for v in geometry.findtext("box/size").split()]
                    points = np.array([[x, y, z, 1] for x in [-dimensions[0], dimensions[0]]
                                       for y in [-dimensions[1], dimensions[1]] for z in [-dimensions[2], dimensions[2]]])
                elif geometry.find("cylinder") is not None:
                    radius = float(geometry.findtext("cylinder/radius"))
                    length = float(geometry.findtext("cylinder/length"))
                    points = np.array([[radius * math.cos(a), radius * math.sin(a), z, 1]
                                       for a in np.linspace(0, math.tau, 20, endpoint=False)
                                       for z in [-length / 2, length / 2]])
                else:
                    radius = float(geometry.findtext("sphere/radius"))
                    points = np.array([[radius * math.cos(a), radius * math.sin(a), 0, 1]
                                       for a in np.linspace(0, math.tau, 28, endpoint=False)])
                    points = np.vstack([points, [0, 0, radius, 1]])
                points = (t @ points.T).T[:, :3]
                entry=(float(max(points[:, 2])),hull(points[:, :2]),color)
                visuals.append(entry)
                if model.get('name')!='parking_robot':scenery_visuals.append(entry)
    for _, polygon, color in sorted(visuals, key=lambda v: v[0]):
        if len(polygon) >= 3:
            ax.add_patch(Polygon(polygon, closed=True, facecolor=color, edgecolor="none"))
    # Actor and path symbols indicate initial positions and planned walking routes.
    colors = {"blue": "#4b95ff", "orange": "#f5a43a", "green": "#48d290"}
    for person in truth.get("pedestrians", []):
        x,y = person["start"]
        ex,ey = person["end"]
        color = colors[person["shirt"]]
        ax.plot([x,ex],[y,ey], linestyle=(0,(4,4)), lw=1.1, color=color, alpha=0.7)
        ax.add_patch(Circle((x,y),0.34*scale,facecolor=color,edgecolor="white",lw=0.6,zorder=3))
        ax.add_patch(Circle((x+0.03*scale,y),0.13*scale,facecolor="#c38d68",edgecolor="none",zorder=4))
    for y in [-9, 9]:
        ax.text(0, (y + 1.5) * scale, "Driving aisle", color="#b5bec5",
                fontsize=9, ha="center", va="center")
    if truth.get('scenarios'):
        for case in truth['scenarios']:
            x,y,yaw=case['goal_pose_m_deg']
            color='#75e6a2' if case['id']=='S6' else '#f6c46b' if case['id']=='S2' else '#fa8f8d'
            ghost=rectangle((x,y),4.62*scale,2.12*scale,math.radians(yaw))
            ax.add_patch(Polygon(ghost,closed=True,facecolor=color,edgecolor=color,alpha=0.20,lw=1.2))
            label_y=(22.1 if y>0 else -22.1)*scale
            short={'S1':'Inward angles','S2':'Pedestrian: wait','S3':'Too narrow',
                   'S4':'Too steep an angle','S5':'Entry blocked','S6':'Clear reverse park'}[case['id']]
            ax.annotate(case['id']+' · '+short,xy=(x,y),xytext=(x,label_y),
                        ha='center',va='center',fontsize=8.7,fontweight='bold',color=color,
                        bbox={'boxstyle':'round,pad=0.3','facecolor':'#233340','edgecolor':'none'},
                        arrowprops={'arrowstyle':'->','color':color,'lw':1.1})
            if case.get('reference_path_csv'):
                import csv
                with (root/'config'/case['reference_path_csv']).open() as handle:
                    path=list(csv.DictReader(handle))
                ax.plot([float(p['x_m']) for p in path],[float(p['y_m']) for p in path],
                        color=color,ls='--',lw=1.3)
    sx, sy, _ = truth["suggested_robot_spawn"]["position_m"]
    ax.annotate("Car-sized 4WD robot\n4.62 × 2.12 m footprint", xy=(sx, sy), xytext=(-28 * scale, -14.5 * scale),
                fontsize=9, color="white", ha="center",
                arrowprops={"arrowstyle": "->", "color": "#fbd371", "lw": 1.2})
    ax.text(-30 * scale, -8.8 * scale, "ENTRANCE", fontsize=9, fontweight="bold", color="white", ha="center")
    ax.set_xlim(-35 * scale, 28 * scale)
    ax.set_ylim(-26 * scale, 26 * scale)
    ax.set_aspect("equal")
    ax.axis("off")
    figure.text(0.06, 0.045,
                "S6 is clear now. S2 must wait while a pedestrian occupies its swept path. Labels and path overlays are preview annotations.",
                fontsize=10, color="#4a6171")
    figure.text(0.06, 0.022,
                "Walking actors are animated visual obstacles. Robot wheels and sensor bodies are physical models.",
                fontsize=9, color="#6b7d87")
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=135, facecolor=figure.get_facecolor())
    plt.close(figure)
    if truth.get("robot"):
        closeup, detail = plt.subplots(figsize=(7,6),facecolor="#f6f7f7")
        for _,polygon,color in sorted(visuals,key=lambda v:v[0]):
            if len(polygon)>=3:
                detail.add_patch(Polygon(polygon,closed=True,facecolor=color,edgecolor="none"))
        for text,offset,label_offset in [
            ("360° LiDAR",(0,0),(0.05,1.65)),
            ("Front camera",(2.25,0),(2.45,1.15)),
            ("Four powered wheels",(-1.375,-0.79),(-0.4,-1.75))]:
            detail.annotate(text,xy=(sx+offset[0]*scale,sy+offset[1]*scale),
                            xytext=(sx+label_offset[0]*scale,sy+label_offset[1]*scale),
                            color="white",fontsize=11,ha="center",
                            arrowprops={"arrowstyle":"->","color":"#f5d36a"})
        detail.set_xlim(sx-2.9*scale,sx+3.1*scale)
        detail.set_ylim(sy-2.1*scale,sy+2.1*scale)
        detail.set_aspect("equal")
        detail.axis("off")
        closeup.suptitle("CAR-SIZED 4WD STEERING ROBOT",fontsize=15,fontweight="bold",color="#173047")
        closeup.savefig(output.parent/"robot_preview.png",dpi=150,facecolor=closeup.get_facecolor())
        plt.close(closeup)
    if truth.get('scenarios'):
        render_case_panels(root,scenery_visuals,truth,output.parent)
    print(f"Saved {output}")


def render_case_panels(root,visuals,truth,directory):
    import csv
    scale=truth['scale']
    figure,axes=plt.subplots(2,3,figsize=(16,10),facecolor='#f6f7f7')
    figure.subplots_adjust(left=.03,right=.98,top=.87,bottom=.08,hspace=.25,wspace=.06)
    figure.suptitle('SIX PARKING TEST CASES',fontsize=24,fontweight='bold',color='#173047',y=.97)
    figure.text(.5,.918,'Vehicle: 4.62 × 2.12 m   |   Safety margin: 0.15 m   |   Heading tolerance: ±10°',
                ha='center',fontsize=12,color='#4a6171')
    bounds={'S1':[-21,-7,7,20], 'S2':[-7,10,7,20], 'S3':[-21,-7,-20,-6],
            'S4':[-7,7,-20,-6], 'S5':[6,22,-20,-6], 'S6':[6,24,6,20]}
    captions={'S1':'REJECT · footprint collides inside the tapered gap',
              'S2':'WAIT · static path fits; person initially blocks it',
              'S3':'REJECT · 1.70 m gap < 2.12 m footprint',
              'S4':'REJECT · diagonal fit is 35° from perpendicular',
              'S5':'REJECT · final pose fits; 1.75 m entry throat',
              'S6':'ACCEPT · 8.00 m opening and clear reverse arc'}
    for ax,case in zip(axes.flat,truth['scenarios']):
        for _,polygon,color in sorted(visuals,key=lambda v:v[0]):
            if len(polygon)>=3:ax.add_patch(Polygon(polygon,closed=True,facecolor=color,edgecolor='none'))
        color='#75e6a2' if case['id']=='S6' else '#f6c46b' if case['id']=='S2' else '#fa8f8d'
        x,y,yaw=case.get('diagonal_pose_m_deg',case['goal_pose_m_deg'])
        ax.add_patch(Polygon(rectangle((x,y),4.62*scale,2.12*scale,math.radians(yaw)),
                             closed=True,facecolor=color,edgecolor=color,alpha=.32,lw=1.5))
        if case.get('reference_path_csv'):
            with (root/'config'/case['reference_path_csv']).open() as handle:path=list(csv.DictReader(handle))
            ax.plot([float(p['x_m']) for p in path],[float(p['y_m']) for p in path],color=color,ls='--',lw=1.8)
        if case['id']=='S2':
            for p in truth['pedestrians']:
                if p['name']==case['blocking_pedestrian']:
                    ax.plot([p['start'][0],p['end'][0]],[p['start'][1],p['end'][1]],color='#f6c46b',ls=':',lw=1.5)
                    ax.add_patch(Circle(p['start'],.40*scale,facecolor='#f6c46b',edgecolor='white'))
        left,right,bottom,top=bounds[case['id']]
        ax.set_xlim(left*scale,right*scale);ax.set_ylim(bottom*scale,top*scale)
        ax.set_aspect('equal');ax.axis('off')
        ax.set_title(case['id']+' · '+case['name'],fontsize=11.5,color='#173047',fontweight='bold',pad=10)
        ax.text(.5,-.045,captions[case['id']],transform=ax.transAxes,ha='center',fontsize=9,color='#4a6171')
    figure.text(.5,.025,'Colored vehicle outlines and trajectories are reference overlays, not painted objects or navigation inputs.',
                ha='center',fontsize=10,color='#6b7d87')
    figure.savefig(directory/'scenario_overview.png',dpi=145,facecolor=figure.get_facecolor())
    plt.close(figure)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    draw(args.root, args.output or args.root / "docs" / "parking_lot_preview.png")
