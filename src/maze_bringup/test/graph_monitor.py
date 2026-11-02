#!/usr/bin/env python3
"""Read-only graph identity, continuous-return and map-alignment diagnostics."""
import argparse
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=float,default=120.)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    rclpy.init();node=rclpy.create_node('graph_monitor')
    latest={};returns={};violations=[]

    def graph(message):
        latest['graph']=json.loads(message.data)

    def grid(message):
        cells=np.asarray(message.data,dtype=np.int16).reshape(message.info.height,message.info.width)
        distance=cv2.distanceTransform((cells<65).astype(np.uint8),cv2.DIST_L2,5)*message.info.resolution
        latest['map']=(message.info,cells,distance)

    def alignment():
        if 'map' not in latest or 'graph' not in latest:return {}
        info,cells,distance=latest['map'];g=latest['graph']
        nodes={p['id']:np.array(p['point']) for p in g['nodes']};errors=[]
        q=info.origin.orientation
        angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        c,s=math.cos(angle),math.sin(angle)
        for edge in g['edges']:
            if not edge['visited']:continue
            a,b=nodes[edge['source']],np.array(edge['end'])
            if np.linalg.norm(b-a)<.5:continue
            point=(a+b)/2-[info.origin.position.x,info.origin.position.y]
            x,y=(c*point[0]+s*point[1])/info.resolution,(-s*point[0]+c*point[1])/info.resolution
            col,row=int(math.floor(x)),int(math.floor(y))
            if 0<=row<cells.shape[0] and 0<=col<cells.shape[1] and cells[row,col]>=0:
                errors.append(abs(float(distance[row,col])-.30))
        return dict(map_samples=len(errors),map_wall_offset_mae_m=float(np.mean(errors)) if errors else None,
                    map_wall_offset_max_error_m=max(errors,default=None))

    stream=Path(args.output).open('w')
    start=time.monotonic()

    def status(message):
        report=json.loads(message.data);report.update(alignment());report['elapsed']=time.monotonic()-start
        if report.get('returning') and report.get('backtrack_target') is not None:
            key=report['backtracks'];identity=(report['junctions'],report['edges'],report['backtrack_target'])
            if key not in returns:returns[key]=identity
            # New side openings may legitimately add nodes/roads during a
            # return. The existing graph and target must still be retained.
            elif (returns[key][2]!=identity[2] or identity[0]<returns[key][0]
                  or identity[1]<returns[key][1]):
                violations.append(dict(backtrack=key,before=returns[key],after=identity))
        report['return_identity_violations']=len(violations)
        stream.write(json.dumps(report)+'\n');stream.flush()

    node.create_subscription(String,'/polyline/status',status,10)
    node.create_subscription(String,'/polyline/graph',graph,10)
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid,'/map',grid,qos)
    while time.monotonic()-start<args.duration:rclpy.spin_once(node,timeout_sec=.1)
    stream.close()
    if 'graph' in latest:Path(args.output+'.graph.json').write_text(json.dumps(latest['graph'],indent=2))
    Path(args.output+'.returns.json').write_text(json.dumps(dict(returns=returns,violations=violations),indent=2))
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
