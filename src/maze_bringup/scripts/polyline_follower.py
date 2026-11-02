#!/usr/bin/env python3
"""Laser-built global orthogonal paths, left-first exploration and pure pursuit."""
import json
import math
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from polyline_core import ExitDetector, Explorer, Frame2D, ScanView, predict_pose, pure_pursuit, pursuit_target, yaw_rate_command, wrap
from return_planner import ReturnGrid
from branch_map import BranchMap
from motion_reference import ReferenceBuilder, project
from terminal_start import TerminalStartGate


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class PolylineFollower(Node):
    def __init__(self):
        super().__init__('polyline_follower')
        defaults=dict(wall_clearance=.30,lookahead=.32,max_linear_speed=.53,
                      max_angular_speed=4.3,max_alignment_speed=2.4,goal_tolerance=.065,
                      turn_preview=.48,scan_timeout=.7,enabled=True,
                      prediction_time=.10,yaw_rate_gain=.5,align_enter_angle=2.4,align_exit_angle=.50,
                      mpc_align_enter_angle=.85,mpc_align_exit_angle=.50,
                      planning_frame='map',junction_merge_radius=.22,map_tf_timeout=1.0,
                      exit_detection_enabled=True,exit_clearance=1.,exit_confirm_time=.8,exit_min_travel=1.,
                      exit_required_open_sides=3,exit_sector_half_angle_deg=10.,exit_min_open_arc_deg=180.,
                      return_home_enabled=True,return_clearance=.20,return_wall_cost=4.,
                      return_lookahead=.28,return_goal_tolerance=.10,return_goal_gain=1.2,
                      return_stop_time=.6,return_retry_period=2.,map_dead_end_pruning=True,
                      wait_for_start=True,terminal_device='/dev/tty',
                      controller_mode='pp',motion_speed=-1.,reference_clearance=.18,
                      reference_lateral_accel=.9,mpc_horizon=20,mpc_dt=.05)
        self.p={key:self.declare_parameter(key,value).value for key,value in defaults.items()}
        self.planner=Explorer(self.p['wall_clearance'],self.p['turn_preview'],self.p['goal_tolerance'],
                              self.p['junction_merge_radius'])
        self.frame=self.p['planning_frame']
        self.map_from_odom=Frame2D(np.zeros(2),0.)
        self.position=None;self.heading=0.
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.odom=self.scan=self.view=None
        self.scan_received=self.odom_received=float('-inf')
        self.last_stamp=None;self.last_status=0.;self.last_log=None
        self.waiting_for_start=self.p['wait_for_start']
        self.enabled=self.p['enabled'] and not self.waiting_for_start
        self.state='WAIT_START' if self.waiting_for_start else 'WAIT_SENSORS'
        self.distance=0.;self.previous_odom=None;self.trajectory=[]
        self.command=(0.,0.);self.target=None
        self.aligning=False
        if self.p['controller_mode'] not in ('pp','optimized_pp','mpc'):
            raise ValueError('controller_mode must be pp, optimized_pp or mpc')
        self.motion_builder=ReferenceBuilder(clearance=self.p['reference_clearance'],
                                             lateral_accel=self.p['reference_lateral_accel'])
        self.motion_speed=self.p['motion_speed'] if self.p['motion_speed']>0 else self.p['max_linear_speed']
        if not 0<self.motion_speed<=.65:raise ValueError('motion_speed must be within (0, 0.65] m/s')
        self.motion_path=[];self.mpc_prediction=[];self.mpc_reference=[];self.motion_report={}
        self.motion_mpc=None
        self.motion_recovering=False
        if self.p['controller_mode']=='mpc':
            from differential_mpc import DifferentialMPC
            self.motion_mpc=DifferentialMPC(horizon=self.p['mpc_horizon'],dt=self.p['mpc_dt'],
                                           max_speed=self.motion_speed)
        self.exit_detector=ExitDetector(self.p['exit_clearance'],self.p['exit_confirm_time'],
                                        self.p['exit_min_travel'],
                                        required_open_sides=self.p['exit_required_open_sides'],
                                        sector_half_angle_deg=self.p['exit_sector_half_angle_deg'],
                                        min_open_arc_deg=self.p['exit_min_open_arc_deg'])
        self.exit_position=None
        self.mission_phase='EXPLORING';self.home_position=None;self.home_anchor=None
        self.latest_map=None;self.return_path=[];self.return_anchors=[];self.return_index=0
        self.return_active=[];self.return_reason='';self.return_plan_length=0.;self.return_expanded=0
        self.return_attempts=0;self.return_ready_at=0.;self.return_next_plan=0.
        self.return_executor=ThreadPoolExecutor(max_workers=1,thread_name_prefix='return_astar')
        self.return_future=None
        self.cmd_pub=self.create_publisher(Twist,'/cmd_vel',1)
        self.path_pub=self.create_publisher(Path,'/polyline/global_path',1)
        self.active_pub=self.create_publisher(Path,'/polyline/active_path',1)
        self.optimized_pub=self.create_publisher(Path,'/polyline/optimized_path',1)
        self.predicted_pub=self.create_publisher(Path,'/polyline/mpc_predicted_path',1)
        self.mpc_ref_pub=self.create_publisher(Path,'/polyline/mpc_reference_path',1)
        self.return_pub=self.create_publisher(Path,'/polyline/astar_return_path',
                           QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.track_pub=self.create_publisher(Path,'/polyline/trajectory',1)
        self.target_pub=self.create_publisher(PoseStamped,'/polyline/lookahead',1)
        self.markers=self.create_publisher(MarkerArray,'/polyline/markers',1)
        self.status_pub=self.create_publisher(String,'/polyline/status',1)
        self.result_pub=self.create_publisher(String,'/polyline/result',
                            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.graph_pub=self.create_publisher(String,'/polyline/graph',1)
        self.compat_status=self.create_publisher(String,'/pp/status',1)
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
        if self.frame=='map':
            self.create_subscription(MarkerArray,'/slam_toolbox/graph_visualization',self.on_slam_graph,1)
            self.create_subscription(OccupancyGrid,'/map',self.on_map,
                                     QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(SetBool,'/polyline/set_enabled',self.set_enabled)
        self.create_service(SetBool,'/pp/set_enabled',self.set_enabled)
        self.start_gate=(TerminalStartGate(self.p['terminal_device'],self.get_logger().info)
                         if self.waiting_for_start else None)
        self.create_timer(.05,self.control,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def destroy_node(self):
        if self.start_gate is not None:self.start_gate.close()
        return super().destroy_node()

    def on_odom(self,message):
        self.odom=message;self.odom_received=time.monotonic()
        p=message.pose.pose.position;point=np.array([p.x,p.y])
        if self.previous_odom is not None:self.distance+=float(np.linalg.norm(point-self.previous_odom))
        self.previous_odom=point

    def on_slam_graph(self,message):
        poses={m.id:np.array([m.pose.position.x,m.pose.position.y]) for m in message.markers
               if m.ns=='slam_toolbox' and m.type==Marker.SPHERE and m.action==Marker.ADD
               and m.header.frame_id==self.frame}
        if not poses:return
        self.planner.update_pose_graph(poses)
        if self.home_position is not None:
            self.home_position=self.planner.slam.resolve(self.home_anchor,self.home_position).copy()
            if self.home_anchor is None or self.home_anchor.second is None:
                self.home_anchor=self.planner.slam.attach(self.home_position)
        for i,point in enumerate(self.return_path):
            self.return_path[i]=self.planner.slam.resolve(self.return_anchors[i],point).copy()
        for sample in self.trajectory:
            sample[0]=self.planner.slam.resolve(sample[1],sample[0]).copy()
            if sample[1] is None or sample[1].second is None:
                sample[1]=self.planner.slam.attach(sample[0])

    def on_map(self,message):
        if message.header.frame_id!=self.frame:return
        self.latest_map=message
        if self.mission_phase!='EXPLORING' and self.p.get('controller_mode','pp')=='pp':return
        info=message.info
        cells=np.asarray(message.data,dtype=np.int16).reshape(info.height,info.width)
        rows,cols=np.nonzero(cells>=65)
        transform=Frame2D(np.array([info.origin.position.x,info.origin.position.y]),yaw(info.origin.orientation))
        if getattr(self,'motion_builder',None) is not None:
            self.motion_builder.set_map(cells,info.resolution,transform.translation,transform.yaw)
        if self.mission_phase!='EXPLORING':return
        c,s=math.cos(transform.yaw),math.sin(transform.yaw)
        x=(cols+.5)*info.resolution;y=(rows+.5)*info.resolution
        walls=transform.translation+np.column_stack((c*x-s*y,s*x+c*y))
        def is_free(point):
            xy=np.floor(transform.inverse(point)/info.resolution).astype(int)
            return bool(0<=xy[0]<info.width and 0<=xy[1]<info.height and 0<=cells[xy[1],xy[0]]<65)
        self.planner.align_to_walls(walls,is_free)
        self.planner.set_branch_map(BranchMap(cells,info.resolution,transform.translation,transform.yaw)
                                   if self.p['map_dead_end_pruning'] else None)

    def localization(self):
        if self.frame=='odom':return Frame2D(np.zeros(2),0.)
        transform=self.buffer.lookup_transform(self.frame,'odom',rclpy.time.Time())
        stamp=rclpy.time.Time.from_msg(transform.header.stamp)
        age=(self.get_clock().now()-stamp).nanoseconds*1e-9
        if age>self.p['map_tf_timeout']:
            raise TransformException('SLAM map to odom transform expired')
        t=transform.transform
        return Frame2D(np.array([t.translation.x,t.translation.y]),yaw(t.rotation))

    def on_scan(self,message):
        self.scan=message;self.scan_received=time.monotonic()

    def set_enabled(self,request,response):
        if self.mission_phase in ('EXIT_REACHED','HOME_REACHED'):
            self.publish_command(0.,0.)
            response.success=not request.data
            response.message='mission finished; restart the exploration to run again'
            return response
        self.enabled=request.data;self.publish_command(0.,0.)
        if request.data:
            self.waiting_for_start=False
            if getattr(self,'start_gate',None) is not None:self.start_gate.close()
        if getattr(self,'motion_mpc',None) is not None:self.motion_mpc.reset()
        self.exit_detector.reset_candidate()
        response.success=True;response.message='enabled' if self.enabled else 'paused'
        return response

    def publish_command(self,linear,angular):
        self.command=float(linear),float(angular)
        message=Twist();message.linear.x=self.command[0];message.angular.z=self.command[1]
        self.cmd_pub.publish(message)

    def update_scan(self,position,heading):
        key=(self.scan.header.stamp.sec,self.scan.header.stamp.nanosec)
        if key==self.last_stamp:return
        try:
            transform=self.buffer.lookup_transform('odom',self.scan.header.frame_id,
                       rclpy.time.Time.from_msg(self.scan.header.stamp)).transform
        except TransformException:
            return  # Keep a fresh previous scan while the next odometry TF arrives.
        # Apply the SAME current SLAM correction to the timestamped scan and
        # current odometry. Historical map TF versus current map TF would mix
        # two localizations during loop corrections.
        origin=self.map_from_odom.apply([transform.translation.x,transform.translation.y])
        self.view=ScanView(self.scan,origin,wrap(yaw(transform.rotation)+self.map_from_odom.yaw),
                           self.p['wall_clearance'])
        self.last_stamp=key
        if self.enabled and self.mission_phase=='EXPLORING':self.planner.update(self.view,position,heading)

    def control(self):
        if getattr(self,'waiting_for_start',False) and self.start_gate.poll():
            self.set_enabled(SetBool.Request(data=True),SetBool.Response())
            self.get_logger().info('收到 start，开始原有探索与返航流程。')
        now=time.monotonic();linear=angular=0.
        fresh=(self.scan is not None and self.odom is not None and
               now-self.scan_received<=self.p['scan_timeout'] and now-self.odom_received<=.7)
        if self.enabled and fresh and self.p['exit_detection_enabled'] and self.mission_phase=='EXPLORING':
            stamp=self.scan.header.stamp.sec+self.scan.header.stamp.nanosec*1e-9
            if self.exit_detector.update(self.scan,self.distance,stamp) and self.exit_position is None:
                # Completion does not depend on SLAM matching in a featureless
                # exterior. Use the latest available map correction for display.
                p=self.odom.pose.pose.position
                self.exit_position=self.map_from_odom.apply([p.x,p.y])
                self.position=self.exit_position.copy();self.target=None;self.aligning=False
                self.mission_phase='PLAN_RETURN' if self.p['return_home_enabled'] else 'EXIT_REACHED'
                self.return_ready_at=now+self.p['return_stop_time']
                self.return_reason='exit confirmed; stopping before A* return'
                self.publish_command(0.,0.)
                self.result_pub.publish(String(data=json.dumps(dict(
                    state='EXIT_REACHED',success=True,reason='sustained open surroundings after corridor traversal',
                    finished=not self.p['return_home_enabled'],
                    frame=self.frame,position=self.exit_position.tolist(),scan_stamp_s=stamp,
                    exit_detection=self.exit_detector.report()))))
                self.get_logger().info('MAZE EXIT REACHED / 已驶出迷宫：停车确认，准备 A* 返航。'
                    if self.p['return_home_enabled'] else 'MAZE EXIT REACHED / 已驶出迷宫：停车结束。')
        elif not self.exit_detector.reached:
            self.exit_detector.reset_candidate()
        if self.mission_phase in ('EXIT_REACHED','HOME_REACHED'):
            self.state=self.mission_phase
            if hasattr(self,'motion_path'):
                self.motion_path=[];self.mpc_prediction=[];self.mpc_reference=[]
        elif getattr(self,'waiting_for_start',False):self.state='WAIT_START'
        elif not self.enabled:self.state='PAUSED'
        elif self.scan is None or self.odom is None:self.state='WAIT_SENSORS'
        elif now-self.scan_received>self.p['scan_timeout'] or now-self.odom_received>.7:self.state='SENSOR_TIMEOUT'
        else:
            try:
                self.map_from_odom=self.localization()
            except TransformException:
                self.state='WAIT_MAP_LOCALIZATION'
            else:
                p=self.odom.pose.pose.position
                self.position=self.map_from_odom.apply([p.x,p.y])
                self.heading=wrap(yaw(self.odom.pose.pose.orientation)+self.map_from_odom.yaw)
                if self.home_position is None:
                    self.home_position=self.position.copy();self.home_anchor=self.planner.slam.attach(self.position)
                if not self.trajectory or np.linalg.norm(self.position-self.trajectory[-1][0])>.035:
                    self.trajectory.append([self.position.copy(),self.planner.slam.attach(self.position)])
                    self.trajectory=self.trajectory[-4000:]
                self.update_scan(self.position,self.heading)
                if self.mission_phase=='EXPLORING':linear,angular=self.track_path()
                else:linear,angular=self.control_return(now)
        self.publish_command(linear,angular)
        if now-self.last_status>.5:
            self.last_status=now;self.publish_debug()

    def track_path(self):
        path=self.planner.path()
        if self.view is None:self.state='WAIT_TF';return 0.,0.
        if len(path)<2:self.state=self.planner.state;return 0.,0.
        if self.planner.state in ('OBSERVE_JUNCTION','WAIT_FRONTIER') and self.planner.next_leg is None:
            if np.linalg.norm(path[-1]-self.position)<self.p['goal_tolerance']:
                self.state=self.planner.state;return 0.,0.
        return PolylineFollower.track_reference(self,path,self.p['max_linear_speed'],self.p['lookahead'],
            'BACKTRACK_PP' if self.planner.leg.returning else 'TRACK_PP','ALIGN_PATH')

    def track_reference(self,path,speed,lookahead,moving_state,align_state):
        if self.p.get('controller_mode','pp')!='pp':
            return self.track_motion_reference(path,speed,lookahead,moving_state,align_state)
        twist=self.odom.twist.twist
        predicted,predicted_heading=predict_pose(self.position,self.heading,twist.linear.x,
                                                twist.angular.z,self.p['prediction_time'])
        self.target=pursuit_target(path,predicted,lookahead)
        linear,angular,angle=pure_pursuit(predicted,predicted_heading,self.target,speed)
        threshold=self.p['align_exit_angle'] if self.aligning else self.p['align_enter_angle']
        self.aligning=abs(angle)>threshold
        if self.aligning:
            limit=min(self.p['max_alignment_speed'],self.p['max_angular_speed'])
            self.state=align_state;return 0.,float(np.clip(1.8*angle,-limit,limit))
        limit=self.p['max_angular_speed']
        angular=float(np.clip(angular,-limit,limit))
        angular=yaw_rate_command(angular,twist.angular.z,self.p['yaw_rate_gain'],limit)
        self.state=moving_state
        return linear,angular

    def track_motion_reference(self,path,speed,lookahead,moving_state,align_state):
        started=time.monotonic();twist=self.odom.twist.twist
        cap=self.motion_speed
        if self.mission_phase=='RETURNING' and speed<self.p['max_linear_speed']-.01:cap=min(cap,speed)
        stop=(self.mission_phase=='RETURNING' or
              (self.planner.leg is not None and self.planner.leg.terminal and self.planner.next_leg is None))
        reference=self.motion_builder.build(path,self.position,cap,stop,observation=self.view)
        self.motion_report=dict(self.motion_builder.report)
        if reference is None:
            self.state='REFERENCE_WAIT_KNOWN';self.motion_path=[]
            self.motion_report['controller']='waiting';return 0.,0.
        self.motion_path=[p.copy() for p in reference.xy]
        progress,projection=project(reference.xy,self.position)
        cross_track=float(np.linalg.norm(self.position-projection))
        curve_speed=float(np.interp(progress,reference.s,reference.speed))
        predicted,predicted_heading=predict_pose(self.position,self.heading,twist.linear.x,
                                                twist.angular.z,self.p['prediction_time'])
        self.target=pursuit_target(self.motion_path,predicted,lookahead)
        _,_,angle=pure_pursuit(predicted,predicted_heading,self.target,curve_speed)
        if self.motion_mpc is not None:
            tangent_error=wrap(float(np.interp(progress,reference.s,reference.yaw))-self.heading)
            recovery_angle=angle if cross_track>.16 else tangent_error
            threshold=(self.p.get('mpc_align_exit_angle',.50) if self.motion_recovering
                       else self.p.get('mpc_align_enter_angle',.85))
            self.motion_recovering=abs(recovery_angle)>threshold
            self.motion_report.update(cross_track_m=cross_track,tangent_error_rad=tangent_error)
            if self.motion_recovering:
                self.aligning=False;self.state=align_state
                self.motion_report['controller']='tangent_align'
                self.motion_mpc.reset()
                return 0.,float(np.clip(1.8*recovery_angle,-self.p['max_alignment_speed'],self.p['max_alignment_speed']))
        threshold=self.p['align_exit_angle'] if self.aligning else self.p['align_enter_angle']
        self.aligning=abs(angle)>threshold
        if self.aligning:
            self.state=align_state
            self.motion_report['controller']='align'
            if self.motion_mpc is not None:self.motion_mpc.reset()
            return 0.,float(np.clip(1.8*angle,-self.p['max_alignment_speed'],self.p['max_alignment_speed']))
        if self.motion_mpc is not None:
            # One current map->odom transform for reference and prediction;
            # odometry gives a continuous frame through SLAM corrections.
            p=self.odom.pose.pose.position;odom_position=np.array([p.x,p.y])
            odom_heading=yaw(self.odom.pose.pose.orientation)
            command,prediction,ref,report=self.motion_mpc.compute(reference,self.position,
                odom_position,odom_heading,self.map_from_odom,twist.linear.x,twist.angular.z,
                self.command,grid=self.motion_builder.grid)
            self.motion_report.update(report)
            self.mpc_prediction=prediction;self.mpc_reference=ref
            if command is not None:
                self.state='RETURN_MPC' if self.mission_phase=='RETURNING' else ('BACKTRACK_MPC' if self.planner.leg.returning else 'TRACK_MPC')
                self.motion_report['controller']='mpc'
                self.motion_report['total_ms']=(time.monotonic()-started)*1000
                return command
        linear,angular,_=pure_pursuit(predicted,predicted_heading,self.target,curve_speed)
        limit=self.p['max_angular_speed'];angular=float(np.clip(angular,-limit,limit))
        angular=yaw_rate_command(angular,twist.angular.z,self.p['yaw_rate_gain'],limit)
        if self.motion_mpc is not None:
            from differential_mpc import limit_wheel_command
            if cross_track>.10 or abs(angle)>.45:linear=min(linear,.25)
            linear,angular=limit_wheel_command((linear,angular),self.command)
        self.state=moving_state if self.motion_mpc is None else 'MPC_FALLBACK_PP'
        self.motion_report['controller']='optimized_pp' if self.motion_mpc is None else 'fallback_pp'
        self.motion_report['total_ms']=(time.monotonic()-started)*1000
        return linear,angular

    def control_return(self,now):
        if self.home_position is not None and np.linalg.norm(self.position-self.home_position)<=self.p['return_goal_tolerance']:
            self.mission_phase=self.state='HOME_REACHED';self.return_active=[];self.target=None
            self.return_reason='A* return completed; robot is back at the start'
            self.result_pub.publish(String(data=json.dumps(dict(state=self.state,success=True,finished=True,
                reason=self.return_reason,frame=self.frame,position=self.position.tolist(),home=self.home_position.tolist()))))
            self.get_logger().info('HOME REACHED / 已返回起点：A* 返航完成，停车。')
            return 0.,0.
        if self.mission_phase!='RETURNING':
            self.state=self.mission_phase if self.mission_phase=='RETURN_NO_PATH' else 'PLAN_RETURN'
            if self.return_future is not None and self.return_future.done():
                try:plan=self.return_future.result()
                except Exception as error:
                    self.return_reason=f'A* planning failed: {error}';plan=None
                self.return_future=None
                if plan is not None:
                    self.return_reason=plan.reason;self.return_expanded=plan.expanded
                if plan is not None and plan.points:
                    self.return_path=[p.copy() for p in plan.points]
                    self.return_anchors=[self.planner.slam.attach(p) for p in self.return_path]
                    self.return_index=0;self.return_plan_length=plan.length;self.aligning=False
                    self.mission_phase='RETURNING'
                    self.result_pub.publish(String(data=json.dumps(dict(state='RETURN_PLANNED',success=True,finished=False,
                        algorithm='A*',length_m=plan.length,expanded=plan.expanded,home=self.home_position.tolist()))))
                    self.get_logger().info(f'A* 返航路线已生成：{plan.length:.2f} m，搜索 {plan.expanded} 个栅格。')
                else:
                    self.mission_phase=self.state='RETURN_NO_PATH'
                    self.return_next_plan=now+self.p['return_retry_period']
                    self.get_logger().warn(self.return_reason)
            if self.mission_phase!='RETURNING':
                if now>=self.return_ready_at and now>=self.return_next_plan and self.return_future is None:
                    if self.latest_map is None or self.home_position is None:
                        self.state='RETURN_WAIT_MAP';self.return_reason='waiting for a SLAM map and recorded home pose'
                    else:
                        message=self.latest_map;start=self.position.copy();home=self.home_position.copy();view=self.view
                        def compute():
                            info=message.info;p=info.origin.position
                            grid=ReturnGrid(np.asarray(message.data,dtype=np.int16).reshape(info.height,info.width),
                                info.resolution,[p.x,p.y],yaw(info.origin.orientation),
                                clearance=self.p['return_clearance'],wall_cost=self.p['return_wall_cost'],observation=view)
                            return grid.plan(start,home)
                        self.return_future=self.return_executor.submit(compute);self.return_attempts+=1
                        self.mission_phase=self.state='PLAN_RETURN';self.return_reason='A* search on the observed inflated map'
                return 0.,0.
        # Advance monotonically along the A* reference instead of rediscovering
        # maze branches on the way home.
        best=(float('inf'),self.return_index)
        for i in range(self.return_index,len(self.return_path)-1):
            a,b=self.return_path[i:i+2];d=b-a
            t=float(np.clip((self.position-a)@d/max(float(d@d),1e-9),0.,1.))
            error=float(np.linalg.norm(self.position-(a+t*d)))
            if error<best[0]:best=error,i
        self.return_index=best[1];self.return_active=self.return_path[self.return_index:]
        remaining=float(np.linalg.norm(self.position-self.return_active[1]))+sum(
            float(np.linalg.norm(b-a)) for a,b in zip(self.return_active[1:-1],self.return_active[2:]))
        speed=self.p['max_linear_speed']
        if remaining<.6:
            speed=min(speed,self.p['return_goal_gain']*float(np.linalg.norm(self.position-self.home_position)))
        return self.track_reference(self.return_active,speed,self.p['return_lookahead'],'RETURN_PP','RETURN_ALIGN')

    def pose(self,point):
        message=PoseStamped();message.header.frame_id=self.frame
        message.header.stamp=self.get_clock().now().to_msg()
        message.pose.position.x=float(point[0]);message.pose.position.y=float(point[1])
        message.pose.orientation.w=1.;return message

    def path_message(self,points):
        message=Path();message.header=self.pose([0.,0.]).header
        message.poses=[self.pose(point) for point in points];return message

    def publish_debug(self):
        planner=self.planner
        report=dict(state=self.state,reason=self.return_reason if self.mission_phase!='EXPLORING' else planner.reason,
                    mission_phase=self.mission_phase,finished=self.mission_phase in ('EXIT_REACHED','HOME_REACHED'),
                    exit_reached=self.exit_detector.reached,exit_detection=self.exit_detector.report(),
                    return_home_active=self.mission_phase=='RETURNING',return_plan_length_m=self.return_plan_length,
                    return_expanded=self.return_expanded,return_attempts=self.return_attempts,
                    distance_m=round(self.distance,3),
                    junctions=len(planner.nodes),backtracks=planner.backtracks,
                    corners_planned=planner.turns,left_openings=planner.openings-planner.right_openings,
                    right_openings=planner.right_openings,
                    map_dead_end_branches=sum(edge.map_dead_end and not edge.visited for edge in planner.edges if not edge.retired),
                    map_forward_turnbacks=planner.map_forward_turnbacks,
                    last_map_turnback=planner.last_map_turnback,
                    return_discoveries=planner.return_discoveries,
                    side_junctions=planner.side_junctions,
                    linear_m_s=self.command[0],angular_rad_s=self.command[1],
                    wall_clearance_m=self.p['wall_clearance'],frame=self.frame,
                    edges=sum(not edge.retired for edge in planner.edges),pending_branches=sum(bool(planner.untried(i)) for i in range(len(planner.nodes))),
                    backtrack_target=planner.return_target,backtrack_edge_ids=planner.last_return_edges,
                    map_revisions=planner.map_revisions,wall_revisions=planner.wall_revisions,
                    slam_keyframes=len(planner.slam.poses),
                    map_odom_xy=self.map_from_odom.translation.tolist(),map_odom_yaw=self.map_from_odom.yaw)
        report.update(controller_mode=self.p['controller_mode'],motion=self.motion_report,
                      enabled=self.enabled,waiting_for_start=self.waiting_for_start)
        if planner.leg is not None:
            report.update(segment_end=planner.leg.end.tolist(),segment_direction=planner.leg.heading,
                          terminal_wall=planner.leg.terminal,returning=planner.leg.returning,active_edge_id=planner.leg.edge_id)
        if self.position is not None:
            report.update(position=self.position.tolist(),yaw=self.heading,
                          yaw_rate=self.odom.twist.twist.angular.z)
        if self.target is not None:report['lookahead_point']=self.target.tolist()
        if self.exit_position is not None:report['exit_position']=self.exit_position.tolist()
        if self.home_position is not None:
            report['home_position']=self.home_position.tolist()
            report['home_distance_m']=float(np.linalg.norm(self.position-self.home_position)) if self.position is not None else None
        status=String(data=json.dumps(report));self.status_pub.publish(status);self.compat_status.publish(status)
        self.graph_pub.publish(String(data=json.dumps(dict(frame=self.frame,**planner.graph_report()))))
        key=(self.state,len(planner.nodes),planner.backtracks,planner.turns,
             report['map_dead_end_branches'],planner.map_forward_turnbacks)
        if key!=self.last_log:
            self.get_logger().info(status.data);self.last_log=key
        self.path_pub.publish(self.path_message(planner.global_path()))
        active=self.motion_path if self.p['controller_mode']!='pp' and self.motion_path else (
            planner.path() if self.mission_phase=='EXPLORING' else self.return_active)
        self.active_pub.publish(self.path_message(active))
        self.optimized_pub.publish(self.path_message(self.motion_path))
        self.predicted_pub.publish(self.path_message(self.mpc_prediction))
        self.mpc_ref_pub.publish(self.path_message(self.mpc_reference))
        self.return_pub.publish(self.path_message(self.return_path))
        self.track_pub.publish(self.path_message([sample[0] for sample in self.trajectory]))
        if self.target is not None:self.target_pub.publish(self.pose(self.target))
        array=MarkerArray()
        marker=Marker();marker.header=self.pose([0.,0.]).header;marker.ns='junctions';marker.id=0
        marker.type=Marker.SPHERE_LIST;marker.action=Marker.ADD;marker.pose.orientation.w=1.
        marker.scale.x=marker.scale.y=marker.scale.z=.065;marker.color.b=1.;marker.color.a=1.
        marker.points=[Point(x=float(node.point[0]),y=float(node.point[1]),z=.04) for node in planner.nodes]
        array.markers.append(marker)
        # A Path cannot represent disconnected branches without fake links.
        # Draw every stored undirected edge exactly once with LINE_LIST.
        segments=planner.graph_segments()
        pruned={edge.edge_id for edge in planner.edges if edge.map_dead_end and not edge.visited}
        for group,name,color in [('visited','visited_roads',(0.15,.35,.95)),
                                 ('unexplored','unexplored_roads',(.0,.75,.85)),
                                 ('dead_end','map_dead_end_roads',(.5,.5,.5))]:
            roads=Marker();roads.header=marker.header;roads.ns=name;roads.id=0
            roads.type=Marker.LINE_LIST;roads.action=Marker.ADD;roads.pose.orientation.w=1.
            roads.scale.x=.025;roads.color.r,roads.color.g,roads.color.b=color;roads.color.a=1.
            for edge_id,a,b,explored in segments:
                category='dead_end' if edge_id in pruned else ('visited' if explored else 'unexplored')
                if category==group:
                    roads.points.extend([Point(x=float(a[0]),y=float(a[1]),z=.03),Point(x=float(b[0]),y=float(b[1]),z=.03)])
            array.markers.append(roads)
        target=Marker();target.header=marker.header;target.ns='return_target';target.id=0
        target.type=Marker.SPHERE;target.pose.orientation.w=1.
        target.scale.x=target.scale.y=target.scale.z=.13;target.color.r=1.;target.color.b=.8;target.color.a=1.
        target.action=Marker.DELETE if planner.return_target is None or self.exit_detector.reached else Marker.ADD
        if planner.return_target is not None:
            p=planner.nodes[planner.return_target].point;target.pose.position.x=float(p[0]);target.pose.position.y=float(p[1])
        array.markers.append(target)
        result=Marker();result.header=marker.header;result.ns='completion';result.id=0
        result.type=Marker.TEXT_VIEW_FACING;result.pose.orientation.w=1.
        result.action=Marker.ADD if self.exit_detector.reached else Marker.DELETE
        result.scale.z=.22;result.color.r=.1;result.color.g=.8;result.color.b=.2;result.color.a=1.
        result.text='MAZE EXIT REACHED\nRobot stopped';label_point=self.exit_position
        if self.mission_phase=='HOME_REACHED':
            result.text='HOME REACHED\nMission complete';label_point=self.home_position
        elif self.mission_phase=='RETURNING':
            result.text='A* RETURN TO START';label_point=self.position
            result.color.r=.65;result.color.g=.15;result.color.b=1.
        elif self.exit_detector.reached and self.p['return_home_enabled']:
            result.text='EXIT FOUND\nPlanning A* return'
            result.color.r=1.;result.color.g=.65;result.color.b=.05
        if label_point is not None:
            result.pose.position=Point(x=float(label_point[0]),y=float(label_point[1]),z=.6)
        array.markers.append(result)
        home=Marker();home.header=marker.header;home.ns='home';home.id=0
        home.type=Marker.SPHERE;home.pose.orientation.w=1.
        home.action=Marker.ADD if self.home_position is not None else Marker.DELETE
        home.scale.x=home.scale.y=home.scale.z=.15;home.color.g=1.;home.color.a=1.
        if self.home_position is not None:
            home.pose.position=Point(x=float(self.home_position[0]),y=float(self.home_position[1]),z=.08)
        array.markers.append(home)
        self.markers.publish(array)


def main():
    rclpy.init();node=PolylineFollower()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if rclpy.ok():node.publish_command(0.,0.)
        node.return_executor.shutdown(wait=False,cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
