// Reference: https://github.com/Zythyra/ifly2_2026/blob/main/src/line_follow/src/left2.cpp
// ROS2 adaptation: continuous lidar boundary points replace camera pixels.
#include "maze_wall_pid/left2_trace.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;
using left2_lidar::Point;

class Left2Lidar : public rclcpp::Node {
  using Scan=sensor_msgs::msg::LaserScan;
  using Marker=visualization_msgs::msg::Marker;
  using Steady=std::chrono::steady_clock;
  enum class ExitPhase { FOLLOW, VERIFY_OPEN, LEAVING, BRAKING, DONE };
public:
  Left2Lidar():Node("left2_lidar"),buffer_(get_clock()),listener_(buffer_) {
    enabled_=declare_parameter("enabled",true);
    waiting_start_=declare_parameter("wait_for_start",false);
    if (waiting_start_) enabled_=false;
    speed_=declare_parameter("linear_speed",.20);
    const double road_width=declare_parameter("road_width",.60);
    corridor_width_=road_width;
    offset_=declare_parameter("target_left_distance",road_width/2.);
    center_corridor_=declare_parameter("center_corridor",false);
    width_min_=declare_parameter("corridor_width_min",.60);
    width_max_=declare_parameter("corridor_width_max",2.8);
    width_filter_=declare_parameter("width_filter_time",.60);
    width_rate_=declare_parameter("width_change_rate",.40);
    horizon_=declare_parameter("trace_horizon",.85);
    center_horizon_=declare_parameter("center_trace_horizon",.0);
    tangent_window_=declare_parameter("tangent_fit_window",.0);
    geometric_control_=declare_parameter("geometric_line_control",false);
    dual_wall_geometry_=declare_parameter("dual_wall_geometry",false);
    heading_gain_=declare_parameter("heading_gain",3.0);
    curvature_filter_=declare_parameter("curvature_filter_time",.20);
    curvature_limit_=declare_parameter("curvature_limit",1.2);
    step_=declare_parameter("trace_step",.015);
    max_left_=declare_parameter("max_left_distance",.65);
    turn_speed_=declare_parameter("turn_angular_speed",.80);
    right_limit_=declare_parameter("right_angular_limit",1.8);
    front_turn_=declare_parameter("front_turn_distance",.60);
    emergency_=declare_parameter("emergency_distance",.23);
    timeout_=declare_parameter("scan_timeout",.7);
    yaw_gain_=declare_parameter("yaw_rate_gain",.0);
    angular_acceleration_=declare_parameter("angular_acceleration_limit",2.0);
    exit_enabled_=declare_parameter("exit_detection_enabled",false);
    exit_clearance_=declare_parameter("exit_clearance",1.4);
    exit_confirm_=declare_parameter("exit_confirm_time",.8);
    exit_min_travel_=declare_parameter("exit_min_travel",5.0);
    exit_open_confirm_=declare_parameter("exit_open_confirm_time",.30);
    exit_clear_distance_=declare_parameter("exit_clear_distance",.30);
    loss_frames_=declare_parameter("lost_confirm_frames",2);
    debug_window_=declare_parameter("debug_window",true);
    pid_.p=declare_parameter("pid_p",4.);
    pid_.i=declare_parameter("pid_i",0.);
    pid_.d=declare_parameter("pid_d",12.);
    pid_.limit=declare_parameter("angular_limit",1.1);
    pid_.integral_limit=declare_parameter("integral_limit",.15);
    if (speed_<=0 || speed_>1.5 || offset_<=.15 || step_<=0 || horizon_<=step_ ||
        front_turn_<=emergency_ || turn_speed_<=0 || right_limit_<=0 || angular_acceleration_<=0 || yaw_gain_<0 ||
        exit_confirm_<=0 || exit_clearance_<=offset_ || exit_min_travel_<0 ||
        width_min_<=.3 || width_max_<=width_min_ || width_filter_<=0 || width_rate_<=0 ||
        center_horizon_<0 || (center_horizon_>0 && center_horizon_<=step_) ||
        tangent_window_<0 || heading_gain_<=0 || curvature_filter_<=0 || curvature_limit_<=0 ||
        exit_open_confirm_<=0 || exit_clear_distance_<=0 ||
        (center_corridor_ && (road_width<width_min_ || road_width>width_max_)))
      throw std::invalid_argument("Invalid Burger speed, road width or trace window");
    cmd_pub_=create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",1);
    marker_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("/wall_follow/markers",1);
    image_pub_=create_publisher<sensor_msgs::msg::Image>("/wall_follow/debug_image",1);
    status_pub_=create_publisher<std_msgs::msg::String>("/wall_follow/status",1);
    result_pub_=create_publisher<std_msgs::msg::String>("/wall_follow/result",rclcpp::QoS(1).transient_local());
    old_status_pub_=create_publisher<std_msgs::msg::String>("/pp/status",1);
    path_pub_=create_publisher<nav_msgs::msg::Path>("/wall_follow/trajectory",1);
    scan_sub_=create_subscription<Scan>("/scan",rclcpp::SensorDataQoS(),
      std::bind(&Left2Lidar::on_scan,this,std::placeholders::_1));
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr m) {
        const auto &p=m->pose.pose.position;
        const auto &q=m->pose.pose.orientation;
        const double heading=std::atan2(2.*(q.w*q.z+q.x*q.y),1.-2.*(q.y*q.y+q.z*q.z));
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(heading) ||
            !std::isfinite(m->twist.twist.angular.z) || !std::isfinite(m->twist.twist.linear.x)) return;
        last_odom_=Steady::now();measured_angular_=m->twist.twist.angular.z;
        measured_linear_=m->twist.twist.linear.x;last_yaw_=heading;
        if (have_odom_) distance_+=std::hypot(p.x-last_x_,p.y-last_y_);
        last_x_=p.x;last_y_=p.y;have_odom_=true;
        if (path_.poses.empty() || std::hypot(p.x-path_.poses.back().pose.position.x,
                                            p.y-path_.poses.back().pose.position.y)>.04) {
          geometry_msgs::msg::PoseStamped pose;pose.header=m->header;pose.pose=m->pose.pose;
          path_.poses.push_back(pose);
          if (path_.poses.size()>10000) path_.poses.erase(path_.poses.begin());
        }
      });
    const auto enable=[this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                           std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      if (finished_) {
        command_=geometry_msgs::msg::Twist{};cmd_pub_->publish(command_);
        response->success=!request->data;response->message="exit reached; restart launch for a new run";return;
      }
      waiting_start_=false;reset_exit();
      enabled_=request->data;pid_.reset();lost_=0;last_angular_=0.;command_=geometry_msgs::msg::Twist{};
      state_=finished_?"EXIT_REACHED":(enabled_?"WAIT_SCAN":"PAUSED");cmd_pub_->publish(command_);
      response->success=true;response->message=state_;
    };
    enable_service_=create_service<std_srvs::srv::SetBool>("/wall_follow/set_enabled",enable);
    old_enable_service_=create_service<std_srvs::srv::SetBool>("/pp/set_enabled",enable);
    timer_=create_wall_timer(50ms,[this]() {
      if (finished_) {command_=geometry_msgs::msg::Twist{};set_state("EXIT_REACHED");}
      else if (!enabled_) { command_=geometry_msgs::msg::Twist{};set_state(waiting_start_?"WAIT_START":"PAUSED"); }
      else if (std::chrono::duration<double>(Steady::now()-last_scan_).count()>timeout_) {
        command_=geometry_msgs::msg::Twist{};set_state("WAIT_SCAN");pid_.reset();last_angular_=0.;reset_exit();
      } else if (std::chrono::duration<double>(Steady::now()-last_odom_).count()>timeout_) {
        command_=geometry_msgs::msg::Twist{};set_state("WAIT_ODOM");pid_.reset();last_angular_=0.;reset_exit();
      }
      cmd_pub_->publish(command_);
    });
    RCLCPP_INFO(get_logger(),"left2 lidar: continuous boundary -> weighted error -> PID; v=%.2f, offset=%.2f",speed_,offset_);
  }
  ~Left2Lidar() override { if (debug_window_) cv::destroyAllWindows(); }
private:
  double sector(const Scan& scan,double center,double half) {
    std::vector<double> values;
    size_t expected=0;
    for (size_t i=0;i<scan.ranges.size();++i) {
      const double angle=scan.angle_min+i*scan.angle_increment;
      if (std::abs(std::atan2(std::sin(angle-center),std::cos(angle-center)))>half) continue;
      ++expected;
      double r=scan.ranges[i];
      if (std::isinf(r) && r>0) r=scan.range_max;
      if (std::isfinite(r) && r>=scan.range_min && r<=scan.range_max) values.push_back(r);
    }
    if (expected<3 || values.size()<.8*expected) return -1.;
    return *std::min_element(values.begin(),values.end());
  }
  void set_state(const std::string& state) {
    state_=state;
  }
  void report_state() {
    if (state_!=logged_state_) {RCLCPP_INFO(get_logger(),"%s",state_.c_str());logged_state_=state_;}
  }
  void update_corridor(double dt) {
    const auto left=left2_lidar::fit_side_wall(points_,1);
    const auto right=left2_lidar::fit_side_wall(points_,-1);
    wall_left_=left.valid?left.distance:-1.;wall_right_=right.valid?right.distance:-1.;
    const double width=left.distance+right.distance;
    width_observed_=left.valid && right.valid && std::abs(left.angle-right.angle)<.35 &&
      width>=width_min_ && width<=width_max_;
    // A curved wall's short chord can look like a wider straight corridor.
    // Only learn width from two locally straight, similarly oriented faces.
    if (geometric_control_) width_observed_=width_observed_ && left.rms<.012 && right.rms<.012 &&
      std::abs(left.angle)<.35 && std::abs(right.angle)<.35 && std::abs(left.angle-right.angle)<.15;
    have_pair_=have_pair_ || width_observed_;
    if (!center_corridor_) return;
    // Use perpendicular wall distances, not two oblique range readings.
    // At an opening or turn retain the last reliable width, never range_max.
    if (width_observed_) {
      const double delta=(width-corridor_width_)*(1.-std::exp(-dt/width_filter_));
      corridor_width_+=std::clamp(delta,-width_rate_*dt,width_rate_*dt);
    }
    offset_=corridor_width_/2.;
  }
  void reset_exit() {
    exit_phase_=ExitPhase::FOLLOW;exit_since_=-1.;exit_stop_since_=-1.;exit_progress_=0.;
  }
  bool handle_exit(const Scan& scan,double stamp) {
    if (!exit_enabled_) return false;
    const bool full_scan=(scan.ranges.size()-1)*std::abs(scan.angle_increment)>=
      2.*M_PI-2.*std::abs(scan.angle_increment);
    const bool open=have_wall_ && have_pair_ && distance_>=exit_min_travel_ && full_scan &&
      !width_observed_ && !trace_.found && front_>exit_clearance_ &&
      exit_left_>exit_clearance_ && exit_right_>exit_clearance_;
    if (exit_phase_==ExitPhase::FOLLOW) {
      if (!open) return false;
      exit_phase_=ExitPhase::VERIFY_OPEN;exit_since_=stamp;
      exit_x_=last_x_;exit_y_=last_y_;exit_heading_=last_yaw_;
    }
    if (exit_phase_==ExitPhase::VERIFY_OPEN) {
      if (!open) {reset_exit();return false;}
      if (stamp-exit_since_>=exit_open_confirm_-1e-9) exit_phase_=ExitPhase::LEAVING;
    }
    exit_progress_=(last_x_-exit_x_)*std::cos(exit_heading_)+(last_y_-exit_y_)*std::sin(exit_heading_);
    if (exit_phase_==ExitPhase::LEAVING && (width_observed_ || front_<front_turn_)) {
      reset_exit();return false;
    }
    pid_.reset();lost_=0;
    if (exit_phase_==ExitPhase::VERIFY_OPEN || exit_phase_==ExitPhase::LEAVING) {
      // Clear the end of both walls rather than turning around the exit post.
      command_.linear.x=speed_;
      const double error=std::atan2(std::sin(exit_heading_-last_yaw_),std::cos(exit_heading_-last_yaw_));
      command_.angular.z=std::clamp(2.*error,-.5,.5);
      set_state(exit_phase_==ExitPhase::VERIFY_OPEN?"EXIT_VERIFY_OPEN":"EXIT_LEAVING");
      if (exit_phase_==ExitPhase::LEAVING && exit_progress_>=exit_clear_distance_) {
        exit_phase_=ExitPhase::BRAKING;command_=geometry_msgs::msg::Twist{};
      }
    }
    if (exit_phase_==ExitPhase::BRAKING) {
      command_=geometry_msgs::msg::Twist{};set_state("EXIT_BRAKING");
      if (std::abs(measured_linear_)<.02 && std::abs(measured_angular_)<.10) {
        if (exit_stop_since_<0.) exit_stop_since_=stamp;
        if (stamp-exit_stop_since_>=exit_confirm_-1e-9) {
          finished_=true;enabled_=false;exit_phase_=ExitPhase::DONE;set_state("EXIT_REACHED");
          std_msgs::msg::String result;
          result.data="{\"state\":\"EXIT_REACHED\",\"finished\":true,\"success\":true,\"distance_m\":"+
            std::to_string(distance_)+",\"exit_clear_distance_m\":"+std::to_string(exit_progress_)+
            ",\"x\":"+std::to_string(last_x_)+",\"y\":"+std::to_string(last_y_)+"}";
          result_pub_->publish(result);
          RCLCPP_INFO(get_logger(),"EXIT REACHED / 已驶离出口 %.2f m，确认停稳，任务完成。",exit_progress_);
        }
      } else exit_stop_since_=-1.;
    }
    return true;
  }
  void on_scan(const Scan::SharedPtr scan) {
    if (finished_) {
      command_=geometry_msgs::msg::Twist{};set_state("EXIT_REACHED");
      cmd_pub_->publish(command_);publish_debug(scan->header.stamp);return;
    }
    const auto steady_now=Steady::now();
    const int64_t scan_ns=rclcpp::Time(scan->header.stamp).nanoseconds();
    if (scan_ns==last_scan_ns_) return;
    if (scan_ns<last_scan_ns_ || std::chrono::duration<double>(steady_now-last_scan_).count()>timeout_) {
      pid_.reset();last_angular_=0.;reset_exit();
    }
    last_scan_ns_=scan_ns;
    const double dt=std::clamp(std::chrono::duration<double>(steady_now-last_scan_).count(),.03,.3);
    last_scan_=steady_now;
    points_.clear();
    if (scan->ranges.size()<32 || !std::isfinite(scan->angle_min) ||
        !std::isfinite(scan->angle_increment) || std::abs(scan->angle_increment)<1e-8 ||
        !std::isfinite(scan->range_min) || !std::isfinite(scan->range_max) ||
        scan->range_min<0 || scan->range_max<=scan->range_min) {
      command_=geometry_msgs::msg::Twist{};set_state("INVALID_SCAN");pid_.reset();last_angular_=0.;reset_exit();
      cmd_pub_->publish(command_);report_state();return;
    }
    try {
      const auto message=buffer_.lookupTransform("base_footprint",scan->header.frame_id,tf2::TimePointZero);
      tf2::Transform transform;tf2::fromMsg(message.transform,transform);
      for (size_t i=0;i<scan->ranges.size();++i) {
        const double r=scan->ranges[i];
        if (!std::isfinite(r) || r<scan->range_min || r>=scan->range_max) continue;
        const double angle=scan->angle_min+i*scan->angle_increment;
        const auto point=transform*tf2::Vector3(r*std::cos(angle),r*std::sin(angle),0.);
        points_.push_back({point.x(),point.y()});
      }
    } catch (const tf2::TransformException&) {
      command_=geometry_msgs::msg::Twist{};set_state("WAIT_LASER_TF");pid_.reset();last_angular_=0.;reset_exit();
      cmd_pub_->publish(command_);return;
    }
    // Wrapped angles support Burger's 0..2pi layout, including -15 degrees.
    // Use full-sector minima: front +/-15 deg, left 60..90 deg, right +/-15 deg.
    front_=sector(*scan,0.,M_PI/12.);left_=sector(*scan,5.*M_PI/12.,M_PI/12.);
    right_=sector(*scan,-M_PI/2.,M_PI/12.);
    // Check actual flanks for wall endings. The broad 60..90 deg left
    // tracking sector can still see an outdoor pillar far ahead of the exit.
    exit_left_=sector(*scan,M_PI/2.,8.*M_PI/180.);
    exit_right_=sector(*scan,-M_PI/2.,8.*M_PI/180.);
    update_corridor(dt);
    trace_=left2_lidar::trace_edge(points_,offset_,horizon_,step_,max_left_,center_horizon_,tangent_window_);
    geometry_=geometric_control_?left2_lidar::wall_geometry(trace_.edge,offset_):
      left2_lidar::line_geometry(trace_.center);
    geometry_source_=geometry_.valid?1:0;right_trace_=left2_lidar::Trace{};
    if (geometric_control_ && dual_wall_geometry_ && center_corridor_) {
      auto mirrored=points_;for(auto &p:mirrored) p.y=-p.y;
      right_trace_=left2_lidar::trace_edge(mirrored,offset_,horizon_,step_,max_left_,center_horizon_,tangent_window_);
      const auto corridor=left2_lidar::corridor_geometry(trace_,right_trace_,offset_);
      geometry_=corridor.geometry;geometry_source_=corridor.source;
      for(auto &p:right_trace_.edge) p.y=-p.y;
      for(auto &p:right_trace_.center) p.y=-p.y;
    }
    command_=geometry_msgs::msg::Twist{};
    if (finished_) set_state("EXIT_REACHED");
    else if (!enabled_) {reset_exit();set_state(waiting_start_?"WAIT_START":"PAUSED");}
    else if (front_<0 || left_<0 || right_<0) {reset_exit();pid_.reset();set_state("INVALID_SCAN");}
    else if (!have_odom_ || std::chrono::duration<double>(steady_now-last_odom_).count()>timeout_) {
      reset_exit();pid_.reset();set_state("WAIT_ODOM");
    }
    else {
      have_wall_=have_wall_ || trace_.found;
      const double stamp=scan_ns*1e-9;
      const bool track_found=trace_.found || (dual_wall_geometry_ && geometry_.valid);
      lost_=track_found ? 0 : lost_+1;
      if (handle_exit(*scan,stamp)) {
        // Exit verification, clearing and braking own the command until done.
      } else if (front_<front_turn_) {
        // Front danger takes priority over lost-left recovery, including the
        // right-hand bend of an S corridor. Do not blindly turn left here.
        pid_.reset();command_.angular.z=-std::min(turn_speed_,right_limit_);
        set_state(front_<emergency_?"CLEARANCE_RECOVERY":"FRONT_TURN_RIGHT");
      } else if (lost_>=loss_frames_) {
        command_.linear.x=speed_;
        pid_.reset();command_.angular.z=turn_speed_;set_state("LEFT_LINE_LOST");
      } else if (track_found) {
        command_.linear.x=speed_;
        const bool geometric=geometric_control_ && geometry_.valid;
        if (geometric!=was_geometric_) {pid_.reset();was_geometric_=geometric;}
        double desired;
        if (geometric) {
          const double curvature=std::clamp(geometry_.curvature,-curvature_limit_,curvature_limit_);
          filtered_curvature_+=(curvature-filtered_curvature_)*(1.-std::exp(-dt/curvature_filter_));
          desired=speed_*filtered_curvature_+heading_gain_*speed_*geometry_.heading+pid_.update(geometry_.lateral,dt);
        } else {
          filtered_curvature_=0.;desired=pid_.update(trace_.error,dt);
        }
        const double damped=std::clamp(desired+yaw_gain_*(desired-measured_angular_),-pid_.limit,pid_.limit);
        command_.angular.z=last_angular_+std::clamp(damped-last_angular_,-angular_acceleration_*dt,angular_acceleration_*dt);
        set_state(geometric?"TRACE_CURVATURE":"TRACE_PID");
      } else {
        command_.linear.x=speed_;
        command_.angular.z=last_angular_;set_state("TRACE_GAP");
      }
    }
    report_state();
    last_angular_=command_.angular.z;
    cmd_pub_->publish(command_);
    publish_debug(scan->header.stamp);
  }
  Marker marker(int id,int type,const rclcpp::Time& stamp) {
    Marker m;m.header.frame_id="base_footprint";m.header.stamp=stamp;m.ns="left2_lidar";m.id=id;
    m.type=type;m.action=Marker::ADD;m.pose.orientation.w=1.;m.color.a=1.;
    m.scale.x=m.scale.y=.025;return m;
  }
  void publish_debug(const rclcpp::Time& stamp) {
    visualization_msgs::msg::MarkerArray array;
    auto edge=marker(0,Marker::SPHERE_LIST,stamp);edge.color.g=1.;
    auto center=marker(1,Marker::LINE_STRIP,stamp);center.color.r=1.;center.color.g=.8;center.scale.x=.02;
    auto right=marker(3,Marker::SPHERE_LIST,stamp);right.color.b=1.;right.color.g=.5;
    for (auto p:trace_.edge) {geometry_msgs::msg::Point v;v.x=p.x;v.y=p.y;v.z=.035;edge.points.push_back(v);}
    const auto &active_center=geometry_source_==2?right_trace_.center:trace_.center;
    for (auto p:active_center) {geometry_msgs::msg::Point v;v.x=p.x;v.y=p.y;v.z=.04;center.points.push_back(v);}
    for (auto p:right_trace_.edge) {geometry_msgs::msg::Point v;v.x=p.x;v.y=p.y;v.z=.035;right.points.push_back(v);}
    if (edge.points.empty()) edge.action=Marker::DELETE;
    if (center.points.empty()) center.action=Marker::DELETE;
    if (right.points.empty()) right.action=Marker::DELETE;
    auto completion=marker(2,Marker::TEXT_VIEW_FACING,stamp);
    completion.action=finished_?Marker::ADD:Marker::DELETE;
    completion.pose.position.z=.5;completion.scale.z=.18;
    completion.color.g=1.;completion.text="EXIT REACHED\nRobot stopped";
    array.markers={edge,center,completion,right};marker_pub_->publish(array);
    std::ostringstream s;s<<std::fixed<<std::setprecision(4);
    s<<"{\"state\":\""<<state_<<"\",\"enabled\":"<<(enabled_?"true":"false")
     <<",\"finished\":"<<(finished_?"true":"false")<<",\"trace_points\":"<<trace_.edge.size()
     <<",\"line_error_m\":"<<trace_.error<<",\"p_term\":"<<pid_.p_term
     <<",\"i_term\":"<<pid_.i_term<<",\"d_term\":"<<pid_.d_term
     <<",\"linear_m_s\":"<<command_.linear.x<<",\"angular_rad_s\":"<<command_.angular.z
     <<",\"front_m\":"<<front_<<",\"left_m\":"<<left_<<",\"right_m\":"<<right_
     <<",\"exit_left_m\":"<<exit_left_<<",\"exit_right_m\":"<<exit_right_
     <<",\"target_left_m\":"<<offset_<<",\"distance_m\":"<<distance_
     <<",\"corridor_width_m\":"<<corridor_width_<<",\"width_observed\":"<<(width_observed_?"true":"false")
     <<",\"left_wall_perpendicular_m\":"<<wall_left_<<",\"right_wall_perpendicular_m\":"<<wall_right_
     <<",\"geometry_valid\":"<<(geometry_.valid?"true":"false")<<",\"lateral_error_m\":"<<geometry_.lateral
     <<",\"geometry_source\":"<<geometry_source_<<",\"right_trace_points\":"<<right_trace_.edge.size()
     <<",\"heading_error_rad\":"<<geometry_.heading<<",\"curvature_m_inv\":"<<geometry_.curvature
     <<",\"filtered_curvature_m_inv\":"<<filtered_curvature_<<",\"curve_fit_rms_m\":"<<geometry_.rms
     <<",\"exit_progress_m\":"<<exit_progress_<<"}";
    std_msgs::msg::String report;report.data=s.str();status_pub_->publish(report);old_status_pub_->publish(report);
    if (!path_.poses.empty()) {path_.header=path_.poses.back().header;path_pub_->publish(path_);}
    cv::Mat view(650,700,CV_8UC3,cv::Scalar(28,28,28));
    const auto pixel=[](Point p){return cv::Point(cvRound(350-p.y*300),cvRound(570-p.x*300));};
    cv::line(view,pixel({-.15,0.}),pixel({1.5,0.}),cv::Scalar(100,100,100),1);
    for (auto p:points_) {const auto v=pixel(p);if(v.x>=0&&v.x<view.cols&&v.y>=80&&v.y<view.rows) cv::circle(view,v,1,{150,150,150},-1);}
    for (auto p:trace_.edge) cv::circle(view,pixel(p),3,{0,235,0},-1);
    for (auto p:right_trace_.edge) cv::circle(view,pixel(p),3,{255,150,0},-1);
    for (auto p:active_center) cv::circle(view,pixel(p),3,{0,220,255},-1);
    cv::rectangle(view,pixel({.09,.09}),pixel({-.09,-.09}),{255,170,80},2);
    cv::arrowedLine(view,pixel({0.,0.}),pixel({.18,0.}),{255,170,80},2);
    cv::putText(view,"left2 lidar: "+state_,{15,26},cv::FONT_HERSHEY_SIMPLEX,.58,{255,255,255},1);
    std::ostringstream text;text<<std::fixed<<std::setprecision(3)<<"e="<<trace_.error<<"  v="<<command_.linear.x<<"  w="<<command_.angular.z;
    cv::putText(view,text.str(),{15,52},cv::FONT_HERSHEY_SIMPLEX,.55,{255,255,255},1);
    cv::putText(view,"gray: scan   green/blue: walls   yellow: offset",{15,78},cv::FONT_HERSHEY_SIMPLEX,.43,{190,230,230},1);
    sensor_msgs::msg::Image image;image.header.frame_id="base_footprint";image.header.stamp=stamp;
    image.width=view.cols;image.height=view.rows;image.encoding="bgr8";image.step=view.cols*3;
    image.data.assign(view.datastart,view.dataend);image_pub_->publish(image);
    if (debug_window_) {cv::imshow("left2 lidar - line tracking",view);cv::waitKey(1);}
  }
  tf2_ros::Buffer buffer_;tf2_ros::TransformListener listener_;
  left2_lidar::PID pid_;left2_lidar::Trace trace_;std::vector<Point> points_;
  left2_lidar::LineGeometry geometry_;left2_lidar::Trace right_trace_;
  bool dual_wall_geometry_=false;int geometry_source_=0;
  bool geometric_control_=false,was_geometric_=false;
  double tangent_window_,heading_gain_,curvature_filter_,curvature_limit_,filtered_curvature_=0.;
  bool enabled_,debug_window_,waiting_start_=false,have_odom_=false,have_wall_=false,finished_=false,exit_enabled_=false;
  bool center_corridor_=false,width_observed_=false,have_pair_=false;
  double speed_,offset_,horizon_,step_,max_left_,turn_speed_,front_turn_,emergency_,timeout_,right_limit_;
  double front_=-1,left_=-1,right_=-1,last_angular_=0.,distance_=0.,last_x_=0.,last_y_=0.;
  double exit_left_=-1.,exit_right_=-1.;
  double yaw_gain_,angular_acceleration_,measured_angular_=0.,measured_linear_=0.,last_yaw_=0.;
  double corridor_width_,width_min_,width_max_,width_filter_,width_rate_,center_horizon_,wall_left_=-1.,wall_right_=-1.;
  double exit_clearance_,exit_confirm_,exit_min_travel_,exit_since_=-1.;
  double exit_open_confirm_,exit_clear_distance_,exit_stop_since_=-1.,exit_progress_=0.;
  double exit_x_=0.,exit_y_=0.,exit_heading_=0.;
  ExitPhase exit_phase_=ExitPhase::FOLLOW;
  int64_t last_scan_ns_=-1;
  int lost_=0,loss_frames_;
  std::string state_="WAIT_SCAN",logged_state_;
  Steady::time_point last_scan_=Steady::now()-10s;
  Steady::time_point last_odom_=Steady::now()-10s;
  geometry_msgs::msg::Twist command_;nav_msgs::msg::Path path_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_,old_status_pub_,result_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<Scan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_,old_enable_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<Left2Lidar>());
  rclcpp::shutdown();return 0;
}
