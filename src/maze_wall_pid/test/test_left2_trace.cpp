#include <gtest/gtest.h>
#include "maze_wall_pid/left2_trace.hpp"
using namespace left2_lidar;

TEST(Left2Trace, ContinuousWallProducesHalfRoadWidthOffset) {
  std::vector<Point> cloud;
  for (double x=-.05;x<1.;x+=.005) {cloud.push_back({x,.3});cloud.push_back({x,-.3});}
  auto trace=trace_edge(cloud,.3);
  ASSERT_TRUE(trace.found);EXPECT_GT(trace.edge.size(),30u);
  EXPECT_NEAR(trace.error,0.,1e-9);
  for (auto p:trace.center) EXPECT_NEAR(p.y,0.,1e-9);
}
TEST(Left2Trace, GapDoesNotJumpOntoAnotherWall) {
  std::vector<Point> cloud;
  for (double x=-.05;x<.25;x+=.005) cloud.push_back({x,.3});
  for (double x=.5;x<.85;x+=.005) cloud.push_back({x,.3});
  auto trace=trace_edge(cloud,.3);
  ASSERT_TRUE(trace.found);EXPECT_LT(trace.edge.back().x,.3);
  for (auto &p:cloud) p.x+=.4;
  EXPECT_FALSE(trace_edge(cloud,.3).found);
}
TEST(Left2Trace, FrontWallIsNotALeftTrack) {
  std::vector<Point> cloud;
  for (double y=.07;y<.65;y+=.005) cloud.push_back({.3,y});
  EXPECT_FALSE(trace_edge(cloud,.3).found);
}
TEST(Left2Trace, NearPointsHaveMoreWeightThanFarPoints) {
  std::vector<Point> near(60,{0.,.3}),far=near;
  near[3].y+=.1;far[55].y+=.1;
  EXPECT_GT(error_calculater(near,.3),error_calculater(far,.3));
}
TEST(Left2Trace, RearLeftEntranceEdgeCanBeTrackedAroundCorner) {
  std::vector<Point> cloud;
  for(double y=.30;y<=.975;y+=.005) cloud.push_back({-.10,y});
  for(double x=-.10;x<.5;x+=.005) cloud.push_back({x,.975});
  auto trace=trace_edge(cloud,.3);
  ASSERT_TRUE(trace.found);
  EXPECT_NEAR(trace.center.front().x,.20,.025);
  EXPECT_GT(trace.error,.15);
}
TEST(Left2Trace, PerpendicularFrontSegmentOffsetsIntoTheCorridor) {
  std::vector<Point> cloud;
  for(double x=-.05;x<=.5;x+=.005) cloud.push_back({x,.30});
  for(double y=.30;y>-.4;y-=.005) cloud.push_back({.5,y});
  auto trace=trace_edge(cloud,.3,1.1);
  ASSERT_TRUE(trace.found);
  EXPECT_LT(trace.edge.back().y,0.);
  EXPECT_NEAR(trace.center.back().x,.20,.025);
  EXPECT_LT(trace.center.back().y,0.);
}
TEST(Left2Trace, SteeringSignAndErrorConverge) {
  PID pid;double y=-.07,yaw=.10;
  for(int k=0;k<400;++k) {
    std::vector<Point> cloud;
    for(double x=-.05;x<.85;x+=.005)
      cloud.push_back({x,(.3-y)/std::cos(yaw)-x*std::tan(yaw)});
    auto trace=trace_edge(cloud,.3);ASSERT_TRUE(trace.found);
    double w=pid.update(trace.error,.05);
    y+=.2*std::sin(yaw)*.05;yaw+=w*.05;
  }
  EXPECT_LT(std::abs(y),.012);EXPECT_LT(std::abs(yaw),.025);
  EXPECT_GT(error_calculater({{0.,.4},{.1,.4}},.3),0.);
  EXPECT_LT(error_calculater({{0.,.2},{.1,.2}},.3),0.);
}
TEST(Left2Trace, PIDIntegralIsBoundedAndReset) {
  PID pid;pid.i=1.;
  for(int k=0;k<200;++k) EXPECT_LE(std::abs(pid.update(1.,.1)),pid.limit);
  EXPECT_LE(pid.integral,pid.integral_limit);
  pid.reset();EXPECT_EQ(pid.integral,0.);EXPECT_EQ(pid.update(0.,.1),0.);
}

TEST(Left2Geometry, StraightLineSeparatesLateralAndHeadingError) {
  std::vector<Point> points;
  const double heading=.20,offset=.12;
  for(double s=-.20;s<.85;s+=.015)
    points.push_back({s*std::cos(heading)-offset*std::sin(heading),
                      s*std::sin(heading)+offset*std::cos(heading)});
  const auto g=line_geometry(points);ASSERT_TRUE(g.valid);
  EXPECT_NEAR(g.lateral,offset,1e-5);EXPECT_NEAR(g.heading,heading,1e-5);
  EXPECT_NEAR(g.curvature,0.,1e-6);
}

TEST(Left2Geometry, ConstantCurvatureNeedsSteeringEvenWithZeroTrackingError) {
  for(double sign:{-1.,1.}) {
    std::vector<Point> points;const double radius=1.5;
    for(double s=-.15;s<.8;s+=.015)
      points.push_back({radius*std::sin(s/radius),sign*radius*(1.-std::cos(s/radius))});
    const auto g=line_geometry(points);ASSERT_TRUE(g.valid);
    EXPECT_NEAR(g.lateral,0.,.01);EXPECT_NEAR(g.heading,0.,.03);
    EXPECT_NEAR(g.curvature,sign/radius,.06);
  }
}

TEST(Left2Geometry, VeryShortFragmentsCannotInventAWholeTurn) {
  std::vector<Point> points;
  for(int i=0;i<10;++i) points.push_back({i*.01,.01*i*i});
  EXPECT_FALSE(line_geometry(points).valid);
}

TEST(Left2Geometry, WallFitRejectsFrontWallAndEstimatesPerpendicularDistance) {
  std::vector<Point> cloud;
  for(double x=-.3;x<=.3;x+=.01) {
    cloud.push_back({x,.85+.20*x});cloud.push_back({x,-.85+.20*x});
  }
  const auto left=fit_side_wall(cloud,1),right=fit_side_wall(cloud,-1);
  ASSERT_TRUE(left.valid);ASSERT_TRUE(right.valid);
  EXPECT_NEAR(left.distance+right.distance,1.7/std::sqrt(1.04),.001);
  cloud.clear();for(double y=.2;y<1.5;y+=.01) cloud.push_back({.25,y});
  EXPECT_FALSE(fit_side_wall(cloud,1).valid);
}

TEST(Left2Geometry, BroadTangentFitSuppressesNoiseAmplifiedByLargeOffset) {
  std::vector<Point> cloud;
  for(double x=-.15;x<1.3;x+=.005) cloud.push_back({x,.85+.004*std::sin(100.*x)});
  const auto trace=trace_edge(cloud,.85,1.5,.015,1.6,.85,.18);
  ASSERT_TRUE(trace.found);
  const auto geometry=line_geometry(trace.center);ASSERT_TRUE(geometry.valid);
  EXPECT_LT(std::abs(geometry.lateral),.02);EXPECT_LT(std::abs(geometry.heading),.06);
  EXPECT_LT(std::abs(geometry.curvature),.18);
}

TEST(Left2Geometry, OffsetCurvedWallsRetainsTurnDirection) {
  for(double sign:{-1.,1.}) {
    std::vector<Point> cloud;
    const double radius=1.5-sign*.85;
    for(double angle=-.2;angle<1.1;angle+=.004)
      cloud.push_back({radius*std::sin(angle),sign*(1.5-radius*std::cos(angle))});
    const auto trace=trace_edge(cloud,.85,1.5,.015,1.6,.85,.18);
    ASSERT_TRUE(trace.found);
    const auto geometry=line_geometry(trace.center);ASSERT_TRUE(geometry.valid);
    EXPECT_NEAR(geometry.curvature,sign/1.5,.18);
    EXPECT_LT(std::abs(geometry.lateral),.04);
    const auto wall=wall_geometry(trace.edge,.85);ASSERT_TRUE(wall.valid);
    EXPECT_NEAR(wall.curvature,sign/1.5,.18);
    EXPECT_LT(std::abs(wall.lateral),.04);
  }
}

TEST(Left2Corridor, RightWallCarriesCenterlineAcrossShortLeftOcclusion) {
  Trace left,right;
  for(double x=-.1;x<.06;x+=.015) left.edge.push_back({x,.85});
  for(double x=-.1;x<.9;x+=.015) right.edge.push_back({x,.85});
  const auto result=corridor_geometry(left,right,.85);
  ASSERT_TRUE(result.geometry.valid);EXPECT_EQ(result.source,2);
  EXPECT_NEAR(result.geometry.lateral,0.,1e-6);EXPECT_NEAR(result.geometry.heading,0.,1e-6);
}

TEST(Left2Corridor, MirroringKeepsSteeringSignForOffCenterRobot) {
  Trace left,right;
  for(double x=-.1;x<.9;x+=.015) {
    left.edge.push_back({x,1.05});right.edge.push_back({x,.65});
  }
  const auto result=corridor_geometry(left,right,.85);
  ASSERT_TRUE(result.geometry.valid);EXPECT_EQ(result.source,3);
  EXPECT_NEAR(result.geometry.lateral,.20,1e-5);
}

TEST(Left2Corridor, RejectShortConflictingTangentRatherThanPerfectFitResidual) {
  Trace left,right;
  for(double s=0.;s<.25;s+=.015)
    left.edge.push_back({s*std::cos(.7),.85+s*std::sin(.7)});
  for(double x=-.1;x<.9;x+=.015) right.edge.push_back({x,.85});
  const auto result=corridor_geometry(left,right,.85);
  ASSERT_TRUE(result.geometry.valid);EXPECT_EQ(result.source,2);
  EXPECT_NEAR(result.geometry.heading,0.,1e-6);
}

TEST(Left2Corridor, MissingBothWallsCannotInventATrack) {
  Trace empty;
  EXPECT_FALSE(corridor_geometry(empty,empty,.85).geometry.valid);
}
