#pragma once
// Adapted from Zythyra/ifly2_2026 left2.cpp, blob
// 72924b249b756bd87030678057eca3e62a120ec0: trace_edge,
// error_calculater, and the bounded PID. Coordinates here are metres,
// x forward / y left; no camera calibration or perspective compensation.
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace left2_lidar {
struct Point { double x, y; };
struct Trace {
  std::vector<Point> edge, center;
  double error = 0.;
  bool found = false;
};

struct CurvePolynomial {
  std::array<Point,3> c{};
  bool valid=false;
};

// Least-squares x(s), y(s), with s measured along the observed curve. Unlike
// y(x), this remains well-defined through a bend and near vertical tangents.
inline CurvePolynomial fit_curve(const std::vector<Point>& points,
                                const std::vector<double>& arc,size_t low,size_t high,double origin) {
  CurvePolynomial result;
  if (high<=low+3 || arc[high]-arc[low]<.08) return result;
  double a[3][5]{};
  for (size_t i=low;i<=high;++i) {
    const double s=arc[i]-origin,b[3]={1.,s,s*s};
    for(int row=0;row<3;++row) {
      for(int col=0;col<3;++col) a[row][col]+=b[row]*b[col];
      a[row][3]+=b[row]*points[i].x;a[row][4]+=b[row]*points[i].y;
    }
  }
  for(int k=0;k<3;++k) {
    int pivot=k;
    for(int i=k+1;i<3;++i) if(std::abs(a[i][k])>std::abs(a[pivot][k])) pivot=i;
    if(std::abs(a[pivot][k])<1e-10) return result;
    if(pivot!=k) for(int j=k;j<5;++j) std::swap(a[pivot][j],a[k][j]);
    const double d=a[k][k];for(int j=k;j<5;++j) a[k][j]/=d;
    for(int i=0;i<3;++i) if(i!=k) {
      const double v=a[i][k];for(int j=k;j<5;++j) a[i][j]-=v*a[k][j];
    }
  }
  for(int k=0;k<3;++k) result.c[k]={a[k][3],a[k][4]};
  result.valid=true;return result;
}

inline std::vector<double> curve_arc(const std::vector<Point>& points) {
  std::vector<double> arc(points.size(),0.);
  for(size_t i=1;i<points.size();++i) arc[i]=arc[i-1]+std::hypot(points[i].x-points[i-1].x,points[i].y-points[i-1].y);
  return arc;
}

struct LineGeometry {
  double lateral=0.,heading=0.,curvature=0.,rms=0.;
  bool valid=false;
};

inline LineGeometry line_geometry(const std::vector<Point>& points) {
  LineGeometry result;
  if(points.size()<7) return result;
  const auto arc=curve_arc(points);
  const auto poly=fit_curve(points,arc,0,points.size()-1,0.);
  if(!poly.valid || arc.back()<.20) return result;
  double nearest=0.,best=1e9;
  // Also allow a short extrapolation back to the axle when the wall is
  // partly occluded. Never infer a long unseen turn from a short fragment.
  for(double s=-.15;s<=arc.back();s+=.01) {
    const double x=poly.c[0].x+s*poly.c[1].x+s*s*poly.c[2].x;
    const double y=poly.c[0].y+s*poly.c[1].y+s*s*poly.c[2].y;
    if(x*x+y*y<best) {best=x*x+y*y;nearest=s;}
  }
  const auto p=Point{poly.c[0].x+nearest*poly.c[1].x+nearest*nearest*poly.c[2].x,
                     poly.c[0].y+nearest*poly.c[1].y+nearest*nearest*poly.c[2].y};
  const auto d=Point{poly.c[1].x+2.*nearest*poly.c[2].x,poly.c[1].y+2.*nearest*poly.c[2].y};
  const double norm=std::hypot(d.x,d.y);
  if(norm<.35 || norm>1.7) return result;
  result.heading=std::atan2(d.y,d.x);
  result.lateral=(-d.y*p.x+d.x*p.y)/norm;
  result.curvature=2.*(d.x*poly.c[2].y-d.y*poly.c[2].x)/(norm*norm*norm);
  double error=0.;
  for(size_t i=0;i<points.size();++i) {
    const double s=arc[i];
    error+=std::pow(points[i].x-poly.c[0].x-s*poly.c[1].x-s*s*poly.c[2].x,2)+
      std::pow(points[i].y-poly.c[0].y-s*poly.c[1].y-s*s*poly.c[2].y,2);
  }
  result.rms=std::sqrt(error/points.size());
  result.valid=std::isfinite(result.curvature) && std::abs(result.heading)<1.3 &&
    std::abs(result.curvature)<3. && result.rms<.06;
  return result;
}

inline LineGeometry wall_geometry(const std::vector<Point>& points,double offset) {
  LineGeometry result;
  if(points.size()<7) return result;
  const auto arc=curve_arc(points);
  size_t high=points.size()-1;
  while(high>0 && arc[high]>.80) --high;
  const auto poly=fit_curve(points,arc,0,high,0.);
  if(!poly.valid || arc[high]<.20) return result;
  double best=1e9;
  for(double s=-.10;s<=arc[high];s+=.01) {
    const Point p{poly.c[0].x+s*poly.c[1].x+s*s*poly.c[2].x,
                   poly.c[0].y+s*poly.c[1].y+s*s*poly.c[2].y};
    const Point d{poly.c[1].x+2.*s*poly.c[2].x,poly.c[1].y+2.*s*poly.c[2].y};
    const double norm=std::hypot(d.x,d.y);
    if(norm<.35 || norm>1.7) continue;
    const Point center{p.x+offset*d.y/norm,p.y-offset*d.x/norm};
    const double distance=center.x*center.x+center.y*center.y;
    if(distance>=best) continue;
    const double wall_curvature=2.*(d.x*poly.c[2].y-d.y*poly.c[2].x)/(norm*norm*norm);
    const double factor=1.+offset*wall_curvature;
    // An offset beyond a concave wall's local radius folds back on itself.
    if(factor<.25) continue;
    best=distance;result.heading=std::atan2(d.y,d.x);
    result.lateral=(-d.y*center.x+d.x*center.y)/norm;
    result.curvature=wall_curvature/factor;
    result.valid=std::abs(result.heading)<1.3 && std::abs(result.curvature)<3.;
  }
  double error=0.;
  for(size_t i=0;i<=high;++i) {
    const double s=arc[i];
    error+=std::pow(points[i].x-poly.c[0].x-s*poly.c[1].x-s*s*poly.c[2].x,2)+
      std::pow(points[i].y-poly.c[0].y-s*poly.c[1].y-s*s*poly.c[2].y,2);
  }
  result.rms=std::sqrt(error/(high+1));
  result.valid=result.valid && result.rms<.045;return result;
}

struct CorridorGeometry {
  LineGeometry geometry;
  int source=0;  // 1: left, 2: right, 3: both. Right input is mirrored.
};

inline CorridorGeometry corridor_geometry(const Trace& left,const Trace& mirrored_right,double offset) {
  const auto l=wall_geometry(left.edge,offset);
  auto r=wall_geometry(mirrored_right.edge,offset);
  r.lateral=-r.lateral;r.heading=-r.heading;r.curvature=-r.curvature;
  if(!l.valid) return {r,r.valid?2:0};
  if(!r.valid) return {l,1};
  const auto quality=[](const Trace& trace,const LineGeometry& g) {
    const auto arc=curve_arc(trace.edge);
    const double length=std::min(.8,arc.back());
    return length*length/(.01+g.rms);
  };
  const double ql=quality(left,l),qr=quality(mirrored_right,r);
  // A tiny visible inner face can fit almost perfectly but its tangent is
  // unreliable. Prefer the longer observation when the estimates disagree.
  if(std::abs(l.heading-r.heading)>.35 || std::abs(l.lateral-r.lateral)>.25)
    return ql>=qr?CorridorGeometry{l,1}:CorridorGeometry{r,2};
  const double w=ql/(ql+qr);
  LineGeometry g;
  g.valid=true;g.lateral=w*l.lateral+(1.-w)*r.lateral;
  g.heading=w*l.heading+(1.-w)*r.heading;
  g.curvature=w*l.curvature+(1.-w)*r.curvature;
  g.rms=w*l.rms+(1.-w)*r.rms;
  return {g,3};
}

struct WallFit {
  double distance=0.,angle=0.,rms=0.;
  bool valid=false;
};

inline WallFit fit_side_wall(const std::vector<Point>& cloud,int side) {
  std::vector<Point> points;
  for (auto p:cloud)
    if (std::abs(p.x)<=.30 && side*p.y>.15 && side*p.y<2.5) points.push_back(p);
  const auto fit=[](const std::vector<Point>& values) {
    WallFit result;
    if (values.size()<8) return result;
    double x=0.,y=0.,xx=0.,xy=0.,yy=0.,lo=1e9,hi=-1e9;
    for (auto p:values) {x+=p.x;y+=p.y;lo=std::min(lo,p.x);hi=std::max(hi,p.x);}
    x/=values.size();y/=values.size();
    if (hi-lo<.20) return result;
    for (auto p:values) {xx+=(p.x-x)*(p.x-x);xy+=(p.x-x)*(p.y-y);yy+=(p.y-y)*(p.y-y);}
    result.angle=.5*std::atan2(2.*xy,xx-yy);
    if (std::abs(result.angle)>.80) return result;  // Reject a front wall/post.
    const double nx=-std::sin(result.angle),ny=std::cos(result.angle);
    result.distance=x*nx+y*ny;
    double error=0.;
    for (auto p:values) error+=std::pow(p.x*nx+p.y*ny-result.distance,2);
    result.rms=std::sqrt(error/values.size());result.valid=true;return result;
  };
  auto initial=fit(points);
  if (!initial.valid) return initial;
  std::vector<double> residual;
  for (auto p:points) residual.push_back(std::abs(-p.x*std::sin(initial.angle)+p.y*std::cos(initial.angle)-initial.distance));
  auto sorted=residual;std::sort(sorted.begin(),sorted.end());
  const double cutoff=std::clamp(3.*sorted[sorted.size()/2],.03,.09);
  std::vector<Point> inliers;
  for (size_t i=0;i<points.size();++i) if (residual[i]<=cutoff) inliers.push_back(points[i]);
  auto result=fit(inliers);
  result.valid=result.valid && side*result.distance>.15 && result.rms<=.05;
  result.distance=std::abs(result.distance);return result;
}

inline std::vector<Point> sample_center(const std::vector<Point>& points,double step,double horizon) {
  if (points.empty()) return {};
  std::vector<Point> sampled{points.front()};double total=0.,next=step;
  for (size_t i=1;i<points.size();++i) {
    const auto a=points[i-1],b=points[i];const double length=std::hypot(b.x-a.x,b.y-a.y);
    if (length<1e-8) continue;
    while (next<=total+length && next<=horizon) {
      const double t=(next-total)/length;
      sampled.push_back({a.x+t*(b.x-a.x),a.y+t*(b.y-a.y)});next+=step;
    }
    total+=length;if (total>=horizon) break;
  }
  return sampled;
}

inline double error_calculater(const std::vector<Point>& points, double offset) {
  if (points.empty()) return 0.;
  double sum = 0.;
  for (size_t i = 0; i < points.size(); ++i) {
    const double index = static_cast<double>(i);
    const double weight = i <= 30 ? 1. - index/100. :
      .7 * std::exp(-.064*(index-30.));
    // Original image x goes right; lidar y goes left, hence the sign change.
    sum += (points[i].y-offset)*weight;
  }
  return sum/static_cast<double>(points.size());
}

inline Trace trace_edge(const std::vector<Point>& all, double offset,
                        double horizon=.85, double step=.015, double max_left=.65,
                        double center_horizon=0.,double tangent_window=0.) {
  // Radar samples replace the ordered boundary pixels of trace_edge. On a
  // left boundary, decreasing ray angle walks from the nearby edge toward
  // the forward edge, including perpendicular walls at a corner.
  std::vector<Point> points=all;
  const auto angle=[](Point p) {
    double a=std::atan2(p.y,p.x);return a<0 ? a+2.*M_PI : a;
  };
  std::sort(points.begin(),points.end(),[&](Point a,Point b){return angle(a)<angle(b);});
  std::vector<size_t> seeds;
  for (size_t i=0;i<points.size();++i) {
    const auto p=points[i];
    if (p.x>=-.20 && p.x<.15 && p.y>.06 && p.y<max_left) seeds.push_back(i);
  }
  std::sort(seeds.begin(),seeds.end(),[&](size_t a,size_t b) {
    return std::hypot(points[a].x,points[a].y)<std::hypot(points[b].x,points[b].y);
  });
  Trace result;
  for (size_t seed:seeds) {
    std::vector<Point> chain{points[seed]};double length=0.;
    for (size_t n=1;n<points.size();++n) {
      const auto p=points[(seed+points.size()-n)%points.size()];
      const double gap=std::hypot(p.x-chain.back().x,p.y-chain.back().y);
      // A physical gap terminates the edge; do not reconnect a different wall.
      if (gap>.10 || p.x<-.25 || std::hypot(p.x,p.y)>1.5) break;
      length+=gap;chain.push_back(p);
      if (length>=horizon) break;
    }
    if (chain.size()<7 || length<.12) continue;
    // Smooth range noise, then use equal-distance samples so nearby rays do
    // not get extra weight merely because angular sampling is denser there.
    std::vector<Point> smooth;
    for (size_t i=0;i<chain.size();++i) {
      Point p{0.,0.};size_t count=0;
      const size_t low=i>2?i-2:0,high=std::min(chain.size()-1,i+2);
      for(size_t k=low;k<=high;++k) {p.x+=chain[k].x;p.y+=chain[k].y;++count;}
      smooth.push_back({p.x/count,p.y/count});
    }
    result.edge.push_back(smooth.front());
    double carry=0.;
    for(size_t i=1;i<smooth.size();++i) {
      Point a=smooth[i-1],b=smooth[i];double segment=std::hypot(b.x-a.x,b.y-a.y);
      if(segment<1e-8) continue;
      double along=step-carry;
      while(along<=segment) {
        const double t=along/segment;
        result.edge.push_back({a.x+t*(b.x-a.x),a.y+t*(b.y-a.y)});
        along+=step;
      }
      carry=segment-(along-step);
    }
    if(result.edge.size()<7) {result.edge.clear();continue;}
    const auto edge_arc=curve_arc(result.edge);
    for(size_t i=0;i<result.edge.size();++i) {
      const auto a=result.edge[i>2?i-2:0];
      const auto b=result.edge[std::min(result.edge.size()-1,i+2)];
      double dx=b.x-a.x,dy=b.y-a.y,norm=std::hypot(dx,dy);
      Point origin=result.edge[i];
      if(tangent_window>0.) {
        const size_t span=std::max(size_t(3),static_cast<size_t>(std::ceil(tangent_window/step)));
        const auto poly=fit_curve(result.edge,edge_arc,i>span?i-span:0,
                                  std::min(result.edge.size()-1,i+span),edge_arc[i]);
        if(poly.valid) {origin=poly.c[0];dx=poly.c[1].x;dy=poly.c[1].y;norm=std::hypot(dx,dy);}
      }
      if(norm<1e-6) {dx=1.;dy=0.;norm=1.;}
      // Right normal of the traced edge: works on straight and corner walls.
      result.center.push_back({origin.x+offset*dy/norm,origin.y-offset*dx/norm});
    }
    result.found=true;
    // A large normal offset stretches inner-wall arcs and compresses outer
    // arcs. Keep PID samples equally spaced on the actual target curve.
    if (center_horizon>0.) result.center=sample_center(result.center,step,center_horizon);
    if (result.center.size()<7) {result=Trace{};continue;}
    result.error=error_calculater(result.center,0.);
    break;
  }
  return result;
}

class PID {
public:
  double p=4., i=0., d=12., limit=1.1, integral_limit=.15;
  double integral=0., previous=0., p_term=0., i_term=0., d_term=0.;
  bool initialized=false;
  void reset() { integral=previous=p_term=i_term=d_term=0.; initialized=false; }
  double update(double error, double dt) {
    integral=std::clamp(integral+error*dt,-integral_limit,integral_limit);
    // left2 uses a bounded frame-to-frame difference, not a second heading PID.
    const double diff=initialized ? std::clamp(error-previous,-.10,.10) : 0.;
    p_term=p*error; i_term=i*integral; d_term=d*diff;
    previous=error; initialized=true;
    return std::clamp(p_term+i_term+d_term,-limit,limit);
  }
};
}  // namespace left2_lidar
