/**
 * \todo
 */

#include <iostream>
#include <iomanip>
#include <memory>

#include <traxxs/path/path.hpp>

template< class T >
using sptr = std::shared_ptr<T>;

using namespace traxxs;

int main(void) {
  
  std::cout << std::setprecision(3) << std::fixed << std::showpos ;
  
  path::CartesianPathWaypoint pt_start, pt_wpt, pt_end;
  
  sptr< path::CartesianSegment< path::LinearSegment, path::SmoothStep7 > > seg_start, seg_end; 
  sptr< path::CartesianSegment< path::CircularBlend, path::SmoothStep7 > > seg_blend;
  
  pt_start.x.p  << 0, 0, 0;
  pt_wpt.x.p    << 1, 0, 0;
  pt_end.x.p    << 1, 1, 0;
  
  pt_start.x.q  = Eigen::Quaterniond( 1, 0, 0, 0 ); // w, x, y , z
  pt_wpt.x.q    = Eigen::Quaterniond( 0, 1, 0, 0 ); // w, x, y , z
  pt_end.x.q    = Eigen::Quaterniond( 0, 0, 1, 0 ); // w, x, y , z
  
  path::PathBounds4d path_bounds;
  path_bounds.dx << 1.0, 1.0, 1.0, 0.1;
  path_bounds.ddx = 10.0 * path_bounds.dx;
  path_bounds.j = 10.0 * path_bounds.ddx;
  
  using JoiningSegment_t  = path::CartesianSegment< path::LinearSegment, path::SmoothStep7 >;
  using BlendSegment_t    = path::CartesianSegment< path::CircularBlend, path::SmoothStep7 >;
  std::vector< path::CartesianPathWaypoint > waypoints = { pt_start, pt_wpt, pt_end};
  std::vector< sptr< path::PathSegment > > segments = path::blendedSegmentsFromWaypoints< path::CartesianPathWaypoint, JoiningSegment_t, BlendSegment_t, double>( 
   path_bounds, waypoints, 0.1 );
  path::Path path( segments );
  
  path.init();
  
  arc::ArcConditions tmp;
  for ( auto& seg : segments ) {
    tmp.s = 0.0;
    std::cout << "from:\t"  << path::Pose( seg->getPosition(tmp) ).p.transpose() << "\n\t" 
                              << path::Pose( seg->getPosition(tmp) ).q.coeffs().transpose() <<  "\n" ;
    tmp.s = seg->getLength();
    std::cout << "to:\t"    << path::Pose( seg->getPosition(tmp) ).p.transpose() << "\n\t" 
                              << path::Pose( seg->getPosition(tmp) ).q.coeffs().transpose() <<  "\n" ;
  }
  
}
