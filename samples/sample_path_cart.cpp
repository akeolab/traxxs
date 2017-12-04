/**
 * \todo
 */

#include <iostream>
#include <iomanip>
#include <memory>

#include <traxxs/path/path.hpp>

template< class T >
using sptr = std::shared_ptr<T>;

int main(void) {
  
  std::cout << std::setprecision(3) << std::fixed << std::showpos ;
  
  CartesianPathConditions pt_start, pt_wpt, pt_end;
  
  sptr< CartesianSegment< LinearSegment, SmoothStep7 > > seg_start, seg_end; 
  sptr< CartesianSegment< CircularBlend, SmoothStep7 > > seg_blend;
  
  pt_start.position.p  << 0, 0, 0;
  pt_wpt.position.p    << 1, 0, 0;
  pt_end.position.p    << 1, 1, 0;
  
  pt_start.position.q  = Eigen::Quaterniond( 1, 0, 0, 0 ); // w, x, y , z
  pt_wpt.position.q    = Eigen::Quaterniond( 0, 1, 0, 0 ); // w, x, y , z
  pt_end.position.q    = Eigen::Quaterniond( 0, 0, 1, 0 ); // w, x, y , z
  
  PathBounds4d path_bounds;
  path_bounds.dx << 1.0, 1.0, 1.0, 0.1;
  path_bounds.ddx = 10.0 * path_bounds.dx;
  path_bounds.j = 10.0 * path_bounds.ddx;
  
  std::vector< CartesianPathConditions > waypoints = { pt_start, pt_wpt, pt_end};
  std::vector< sptr< PathSegment > > segments = blendedSegmentsFromCartesianWaypoints< LinearSegment, CircularBlend, SmoothStep7, double>( 
   path_bounds, waypoints, 0.1 );
  Path path( segments );
  
  path.init();
  
  ArcConditions tmp;
  for ( auto& seg : segments ) {
    tmp.s = 0.0;
    std::cout << "from:\t"  << Pose( seg->getPosition(tmp) ).p.transpose() << "\n\t" 
                              << Pose( seg->getPosition(tmp) ).q.coeffs().transpose() <<  "\n" ;
    tmp.s = seg->getLength();
    std::cout << "to:\t"    << Pose( seg->getPosition(tmp) ).p.transpose() << "\n\t" 
                              << Pose( seg->getPosition(tmp) ).q.coeffs().transpose() <<  "\n" ;
  }
  
}
