/**
 * \todo
 */

#include <iostream>
#include <iomanip>
#include <memory>

#include <traxxs/trajectory/trajectory.hpp>
#include "arctrajgensoftmotion.hpp"
#include "samples_helpers.hpp"

template< class T >
using sptr = std::shared_ptr<T>;

using namespace traxxs;

int main(void) {
  
//   std::cout << std::setprecision(3) << std::fixed << std::showpos ;
  
  path::CartesianPathWaypoint pt_start, pt_wpt, pt_end;
  
  pt_start.x.p  << 0, 0, 0;
  pt_wpt.x.p    << 1, 0, 0;
  pt_end.x.p    << 1, 1, 0;
  
  pt_start.x.q  = Eigen::Quaterniond( 1, 0, 0, 0 ); // w, x, y , z
  pt_wpt.x.q    = Eigen::Quaterniond( 0, 1, 0, 0 ); // w, x, y , z
  pt_end.x.q    = Eigen::Quaterniond( 0, 0, 1, 0 ); // w, x, y , z
  
  pt_start.pathConditionsPosition.dx << 0, 0, 0;
  pt_start.pathConditionsOrientation.dx << 0;
  
//   // force arriving at the waypoint with null velocity to ensure acceleration continuity
//   pt_wpt.pathConditionsPosition.dx << 0, 0, 0;
//   pt_wpt.pathConditionsOrientation.dx << 0;
  
  pt_end.pathConditionsPosition.dx << 0, 0, 0;
  pt_end.pathConditionsOrientation.dx << 0;
  
  path::PathBounds4d path_bounds;
  path_bounds.dx << 1.0, 1.0, 1.0, 10.0;
  path_bounds.ddx = 10.0 * path_bounds.dx;
  path_bounds.j = 10.0 * path_bounds.ddx;
  
  using JoiningSegment_t  = path::CartesianSegment< path::LinearSegment, path::SmoothStep7 >;
  using BlendSegment_t    = path::CartesianSegment< path::CircularBlend, path::SmoothStep7 >;

  std::vector< path::CartesianPathWaypoint > waypoints = { pt_start, pt_wpt, pt_end};
  auto segments = path::blendedSegmentsFromWaypoints< path::CartesianPathWaypoint, JoiningSegment_t, BlendSegment_t, double>( 
   path_bounds, waypoints, 0.1 ); // std::vector< sptr< path::PathSegment > >
  
  trajectory::Trajectory trajectory;
  if ( !trajectory.set< ArcTrajGenSoftMotion >( segments ) )
    return 1;
  
  int seg_idx;
  bool is_beyond;
  trajectory::TrajectoryState state;
  arc::ArcConditions conds;
  std::shared_ptr< path::PathSegment > seg;
  
  StopWatch stopwatch;
  MinMaxAvg<double> minMaxAvg;
  for ( double t = 0; t < 1000.0; t+=0.0001 ) {
    stopwatch.start();
    if ( !trajectory.getArcConditions( t, conds, seg, &seg_idx ) )
      break;
    trajectory.getState( t, state, nullptr, &is_beyond );
    stopwatch.stop();
    minMaxAvg.process( stopwatch.get_ms() );
    if ( is_beyond )
      break;
    std::cout << t << ";" << seg_idx
      << ";" << conds.s << ";" << conds.ds << ";" << conds.dds  << ";" << conds.j 
      << ";" << toCSV( state.x )
      << ";" << toCSV( state.pathConditions.dx ) << ";" << toCSV( state.pathConditions.ddx ) << ";" << toCSV( state.pathConditions.j ) << std::endl;
  }
  std::cerr << "Avg: " << minMaxAvg.getAverage() << "ms, min: " << minMaxAvg.getMin() << "ms, max: " << minMaxAvg.getMax() << "ms\n";
  
  
  
}
