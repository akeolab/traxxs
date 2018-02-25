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
  
  path::PathWaypoint pt_start(2), pt_wpt(2), pt_end(2);
  sptr< path::PathSegment > seg_start, seg_blend, seg_end;
  
  pt_start.x  << 0, 0 ;
  pt_wpt.x    << 1, 0 ;
  pt_end.x    << 1, 1 ;
  
  pt_start.pathConditions.dx << 0, 0;
  pt_end.pathConditions.dx   << 0, 0;
  
//   // use this to arrive on the blend with null velocity
//   pt_end.pathConditions.dx   << 0, 0;
  
  path::PathBounds path_bounds(2);
  path_bounds.dx << 1.0, 1.0;
  path_bounds.ddx = 10.0 * path_bounds.dx;
  path_bounds.j = 10.0 * path_bounds.ddx;
  
  using JoiningSegment_t  = path::LinearSegment;
  using BlendSegment_t    = path::CircularBlend;

  std::vector< path::PathWaypoint > waypoints = { pt_start, pt_wpt, pt_end};
  auto segments = path::blendedSegmentsFromWaypoints< path::PathWaypoint, JoiningSegment_t, BlendSegment_t, double>( 
   path_bounds, waypoints, 0.1 ); // std::vector< sptr< path::PathSegment > >
  
  for( auto& seg : segments )
    seg->init();

  trajectory::Trajectory trajectory;  
  trajectory.set< ArcTrajGenSoftMotion >( segments );
  
  int seg_idx;
  trajectory::TrajectoryState state;
  arc::ArcConditions conds;
  std::shared_ptr< path::PathSegment > seg;
  for ( double t = 0; t < 2.5; t+=0.0001 ) {
    if ( !trajectory.getArcConditions( t, conds, seg, &seg_idx ) )
      break;
    trajectory.getState( t, state );
    std::cout << t << ";" << seg_idx
      << ";" << conds.s << ";" << conds.ds << ";" << conds.dds  << ";" << conds.j 
      << ";" << toCSV( state.x )
      << ";" << toCSV( state.pathConditions.dx ) << ";" << toCSV( state.pathConditions.ddx ) << ";" << toCSV( state.pathConditions.j ) << std::endl;
  }
  
  
  
}
