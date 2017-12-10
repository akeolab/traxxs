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
  
  path::CartesianPathConditions pt_start, pt_wpt, pt_end;
  
  pt_start.position.p  << 0, 0, 0;
  pt_wpt.position.p    << 1, 0, 0;
  pt_end.position.p    << 1, 1, 0;
  
  pt_start.position.q  = Eigen::Quaterniond( 1, 0, 0, 0 ); // w, x, y , z
  pt_wpt.position.q    = Eigen::Quaterniond( 0, 1, 0, 0 ); // w, x, y , z
  pt_end.position.q    = Eigen::Quaterniond( 0, 0, 1, 0 ); // w, x, y , z
  
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

  std::vector< path::CartesianPathConditions > waypoints = { pt_start, pt_wpt, pt_end};
  auto segments = path::blendedSegmentsFromCartesianWaypoints< JoiningSegment_t, BlendSegment_t, double>( 
   path_bounds, waypoints, 0.1 ); // std::vector< sptr< path::PathSegment > >
  path::Path path( segments );
  
  path.init();
  
  std::vector< std::shared_ptr< arc::ArcTrajGen > > arctrajgens;
  for ( unsigned int i = 0; i < path.getSegments().size(); ++i )
    arctrajgens.push_back( std::make_shared< ArcTrajGenSoftMotion >() );
  trajectory::Trajectory trajectory;
  
  trajectory.set( path.getSegments(), arctrajgens );
  
//   return 0;
  
  int seg_idx;
  trajectory::TrajectoryState state;
  arc::ArcConditions conds;
  std::shared_ptr< path::PathSegment > seg;
  for ( double t = 0; t < 4.0; t+=0.0001 ) {
    if ( !trajectory.getArcConditions( t, conds, seg, &seg_idx ) )
      break;
    trajectory.getState( t, state );
    std::cout << t << ";" << seg_idx
      << ";" << conds.s << ";" << conds.ds << ";" << conds.dds  << ";" << conds.j 
      << ";" << toCSV( state.x )
      << ";" << toCSV( state.dx ) << ";" << toCSV( state.ddx ) << ";" << toCSV( state.j ) << std::endl;
  }
  
  
  
}
