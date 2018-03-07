/**
 * \todo
 */

#include <iostream>
#include <iomanip>
#include <memory>

#include <traxxs/tracker/tracker.hpp>
#include "arctrajgensoftmotion.hpp"
#include "samples_helpers.hpp"
#include "samples_helpers_cart.hpp"

template< class T >
using sptr = std::shared_ptr<T>;

using namespace traxxs;

int main(void) {
  
  path::PathBounds4d path_bounds;
  path_bounds.dx << 1.0, 1.0, 1.0, 10.0;
  path_bounds.ddx = 10.0 * path_bounds.dx;
  path_bounds.j = 10.0 * path_bounds.ddx;
  
  auto segments = createSegmentsForCartesianTrajectory_CornerBlend( path_bounds );
  
  auto trajectory = std::make_shared< trajectory::Trajectory >();
  if ( !trajectory->set< ArcTrajGenSoftMotion >( segments ) )
    return 1;
  
  trajectory::TrajectoryState state;
  arc::ArcConditions conds;
  std::shared_ptr< path::PathSegment > seg;
  
  tracker::TrackerStatus status;
  tracker::TrackerTimePursuit tracker;
  tracker.reset( trajectory );
  
  double dt = 0.0001;
  double t = 0 - dt;
  
  for ( unsigned int i = 0; i < 1000000; ++i ) {
    t += dt;
    status = tracker.next( dt, state, state );
    if ( status == tracker::TrackerStatus::Error ) {
      std::cerr << "Tracker error.\n";
      break;
    }
    if ( status == tracker::TrackerStatus::Finished )
      break;
    
    std::cout << t << ";" << 0
      << ";" << 0 << ";" << 0 << ";" << 0  << ";" << 0 
      << ";" << toCSV( state.x )
      << ";" << toCSV( state.pathConditions.dx ) << ";" << toCSV( state.pathConditions.ddx ) << ";" << toCSV( state.pathConditions.j ) << std::endl;
  }
  
  return 0;
  
  
}
