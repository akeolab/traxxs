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
  
  trajectory::TrajectoryState state, state_new;
  arc::ArcConditions conds;
  std::shared_ptr< path::PathSegment > seg;
  
  tracker::TrackerStatus status;
  tracker::TrackerTimePursuit tracker;
  tracker.reset( trajectory );
  
  double dt = 0.0001;
  double t = 0 - dt;
  // initial state
  trajectory->getState( 0.0, state);
  state.x[0] += 1.0; // add some offset to initial state
  
  std::cout << "{" << std::endl;
  std::cout << "\"data\": [" << std::endl;
  for ( unsigned int i = 0; i < 1000000; ++i ) {
    t += dt;
    status = tracker.next( dt, state, state_new );
    if ( status == tracker::TrackerStatus::Error ) {
      std::cerr << "Tracker error.\n";
      break;
    }
    if ( status == tracker::TrackerStatus::Finished )
      break;
    
    // fake system+controller dynamics: slow translation tracking
    dummySystemTranslationKp( state_new, state );
    
    std::cout << "{" << trajectoryFrameToJSON( t, state, state_new ) << "}," << std::endl;
  }
  std::cout << "{} ]" << std::endl;
  std::cout << "}" << std::endl;
  
  return 0;
  
  
}
