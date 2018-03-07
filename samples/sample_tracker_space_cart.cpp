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
  
//   std::cout << std::setprecision(3) << std::fixed << std::showpos ;
  
  path::PathBounds4d path_bounds;
  path_bounds.dx << 1.0, 1.0, 1.0, 10.0;
  path_bounds.ddx = 10.0 * path_bounds.dx;
  path_bounds.j = 10.0 * path_bounds.ddx;
  
  auto segments = createSegmentsForCartesianTrajectory_Square( path_bounds );
  
  auto trajectory = std::make_shared< trajectory::Trajectory >();
  if ( !trajectory->set< ArcTrajGenSoftMotion >( segments ) )
    return 1;
  
  trajectory::TrajectoryState state, state_new, state_tmp;
  arc::ArcConditions conds;
  std::shared_ptr< path::PathSegment > seg;
  
  tracker::TrackerStatus status;
  auto validator = std::make_shared< tracker::TrackerSpaceValidatorPose4d >( 1.e-3, 1.e-2 ); // position tolerance, angle tolerance
  tracker::TrackerSpacePursuit tracker( validator );
  tracker.reset( trajectory );
  
  double dt = 0.0001;
  double t = 0 - dt;
  // initial state
  trajectory->getState( 0.0, state);
  state.x[0] += 1.0; // add some offset to initial state

  for ( unsigned int i = 0; i < 20.0/dt; ++i ) {
    t += dt;
    status = tracker.next( dt, state, state_new );
    if ( status == tracker::TrackerStatus::Error ) {
      std::cerr << "Tracker error.\n";
      break;
    }
    if ( status == tracker::TrackerStatus::Finished )
      break;
    
    // fake system+controller dynamics: slow translation tracking
    state_tmp = state;
    state = state_new;
    state.x.segment(0,3) = state_tmp.x.segment(0,3);
    state.x.segment(0,3) += 0.05 * ( state_new.x.segment(0,3) - state_tmp.x.segment(0,3) );
    
    std::cout << t << ";" << 0
      << ";" << 0 << ";" << 0 << ";" << 0  << ";" << 0 
      << ";" << toCSV( state.x )
      << ";" << toCSV( state.pathConditions.dx ) << ";" << toCSV( state.pathConditions.ddx ) << ";" << toCSV( state.pathConditions.j ) << std::endl;
  }
  
  return 0;
  
  
}
