/**
 * \todo
 */

#include <iostream>
#include <iomanip>
#include <memory>

#include <traxxs/tracker/tracker.hpp>
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
  state.x[0] += 1.0;

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
