// This file is a part of the traxxs framework.
// Copyright 2018, AKEOLAB S.A.S.
// Main contributor(s): Aurelien Ibanez, aurelien@akeo-lab.com
// 
// This software is a computer program whose purpose is to help create and manage trajectories.
// 
// This software is governed by the CeCILL-C license under French law and
// abiding by the rules of distribution of free software.  You can  use, 
// modify and/ or redistribute the software under the terms of the CeCILL-C
// license as circulated by CEA, CNRS and INRIA at the following URL
// "http://www.cecill.info". 
// 
// As a counterpart to the access to the source code and  rights to copy,
// modify and redistribute granted by the license, users are provided only
// with a limited warranty  and the software's author,  the holder of the
// economic rights,  and the successive licensors  have only  limited
// liability. 
// 
// In this respect, the user's attention is drawn to the risks associated
// with loading,  using,  modifying and/or developing or reproducing the
// software by the user in light of its specific status of free software,
// that may mean  that it is complicated to manipulate,  and  that  also
// therefore means  that it is reserved for developers  and  experienced
// professionals having in-depth computer knowledge. Users are therefore
// encouraged to load and test the software's suitability as regards their
// requirements in conditions enabling the security of their systems and/or 
// data to be ensured and,  more generally, to use and operate it in the 
// same conditions as regards security. 
// 
// The fact that you are presently reading this means that you have had
// knowledge of the CeCILL-C license and that you accept its terms.

/**
 * Demonstrates a TrackerSpacePursuit tracker
 */

#include <iostream>
#include <iomanip>
#include <memory>

#include <traxxs/tracker/tracker.hpp>
#include <traxxs/impl/traxxs_softmotion/traxxs_softmotion.hpp>
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
  
  auto segments = createSegmentsForCartesianTrajectory_CornerBlend( path_bounds );
  
  auto trajectory = std::make_shared< trajectory::Trajectory >();
  if ( !trajectory->set< ArcTrajGenSoftMotion >( segments ) )
    return 1;
  
  // the current system state and desired (from trajectory) system state
  trajectory::TrajectoryState state, state_new;
  
  // the status of the tracker
  tracker::TrackerStatus status;
  // the validator defining when we should increment the trajectory
  auto validator = std::make_shared< tracker::TrackerSpaceValidatorPose4d >( 1.e-3, 1.e-2 ); // position tolerance, angle tolerance
  // the tracker itself
  tracker::TrackerSpacePursuit tracker( validator );
  // feed the trajectory to the tracker
  tracker.reset( trajectory );
  
  double dt = 0.0001;
  double t = 0 - dt;
  // set the system initial state and store it into 'state'
  if ( !trajectory->getState( 0.0, state) ) {
    std::cerr << "Could not get state !\n";
    return 1;
  }
  state.x[0] += 1.0; // add some offset to initial state
  
  std::cout << "{" << std::endl;
  std::cout << "\"data\": [" << std::endl;
  for ( unsigned int i = 0; i < 20.0/dt; ++i ) {
    t += dt;
    // ask the tracker what we should be the next system state according to the current state and to the trajectory
    status = tracker.next( dt, state, state_new );
    // if the tracker fails
    if ( status == tracker::TrackerStatus::Error ) {
      std::cerr << "Tracker error.\n";
      break;
    }
    // if the tracker states that we reached the end of the trajectory
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
