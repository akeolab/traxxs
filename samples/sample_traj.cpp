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
  std::cout << "{" << std::endl;
  std::cout << "\"data\": [" << std::endl;
  for ( double t = 0; t < 2.5; t+=0.0001 ) {
    if ( !trajectory.getArcConditions( t, conds, seg, &seg_idx ) )
      break;
    trajectory.getState( t, state );
    std::cout << "{" << trajectoryFrameToJSON( t, seg_idx, conds, state ) << "}," << std::endl;
  }
  std::cout << "{} ]" << std::endl;
  std::cout << "}" << std::endl;
  
  
  
}
