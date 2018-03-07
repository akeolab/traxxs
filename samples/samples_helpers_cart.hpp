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


#include <traxxs/path/path.hpp>

using namespace traxxs;

/**
 * \brief creates a set of consistent segments describing a (blended) cartesian corner in the (x,y) plane with orientation changes.
 * \param[in] path_bounds the bounds on the path
 */
std::vector< std::shared_ptr< path::PathSegment > > createSegmentsForCartesianTrajectory_CornerBlend( const path::PathBounds4d& path_bounds ) 
{
  //
  // we will use three Cartesian waypoints to define the path: a start, and end, and an intermediate waypoint
  //
  path::CartesianPathWaypoint pt_start, pt_wpt, pt_end;
  
  // set the position of the waypoints
  pt_start.x.p  << 0, 0, 0;
  pt_wpt.x.p    << 1, 0, 0;
  pt_end.x.p    << 1, 1, 0;
  // set the orientation of the waypoints
  pt_start.x.q  = Eigen::Quaterniond( 1, 0, 0, 0 ); // w, x, y , z
  pt_wpt.x.q    = Eigen::Quaterniond( 0, 1, 0, 0 ); // w, x, y , z
  pt_end.x.q    = Eigen::Quaterniond( 0, 0, 1, 0 ); // w, x, y , z
  
  //
  // we can define conditions on the waypoints: here we define velocities
  //
  
  // velocities at start
  pt_start.pathConditionsPosition.dx << 0, 0, 0;
  pt_start.pathConditionsOrientation.dx << 0;
  
//   // force arriving at the waypoint with null velocity to ensure acceleration continuity
//   pt_wpt.pathConditionsPosition.dx << 0, 0, 0;
//   pt_wpt.pathConditionsOrientation.dx << 0;
  
  // velocities at end
  pt_end.pathConditionsPosition.dx << 0, 0, 0;
  pt_end.pathConditionsOrientation.dx << 0;
  
  // we define the types of segments we want to use.
  // a Cartesian segment is composed of a segment type for translation, and a segment type for orientation
  // for orientation, a SmoothStep7 type (shape so that dx = 0, ddx = 0, j = 0 always at start/finish) is an ideal candidate 
  //  since it avoids to impose constraints on arc velocity/acceleration/jerk from orientation in order to preserve continuity between segments.
  // we might need two segment types: one for joining waypoints (JoiningSegment_t), the other for blends (BlendSegment_t) (i.e. smoothening of corners)
  using JoiningSegment_t  = path::CartesianSegment< path::LinearSegment, path::SmoothStep7 >;
  using BlendSegment_t    = path::CartesianSegment< path::CircularBlend, path::SmoothStep7 >;
  
  // we use an helper function to create the segments w.r.t. the waypoints and the segment types we defined.
  std::vector< path::CartesianPathWaypoint > waypoints = { pt_start, pt_wpt, pt_end};
  auto segments = path::blendedSegmentsFromWaypoints< path::CartesianPathWaypoint, JoiningSegment_t, BlendSegment_t, double>( 
   path_bounds, waypoints, 0.1 ); // std::vector< sptr< path::PathSegment > >
  
  return segments;
}
