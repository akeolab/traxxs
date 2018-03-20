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

#ifndef TRAXXS_PATH_PATH_H
#define TRAXXS_PATH_PATH_H

#include "segment.hpp"

namespace traxxs {
namespace path {
  
/** \brief enforces rules on start/end waypoints for blend segments */
void blendStartEndFromWaypoints( CartesianPathWaypoint& wpt_start, CartesianPathWaypoint& wpt_end, CartesianPathWaypoint& wpt );

/** \brief enforces rules on start/end waypoints for blend segments */
void blendStartEndFromWaypoints( PathWaypoint& wpt_start, PathWaypoint& wpt_end, PathWaypoint& wpt );


template < class BlendSegment_t, typename... Args >
std::shared_ptr < PathSegment > createBlendSegmentPositionOnly( const PathBounds& path_bounds, const CartesianPathWaypoint& start, const CartesianPathWaypoint& end, const CartesianPathWaypoint& wpt, Args... args ) {
  return std::make_shared< BlendSegment_t >(
      start, end, path_bounds, wpt.x.p, args... );
}  

template < class BlendSegment_t, typename... Args >
std::shared_ptr < PathSegment > createBlendSegmentPositionOnly( const PathBounds& path_bounds, const PathWaypoint& start, const PathWaypoint& end, const PathWaypoint& wpt, Args... args ) {
  return std::make_shared< BlendSegment_t >(
      start, end, path_bounds, wpt.x, args... );
}  

template < class Waypoint_t, class Segment_t, class BlendSegment_t, typename... Args >
std::vector< std::shared_ptr < PathSegment > > blendedSegmentsFromWaypoints( const PathBounds& path_bounds, const std::vector< Waypoint_t >& waypoints, Args... args ) {
  
  std::vector< std::shared_ptr < PathSegment > > segments; // will hold all segments, including blends. This is the returned value
  std::vector< std::shared_ptr < PathSegment > > blends; // will hold the blends solely
  
  // first create all blend segments BlendSegment_t for waypoints, i.e. every waypoint except first and last
  Waypoint_t start, wpt, end;
  for ( unsigned int iwpt = 1; iwpt < waypoints.size() - 1; ++iwpt ) {
    start = waypoints[iwpt-1];
    wpt   = waypoints[iwpt];
    end   = waypoints[iwpt+1];
    blendStartEndFromWaypoints( start, end, wpt );
    std::shared_ptr < PathSegment > seg = createBlendSegmentPositionOnly< BlendSegment_t, Args... >( path_bounds, start, end, wpt, args... );
    // initialize the blend segment so that we can extract its start/end
    seg->init();
    blends.push_back( seg );
  }
  
  // now join all blend segments with Segment_t 
  arc::ArcConditions tmp;
  for ( unsigned int iblend = 0; iblend < blends.size(); ++iblend ) {
    std::shared_ptr < PathSegment > blend = blends[iblend];
    if ( iblend == 0 )
      start = waypoints[0]; // if first blend, we have to join from the first waypoint
    
    tmp.s = 0.0; // the end of the joining segment is the start of the blend
    end = Waypoint_t(); // reset
    end.x = blend->getPosition( tmp ); 
    
    std::shared_ptr < PathSegment > seg = std::make_shared< Segment_t >(
      start, end, path_bounds );
    seg->init(); // blend has already been init'ed
    segments.push_back( seg );
    segments.push_back( blend );
    
    tmp.s = blend->getLength(); // the start of the NEXT joining segment is the end of the PREVIOUS blend
    start = Waypoint_t(); // reset
    start.x = blend->getPosition( tmp );
    
    // finish cleanly
    if ( iblend == blends.size()-1 ) {
      end = waypoints[waypoints.size()-1];
      segments.push_back( 
        std::make_shared< Segment_t >(
          start, end, path_bounds )
      );
      segments.back()->init();
    }
  }
  
  return segments;
}

/** 
 * Will compute:
 * - Arc bounds on each segments with respect to Path bounds on each segments
 * Will ensure:
 * - Arc start/end conditions ensure dx continuity at all costs, i.e. through dx=0 if needed.
 * - Arc start/end conditions ensure ddx and dddx continuity if possible
 */
class Path
{
 public:
  Path( std::vector< std::shared_ptr < PathSegment > > segments );
 public:
  virtual bool init( int i_start = -1 );
  virtual const std::vector< std::shared_ptr < PathSegment > >& getSegments() const { return this->segments_; };
 protected: 
  std::vector< std::shared_ptr < PathSegment > > segments_;
};

} // namespace traxxs 
} // namespace path 

#endif // TRAXXS_PATH_PATH_H

