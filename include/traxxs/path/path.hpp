#ifndef TRAXXS_PATH_PATH_H
#define TRAXXS_PATH_PATH_H

#include "segment.hpp"

namespace traxxs {
namespace path {
  
/** \brief enforces rules on start/end waypoints for blend segments */
void blendStartEndFromWaypoints( CartesianPathConditions& wpt_start, CartesianPathConditions& wpt_end, CartesianPathConditions& wpt );

/** \brief enforces rules on start/end waypoints for blend segments */
void blendStartEndFromWaypoints( PathConditions& wpt_start, PathConditions& wpt_end, PathConditions& wpt );



template < class Segment_t, class BlendSegment_t, typename... Args >
std::vector< std::shared_ptr < PathSegment > > blendedSegmentsFromCartesianWaypoints( const PathBounds& path_bounds, const std::vector< CartesianPathConditions >& waypoints, Args... args ) {
  
  std::vector< std::shared_ptr < PathSegment > > segments; // will hold all segments, including blends. This is the returned value
  std::vector< std::shared_ptr < PathSegment > > blends; // will hold the blends solely
  
  // first create all blend segments BlendSegment_t for waypoints, i.e. every waypoint except first and last
  CartesianPathConditions start, wpt, end;
  for ( unsigned int iwpt = 1; iwpt < waypoints.size() - 1; ++iwpt ) {
    start = waypoints[iwpt-1];
    wpt   = waypoints[iwpt];
    end   = waypoints[iwpt+1];
    blendStartEndFromWaypoints( start, end, wpt );
    std::shared_ptr < PathSegment > seg = std::make_shared< BlendSegment_t >(
      start, end, path_bounds, wpt.position.p, args... );
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
    end = CartesianPathConditions(); // reset
    end.position = blend->getPosition( tmp ); 
    
    std::shared_ptr < PathSegment > seg = std::make_shared< Segment_t >(
      start, end, path_bounds );
    seg->init(); // blend has already been init'ed
    segments.push_back( seg );
    segments.push_back( blend );
    
    tmp.s = blend->getLength(); // the start of the NEXT joining segment is the end of the PREVIOUS blend
    start = CartesianPathConditions(); // reset
    start.position = blend->getPosition( tmp );
    
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
 * Will ensure:
 * - Arc bounds on each segments are compatible with Path bounds on each segments
 * - Arc start/end conditions ensure dx continuity at all costs, i.e. through dx=0 if needed.
 * - Arc start/end conditions ensure ddx and dddx continuity if possible
 */
class Path
{
 public:
  Path( std::vector< std::shared_ptr < PathSegment > > segments );
 public:
  virtual bool init();
  virtual const std::vector< std::shared_ptr < PathSegment > >& getSegments() { return this->segments_; };
 protected: 
  std::vector< std::shared_ptr < PathSegment > > segments_;
};

} // namespace traxxs 
} // namespace path 

#endif // TRAXXS_PATH_PATH_H

