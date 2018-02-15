#ifndef TRAXXS_TRAJECTORY_H
#define TRAXXS_TRAJECTORY_H

#include <traxxs/arc/arc.hpp>
#include <traxxs/path/path.hpp>

namespace traxxs {
namespace trajectory {
  
using TrajectoryState = path::PathWaypoint;


/** 
 * \brief a trajectory object, consisting in a path with arc generators corresponding to the segments of the path
 * \note the "time" parameter at this level is purely virtual. 
 * \warning Any changes in the segments (their bounds, for example) might disturb this virtual timeline. Moreover, previously computed ArcTrajGens are ignored.
 */
class Trajectory {
 public:
  Trajectory(){};
  
 public:
   /** \brief sets the trajectory with segments and arc trajectory generators. All trajectory generators will be fed with their respective segment start/end conditions and bounds */
  virtual bool set( const std::shared_ptr< path::Path >& path, const std::vector< std::shared_ptr< arc::ArcTrajGen > >& arctrajgens );
  
  /** \brief sets the trajectory with segments and an arc trajectory class. All trajectory generators will be created with default ctor and will be fed with their respective segment start/end conditions and bounds */
  template < class ArcTrajGen_t >
  bool set( const std::shared_ptr< path::Path >& path ) {
    std::vector< std::shared_ptr< arc::ArcTrajGen > > arctrajgens;
    for ( unsigned int i = 0; i < path->getSegments().size(); ++i )
      arctrajgens.push_back( std::make_shared< ArcTrajGen_t >() );
    return this->set( path, arctrajgens );
  }
  
  virtual bool getState( double time, TrajectoryState& state_out, int* idx_out = nullptr );
  virtual bool getArcConditions( double time, arc::ArcConditions& conds_out, std::shared_ptr<path::PathSegment>& segment_out, int* idx_out = nullptr );
  
  /** 
   * \brief triggers the computation for all arc trajectory generators 
   * to be used if computations are too heavy to be made on-the-fly. Otherwise trajectories will be computed on demand.
   */
  virtual bool computeAll();
  
 protected:
  virtual bool computeAtIndex( unsigned int idx );
   
 protected:
  std::shared_ptr< path::Path > path_ = nullptr;
  std::vector< std::shared_ptr< arc::ArcTrajGen > > arctrajgens_;

};

} // namespace traxxs
} // namespace trajectory 

#endif // TRAXXS_TRAJECTORY_H
