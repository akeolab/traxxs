#ifndef TRAXXS_TRAJECTORY_H
#define TRAXXS_TRAJECTORY_H

#include <traxxs/arc/arc.hpp>
#include <traxxs/path/path.hpp>

namespace traxxs {
namespace trajectory {
  
using TrajectoryState = path::PathConditions;

/** 
 * \brief a trajectory object, consisting in a sequence of path segments with corresponding arc generators 
 */
class Trajectory {
 public:
  Trajectory(){};
  
 public:
   /** \brief sets the trajectory with segments and arc trajectory generators. All trajectory generators will be fed with their respective segment start/end conditions and bounds */
  virtual bool set( const std::vector< std::shared_ptr< path::PathSegment > >& segments, const std::vector< std::shared_ptr< arc::ArcTrajGen > >& arctrajgens );
  
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
  std::vector< std::shared_ptr< path::PathSegment > > segments_;
  std::vector< std::shared_ptr< arc::ArcTrajGen > > arctrajgens_;
};

} // namespace traxxs
} // namespace trajectory 

#endif // TRAXXS_TRAJECTORY_H
