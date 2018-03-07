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

#ifndef TRAXXS_TRAJECTORY_H
#define TRAXXS_TRAJECTORY_H

#include <traxxs/arc/arc.hpp>
#include <traxxs/path/path.hpp>

namespace traxxs {
namespace trajectory {
  
using TrajectoryState = path::PathWaypoint;

class TrajectorySegment 
{
 public: 
  TrajectorySegment( std::shared_ptr< path::PathSegment >& seg, std::shared_ptr< arc::ArcTrajGen >& traj )
      : seg_( seg ), traj_( traj ){
  }
  virtual ~TrajectorySegment(){};
  
 public:
  std::shared_ptr< path::PathSegment > getPathSegment() const { return this->seg_; };
  std::shared_ptr< arc::ArcTrajGen > getArcTrajGen() const { return this->traj_; };
   
 protected:
  std::shared_ptr< path::PathSegment > seg_ = nullptr;
  std::shared_ptr< arc::ArcTrajGen > traj_ = nullptr;
};

/** 
 * \brief a trajectory object, consisting in a path with arc generators corresponding to the segments of the path
 * \note the "time" parameter at this level is purely virtual. 
 * \warning Any changes in the segments (their bounds, for example) might disturb this virtual timeline. Moreover, previously computed ArcTrajGens are ignored.
 */
class Trajectory 
{
 public:
  Trajectory(){};
  
 public:
   /** \brief sets the trajectory with segments and arc trajectory generators. All trajectory generators will be fed with their respective segment start/end conditions and bounds */
  virtual bool set( const std::vector< std::shared_ptr< path::PathSegment > >& segments, const std::vector< std::shared_ptr< arc::ArcTrajGen > >& arctrajgens );
  
  /** \brief sets the trajectory with segments and an arc trajectory class. All trajectory generators will be created with default ctor and will be fed with their respective segment start/end conditions and bounds */
  template < class ArcTrajGen_t >
  bool set( const std::vector< std::shared_ptr< path::PathSegment > >& segments ) {
    std::vector< std::shared_ptr< arc::ArcTrajGen > > arctrajgens;
    for ( unsigned int i = 0; i < segments.size(); ++i )
      arctrajgens.push_back( std::make_shared< ArcTrajGen_t >() );
    return this->set( segments, arctrajgens );
  }
  
 public:
  virtual bool setPathBounds( double time, const path::PathBounds& path_bounds );
   
 public: 
  virtual bool getState( double time, TrajectoryState& state_out, int* idx_out = nullptr, bool* is_beyond = nullptr );
  virtual bool getArcConditions( double time, arc::ArcConditions& conds_out, std::shared_ptr<path::PathSegment>& segment_out, int* idx_out = nullptr, bool* is_beyond = nullptr );
  /**
   * \brief gets the corresponding segment index at a given time.
   * \param[in]   time      the virtual time
   * \param[out]  idx_out   the segment index
   * \param[out]  time_on_segment_out the time spent on identified segment, will return 
   */
  virtual bool getSegmentIndex( double time, int& idx_out, double* time_on_segment_out = nullptr, bool* is_beyond = nullptr );
  
  /** 
   * \brief triggers the computation for all arc trajectory generators 
   * to be used if computations are too heavy to be made on-the-fly. Otherwise trajectories will be computed on demand.
   */
  virtual bool computeAll( bool force = false );
  
 protected:
  virtual bool computeAtIndex( unsigned int idx, bool force = false );
   
 protected:
  std::shared_ptr< path::Path > path_ = nullptr;
  std::vector< std::shared_ptr< trajectory::TrajectorySegment > > trajsegments_;

};

} // namespace traxxs
} // namespace trajectory 

#endif // TRAXXS_TRAJECTORY_H
