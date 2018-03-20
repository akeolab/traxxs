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

#include <traxxs/trajectory/trajectory.hpp>

bool traxxs::trajectory::Trajectory::set( const std::vector< std::shared_ptr< path::PathSegment > >& segments, const std::vector< std::shared_ptr< arc::ArcTrajGen > >& arctrajgens )
{
  if ( segments.size() != arctrajgens.size() )
    return false;
  /** \todo should we take ownership of the segments to avoid issues ? */
  this->path_ = std::make_shared< path::Path >( segments );
  /** \todo path_ should be reconstructed on-the-fly, i.e. (sliding) pair (of segments) by pair. */
  if ( !this->path_->init() )
    return false;
  this->trajsegments_.clear();
  
  std::shared_ptr< path::PathSegment > seg;
  std::shared_ptr< arc::ArcTrajGen > traj;
  for ( unsigned int iseg = 0; iseg < this->path_->getSegments().size(); ++iseg ) {
    seg = this->path_->getSegments()[iseg];
    traj = arctrajgens[iseg];
    traj->init();
    traj->setInitialConditions( seg->getStartArcConditions() );
    traj->setFinalConditions( seg->getEndArcConditions() );
    traj->setMaxConditions( seg->getArcBounds() );
    /** \todo should we take ownership of the traj to avoid issues ? */
    this->trajsegments_.push_back(
      std::make_shared< TrajectorySegment >( seg, traj )
    );
  }
  return true;
}

bool traxxs::trajectory::Trajectory::setPathBounds(double time, const path::PathBounds& path_bounds)
{
  bool ret = true;
  // find the segment at time, and the conditions on this segment
  int seg_idx;
  double time_on_segment;
  arc::ArcConditions conds_at_time;
  if ( !this->getSegmentIndex( time, seg_idx, &time_on_segment ) )
    return false;
  if ( !this->trajsegments_[seg_idx]->getArcTrajGen()->getConditionsAtTime( time_on_segment, conds_at_time ) )
    return false;
  // split the segment in two segments: one with the previous bounds until conds_at_time.s, the other with the new bounds from conds_at_time.s
  arc::ArcConditions start, middle, end;
  std::shared_ptr< arc::ArcTrajGen > traj = this->trajsegments_[seg_idx]->getArcTrajGen();
  std::shared_ptr< arc::ArcTrajGen > new_traj( traj.get()->clone() );
  std::shared_ptr< path::PathSegment > path = this->trajsegments_[seg_idx]->getPathSegment();
  std::shared_ptr< path::PathSegment > new_path( path.get()->clone() );
  start = traj->getInitialConditions();
  end = traj->getFinalConditions();
  middle = conds_at_time;
  traj->setFinalConditions( middle );
  new_traj->setInitialConditions( middle );
  // set the new path_bounds to the newly created path and further
  new_path->setPathBounds( path_bounds );
  // update the start/end arc conditions
  new_path->setStartArcConditions( middle );
  new_path->setEndArcConditions( end );
  path->setEndArcConditions( middle );

  for ( unsigned int iseg = seg_idx+1; iseg < this->path_->getSegments().size(); ++iseg ) 
    this->trajsegments_[iseg]->getPathSegment()->setPathBounds( path_bounds );
  // insert the new segment
  this->trajsegments_.insert( this->trajsegments_.begin() + seg_idx+1, std::make_shared< TrajectorySegment >( new_path, new_traj ) );
  // then rebuild the path
  std::vector< std::shared_ptr< path::PathSegment > > segments(this->trajsegments_.size());
  for ( unsigned int iseg = 0; iseg < this->trajsegments_.size(); ++iseg )
    segments.at( iseg ) = this->trajsegments_[iseg]->getPathSegment();
  this->path_ = std::make_shared< path::Path >( segments );
  /** \todo path_ should be reconstructed on-the-fly, i.e. (sliding) pair (of segments) by pair. */
  /** \warning when reconstructing the path, the problem might arise that the new path bounds were not respected at this time. 
   * Hence when we reconstruct, this imposes new (over)constraints at the split.
   * Therefore, we final conditions for the first part of the split are constrained by the bounds on the second split.
   * As a consequence, the first part of the split (i.e., potentially the past) may take longer than what really happened until time 'time'
   * We must ignore this "new" duration of the past split segment.
   * The problem is: how can we know if the past split is effectively in the past ?
   * 
   * Illustration: 
   * at t = tx, I'm on segment 0 at s=0.5, ds = 1.0
   * I split at ts > tx, imposing ds_max = 0.5 from the split on
   * Potentially, I can have that at t = tx (the same), I now have s = 0.25 ! (because of continuity constraint at the split)
   * This leads to discontinuities in the trajectory.
   * The problem is that we cannot guarantee that we can find somewhere on segment 0 to match the conditions we were (s=0.5, ds = 1.0)
   * 
   * Conclusion: we reinit the path solely starting at iseg + 1
   */
  if ( !this->path_->init( seg_idx + 1 ) )
    return false;
  
  // and reset the trajs
  /** \note we start at iseg+1 since changes in the path are expected to occur solely starting at iseg+1 
   * \warning might reverberate up to iseg (and no more) but we ignore this cf. remark above
   */
  for ( unsigned int iseg = seg_idx+1; iseg < this->path_->getSegments().size(); ++iseg ) {
    traj = this->trajsegments_[iseg]->getArcTrajGen();
    /** \fixme at this point, we just want to state that the ArcTrajGens should be recomputed. Using ArcTrajGen::init() is probably overkill */
    traj->init();
    if ( iseg > seg_idx+1 ) // we do not want to change the initial conditions for iseg+1 since the final conditions for iseg have been ignore
      traj->setInitialConditions( this->path_->getSegments()[iseg]->getStartArcConditions() );
    traj->setFinalConditions( this->path_->getSegments()[iseg]->getEndArcConditions() );
    traj->setMaxConditions( this->path_->getSegments()[iseg]->getArcBounds() );
  }
  return ret;
}


bool traxxs::trajectory::Trajectory::getState( double time, traxxs::trajectory::TrajectoryState& state_out, int* idx_out /*= nullptr*/, bool* is_beyond /*= nullptr*/ )
{
  arc::ArcConditions conds;
  std::shared_ptr<path::PathSegment> seg;
  bool ret = this->getArcConditions( time, conds, seg, idx_out, is_beyond );
  if( !ret )
    return ret;
  state_out.x   = seg->getPosition( conds );
  state_out.pathConditions.dx  = seg->getVelocity( conds );
  state_out.pathConditions.ddx = seg->getAcceleration( conds );
  state_out.pathConditions.j   = seg->getJerk( conds );
  
  return ret; 
}

bool traxxs::trajectory::Trajectory::getArcConditions( double time, arc::ArcConditions& conds_out, std::shared_ptr<path::PathSegment>& segment_out, int* idx_out /*= nullptr*/, bool* is_beyond /*= nullptr*/ )
{
  bool ret = true;
  int segment_idx;
  double time_on_segment;
  if ( !this->getSegmentIndex( time, segment_idx, &time_on_segment, is_beyond ) )
    return false;
  
  if( idx_out != nullptr )
    *idx_out = segment_idx;
  segment_out = this->path_->getSegments()[segment_idx];
  
  ret &= this->trajsegments_[segment_idx]->getArcTrajGen()->getConditionsAtTime( time_on_segment, conds_out );
  
  return ret;
}

bool traxxs::trajectory::Trajectory::getSegmentIndex( double time, int& idx_out, double* time_on_segment_out /*= nullptr*/, bool* is_beyond /*= nullptr*/ ) {
  if ( this->path_ == nullptr )
    return false;
  bool ret = true;
  // find the corresponding segment, and compute the arctrajgen if needed
  int segment_idx = -1;
  double time_on_segment = time;
  std::shared_ptr< arc::ArcTrajGen > traj;
  for ( unsigned int iseg = 0; iseg < this->path_->getSegments().size(); ++iseg ) {
    traj = this->trajsegments_[iseg]->getArcTrajGen();
    if ( !traj->isComputed() )
      ret &= traj->compute();
    if ( !ret ) {
      std::cerr << "traxxs::trajectory::Trajectory::getSegmentIndex Failed at " << iseg << std::endl;
      return ret;
    }
    if ( time_on_segment <= traj->getDuration() ) {
      segment_idx = iseg;
      break;
    }
    time_on_segment += -traj->getDuration();
  }
  
  if ( is_beyond != nullptr )
      *is_beyond = false;
  if ( segment_idx < 0 ) { // we are further than the last segment
    segment_idx = this->path_->getSegments().size() - 1;
    time_on_segment = std::nan(""); // will be set later
    traj = this->trajsegments_[segment_idx]->getArcTrajGen();
    time_on_segment = traj->getDuration();
    if ( is_beyond != nullptr )
      *is_beyond = true;
  }
  
  idx_out = segment_idx;
  if ( time_on_segment_out != nullptr )
    *time_on_segment_out = time_on_segment;
  return ret;
}



bool traxxs::trajectory::Trajectory::computeAll( bool force /*= false*/ )
{
  bool ret = true;
  for ( unsigned int iseg = 0; iseg < this->path_->getSegments().size(); ++iseg )
    ret &= this->computeAtIndex( iseg, force );
  return ret;
}

bool traxxs::trajectory::Trajectory::computeAtIndex( unsigned int idx, bool force /*= false*/ )
{
  std::cerr << "\tComputing: " << idx << std::endl;
  if ( idx >= this->path_->getSegments().size() )
    return false;
  if ( force || !this->trajsegments_[idx]->getArcTrajGen()->isComputed() )
    return this->trajsegments_[idx]->getArcTrajGen()->compute();
  return true;
}
