#include <traxxs/trajectory/trajectory.hpp>

bool traxxs::trajectory::Trajectory::set( const std::shared_ptr< path::Path >& path, const std::vector< std::shared_ptr< arc::ArcTrajGen > >& arctrajgens )
{
  if ( path->getSegments().size() != arctrajgens.size() )
    return false;
  this->path_ = path;
  this->arctrajgens_ = arctrajgens;
  
  std::shared_ptr< path::PathSegment > seg;
  std::shared_ptr< arc::ArcTrajGen > traj;
  for ( unsigned int iseg = 0; iseg < this->path_->getSegments().size(); ++iseg ) {
    seg = this->path_->getSegments()[iseg];
    traj = this->arctrajgens_[iseg];
    traj->setInitialConditions( seg->getStartArcConditions() );
    traj->setFinalConditions( seg->getEndArcConditions() );
    traj->setMaxConditions( seg->getArcBounds() );
  }
  return true;
}

bool traxxs::trajectory::Trajectory::getState( double time, traxxs::trajectory::TrajectoryState& state_out, int* idx_out /*= nullptr*/ )
{
  arc::ArcConditions conds;
  std::shared_ptr<path::PathSegment> seg;
  bool ret = this->getArcConditions( time, conds, seg, idx_out );
  if( !ret )
    return ret;
  state_out.x   = seg->getPosition( conds );
  state_out.pathConditions.dx  = seg->getVelocity( conds );
  state_out.pathConditions.ddx = seg->getAcceleration( conds );
  state_out.pathConditions.j   = seg->getJerk( conds );
  
  return ret; 
}

bool traxxs::trajectory::Trajectory::getArcConditions( double time, arc::ArcConditions& conds_out, std::shared_ptr<path::PathSegment>& segment_out, int* idx_out /*= nullptr*/ )
{
  bool ret = true;
  int segment_idx;
  double time_on_segment;
  if ( !this->getSegmentIndex( time, segment_idx, &time_on_segment ) )
    return false;
  
  if( idx_out != nullptr )
    *idx_out = segment_idx;
  segment_out = this->path_->getSegments()[segment_idx];
  
  std::shared_ptr< arc::ArcTrajGen > traj;
  traj = this->arctrajgens_[segment_idx];
  ret &= traj->getConditionsAtTime( time_on_segment, conds_out );
  
  return ret;
}

bool traxxs::trajectory::Trajectory::getSegmentIndex( double time, int& idx_out, double* time_on_segment_out /*= nullptr*/ ) {
  if ( this->path_ == nullptr )
    return false;
  bool ret = true;
  // find the corresponding segment, and compute the arctrajgen if needed
  int segment_idx = -1;
  double time_on_segment = time;
  std::shared_ptr< arc::ArcTrajGen > traj;
  for ( unsigned int iseg = 0; iseg < this->path_->getSegments().size(); ++iseg ) {
    traj = this->arctrajgens_[iseg];
    if ( std::isnan( traj->getDuration() ) )
      ret &= traj->compute();
    if ( !ret ) {
      std::cerr << "Failed at " << iseg << std::endl;
      return ret;
    }
    if ( time_on_segment <= traj->getDuration() ) {
      segment_idx = iseg;
      break;
    }
    time_on_segment += -traj->getDuration();
  }
  
  if ( segment_idx < 0 ) { // we are further than the last segment
    segment_idx = this->path_->getSegments().size() - 1;
    time_on_segment = std::nan(""); // will be set later
  }
  
  // in case of "overflow"
  if ( std::isnan( time_on_segment ) ) { 
    traj = this->arctrajgens_[segment_idx];
    time_on_segment = traj->getDuration();
  }
  
  idx_out = segment_idx;
  if ( time_on_segment_out != nullptr )
    *time_on_segment_out = time_on_segment;
  return ret;
}



bool traxxs::trajectory::Trajectory::computeAll()
{
  bool ret = true;
  for ( unsigned int iseg = 0; iseg < this->path_->getSegments().size(); ++iseg )
    ret &= this->computeAtIndex( iseg );
  return ret;
}

bool traxxs::trajectory::Trajectory::computeAtIndex( unsigned int idx )
{
  if ( idx >= this->path_->getSegments().size() )
    return false;
  return this->arctrajgens_[idx]->compute();
}
