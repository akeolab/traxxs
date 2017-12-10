#include <traxxs/trajectory/trajectory.hpp>

bool traxxs::trajectory::Trajectory::set( const std::vector< std::shared_ptr< path::PathSegment > >& segments, const std::vector< std::shared_ptr< arc::ArcTrajGen > >& arctrajgens )
{
  if ( segments.size() != arctrajgens.size() )
    return false;
  this->segments_ = segments;
  this->arctrajgens_ = arctrajgens;
  
  std::shared_ptr< path::PathSegment > seg;
  std::shared_ptr< arc::ArcTrajGen > traj;
  for ( unsigned int iseg = 0; iseg < this->segments_.size(); ++iseg ) {
    seg = this->segments_[iseg];
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
  // find the corresponding segment, and compute the arctrajgen if needed
  int segment_idx = -1;
  double time_on_segment = time;
  std::shared_ptr< arc::ArcTrajGen > traj;
  for ( unsigned int iseg = 0; iseg < this->segments_.size(); ++iseg ) {
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
    segment_idx = this->segments_.size() - 1;
    time_on_segment = std::nan(""); // will be set later
  }
  
  if( idx_out != nullptr )
    *idx_out = segment_idx;
  segment_out = this->segments_[segment_idx];
  
  traj = this->arctrajgens_[segment_idx];
  if ( std::isnan( time_on_segment ) ) // in case of "overflow"
    time_on_segment = traj->getDuration();
  
  ret &= traj->getConditionsAtTime( time_on_segment, conds_out );
  
  return ret;
}



bool traxxs::trajectory::Trajectory::computeAll()
{
  bool ret = true;
  for ( unsigned int iseg = 0; iseg < this->segments_.size(); ++iseg )
    ret &= this->computeAtIndex( iseg );
  return ret;
}

bool traxxs::trajectory::Trajectory::computeAtIndex( unsigned int idx )
{
  if ( idx >= this->segments_.size() )
    return false;
  return this->arctrajgens_[idx]->compute();
}
