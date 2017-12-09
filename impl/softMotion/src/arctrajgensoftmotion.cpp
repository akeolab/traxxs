#include "arctrajgensoftmotion.hpp"

#include <stdexcept>

bool toSoftMotion(const traxxs::arc::ArcConditions& c_in, SM_COND& c_out)
{
  if ( std::isnan( c_in.s ) || std::isnan( c_in.ds ) || std::isnan( c_in.dds ) )
    return false;
  c_out.x = c_in.s;
  c_out.v = c_in.ds;
  c_out.a = c_in.dds;
  return true;
}

bool toSoftMotion(const traxxs::arc::ArcConditions& c_in, SM_LIMITS& c_out)
{
  if ( std::isnan( c_in.ds ) || std::isnan( c_in.dds ) || std::isnan( c_in.j ) )
    return false;
  c_out.maxVel  = c_in.ds;
  c_out.maxAcc  = c_in.dds;
  c_out.maxJerk = c_in.j;
  return true;
}

bool fromSoftMotion( const SM_COND& c_in, traxxs::arc::ArcConditions& c_out, double time /*= std::nan("")*/ )
{
  c_out.s   = c_in.x;
  c_out.ds  = c_in.v;
  c_out.dds = c_in.a;
  if ( !std::isnan( time ) )
    c_out.t = time;
  return true;
}

bool fromSoftMotion( const SM_LIMITS& limits_in, traxxs::arc::ArcConditions& c_out, double time /*= std::nan("")*/ )
{
  c_out.ds  = limits_in.maxVel;
  c_out.dds = limits_in.maxAcc;
  c_out.j   = limits_in.maxJerk;
  if ( !std::isnan( time ) )
    c_out.t = time;
  return true;
}

SmTrajWrapper::SmTrajWrapper()
{
  this->sm_traj_ = SM_TRAJ();
  this->brake_duration_ = 0.0;
}

int SmTrajWrapper::computeTraj( std::vector<SM_COND> IC, std::vector<SM_COND> FC, std::vector<SM_LIMITS> limits, SM_TRAJ::SM_TRAJ_MODE mode ) 
{
  // made only for 1D trajectories, in SM_TRAJ::SM_INDEPENDANT mode
  if ( IC.size() != 1 || FC.size() != 1 || limits.size() != 1 || mode != SM_TRAJ::SM_INDEPENDANT )
    throw std::invalid_argument( "SmTrajWrapper only handles 1D trajectories in SM_TRAJ::SM_INDEPENDANT mode." );
  
  /** \todo check if IC in bounds ! If not, IC for SM_TRAJ are different from params, resulting from braking phase */
  limits[0].maxVel = std::fabs( limits[0].maxVel );
  limits[0].maxAcc = std::fabs( limits[0].maxAcc );
  limits[0].maxJerk = std::fabs( limits[0].maxJerk );
  ic_ = IC[0];
  ic_smtraj_ = IC[0];
  if ( std::fabs( ic_.v ) <= limits[0].maxVel && std::fabs( ic_.a ) <= limits[0].maxAcc ) {
    // then it's all good
    this->brake_duration_ = 0.0;
  } else if ( std::fabs( ic_.a ) <= limits[0].maxAcc ) { // if we have an out-of-bounds velocity solely
    SM_COND brake_ic, brake_fc;
    SM_LIMITS brake_limits;
    brake_ic = ic_;
    brake_limits = limits[0];
    brake_limits.maxVel = std::fabs( brake_ic.v );
    brake_fc.v = ( ic_.v > 0 ? +1 : -1 ) * limits[0].maxVel ;
    brake_fc.a = 0.0; // set null acceleration after braking to ensure we reach a viable state
    double critical_length;
    // the critical length is the minimum distance to reach the FC within the bounds. If longer, will lead to oscillations
    int ret = sm_CalculOfCriticalLength(brake_ic, brake_fc, brake_limits, &critical_length);
    if ( ret != SM_STATUS::SM_OK )
      return ret;
    brake_fc.x = brake_ic.x + critical_length;
    ret = brake_sm_traj_.computeTraj( std::vector<SM_COND>{ brake_ic }, std::vector<SM_COND>{ brake_fc }, std::vector<SM_LIMITS>{ brake_limits }, mode );
    if ( ret != SM_STATUS::SM_OK )
      return ret;
    brake_duration_ = brake_sm_traj_.getDuration();
    std::vector<SM_COND> conds_after_brake;
    brake_sm_traj_.getMotionCond( brake_duration_, conds_after_brake );
    ic_smtraj_ = conds_after_brake[0];
  } else { // acceleration in out of bounds, with potentially out-of-bounds velocity
    /** \todo what about acceleration bounds ?
     */
    throw std::runtime_error( "Acceleration out of bounds not implemented." );
  }
  
  return this->sm_traj_.computeTraj( std::vector<SM_COND>{ ic_smtraj_ }, FC, limits, mode );
}

int SmTrajWrapper::getMotionCond(double time, std::vector<SM_COND>& cond)
{
  if ( time >= brake_duration_ ) {
    return this->sm_traj_.getMotionCond( time - this->brake_duration_, cond );
  } else {
    return this->brake_sm_traj_.getMotionCond( time, cond );
  }
}


double SmTrajWrapper::getDuration()
{
  return this->sm_traj_.getDuration() + this->brake_duration_;
}






bool ArcTrajGenSoftMotion::do_init()
{
  this->sm_traj_ = SmTraj_t();
}

bool ArcTrajGenSoftMotion::do_compute()
{
  bool ret = true;
  
  SM_COND c_i;
  ret &= toSoftMotion( this->c_i_, c_i );
  std::vector< SM_COND > c_i_s;
  c_i_s.push_back( c_i );
  if ( ret )
    ret &= this->sm_traj_compute( this->sm_traj_, c_i_s );
  
  if ( !ret )
    this->duration_ = std::nan("");
  else
    this->duration_ = this->sm_traj_.getDuration();
  
  return ret;
}

bool ArcTrajGenSoftMotion::do_compute_next_conditions( const traxxs::arc::ArcConditions& c_in, traxxs::arc::ArcConditions& c_out )
{
  if ( std::isnan( this->dt_ ) )
    return false; // without a period set, what is the "next" condition ?
  SmTraj_t traj;
  SM_COND c_i;
  if ( !toSoftMotion( c_in, c_i ) )
    return false;
  std::vector< SM_COND > c_i_s{ c_i };
  
  if ( !this->sm_traj_compute( traj, c_i_s ) )
    return false;
  std::vector< SM_COND > conds;
  if ( !this->sm_traj_conditions_at_time( traj, this->dt_, conds ) )
    return false;
  if ( !fromSoftMotion( conds[0], c_out ) )
    return false;
  
  return true;
}

bool ArcTrajGenSoftMotion::do_get_conditions_at_time( double t, traxxs::arc::ArcConditions& c_out )
{
  /** \todo implement something to store the status of the computation (UNDEF, SUCCESS, FAILURE), preferably in ArcTrajGen */
  if ( std::isnan( this->getDuration() ) )
    return false;
  std::vector< SM_COND > conds;
  if ( !this->sm_traj_conditions_at_time( this->sm_traj_, t, conds ) )
    return false;
  if ( !fromSoftMotion( conds[0], c_out, t ) )
    return false;
  return true;
}


double ArcTrajGenSoftMotion::do_get_duration()
{
  return this->duration_;
}


bool ArcTrajGenSoftMotion::sm_traj_compute( SmTraj_t& traj, const std::vector<SM_COND>& c_i )
{
  /** 
   * \todo check for nans in conditions and limits
   */
  bool ret = true;
  
  SM_COND c_f;
  SM_LIMITS c_max;
  
  ret &= toSoftMotion( this->c_f_, c_f );
  ret &= toSoftMotion( this->c_max_, c_max );
  if ( ret ) {
    std::vector<SM_COND> c_f_s{ c_f };
    std::vector<SM_LIMITS> c_max_s{ c_max };
    int cret = traj.computeTraj( c_i, c_f_s, c_max_s, SM_TRAJ::SM_INDEPENDANT );
    ret &= (cret == SM_STATUS::SM_OK);
  }
  
  return ret;
}

bool ArcTrajGenSoftMotion::sm_traj_conditions_at_time( SmTraj_t& traj, double time, std::vector<SM_COND>& c_out )
{
  int cret = traj.getMotionCond( std::min( std::max( time, 0.0 ), this->getDuration() ), c_out );
  return (cret == SM_STATUS::SM_OK);
}

