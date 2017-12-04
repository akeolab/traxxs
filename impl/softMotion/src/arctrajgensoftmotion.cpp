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


bool ArcTrajGenSoftMotion::do_init()
{
  this->sm_traj_ = SM_TRAJ();
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
  SM_TRAJ traj;
  SM_COND c_i;
  if ( !toSoftMotion( c_in, c_i ) )
    return false;
  std::vector< SM_COND > c_i_s;
  c_i_s.push_back( c_i );
  
  if ( !this->sm_traj_compute( traj, c_i_s ) )
    return false;
  std::vector< SM_COND > conds;
  if ( !this->sm_traj_conditions_at_time( traj, this->dt_, conds ) )
    return false;
  if ( !fromSoftMotion( conds[0], c_out ) )
    return false;
  
  return true;
}

bool ArcTrajGenSoftMotion::do_get_conditions_at_time(double t, traxxs::arc::ArcConditions& c_out)
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


bool ArcTrajGenSoftMotion::sm_traj_compute(SM_TRAJ& traj, const std::vector<SM_COND>& c_i)
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
    std::vector<SM_COND> c_f_s;
    std::vector<SM_LIMITS> c_max_s;
    c_f_s.push_back( c_f );
    c_max_s.push_back( c_max );
    int cret = traj.computeTraj( c_i, c_f_s, c_max_s, SM_TRAJ::SM_INDEPENDANT );
    ret &= (cret == 0);
  }
  
  return ret;
}

bool ArcTrajGenSoftMotion::sm_traj_conditions_at_time(SM_TRAJ& traj, double time, std::vector<SM_COND>& c_out)
{
  int cret = this->sm_traj_.getMotionCond( std::min( std::max( time, 0.0 ), this->getDuration() ), c_out );
  return (cret == 0);
}

