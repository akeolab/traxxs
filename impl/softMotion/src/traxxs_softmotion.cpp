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

#include <softMotion.h>

#include <traxxs/impl/traxxs_softmotion/traxxs_softmotion.hpp>

#include <stdexcept>

class SmTrajWrapper;

// using SmTraj_t = SM_TRAJ;
using SmTraj_t = SmTrajWrapper;

bool toSoftMotion( const traxxs::arc::ArcConditions& c_in, SM_COND& c_out );
bool toSoftMotion( const traxxs::arc::ArcConditions& c_in, SM_LIMITS& c_out );

bool fromSoftMotion( const SM_COND& c_in, traxxs::arc::ArcConditions& c_out, double time = std::nan("") );
bool fromSoftMotion( const SM_LIMITS& limits_in, traxxs::arc::ArcConditions& c_out, double time = std::nan("") );

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

/** 
 * \brief This wrapper adds an "out-of-bounds" feature to SM_TRAJ
 * Indeed, SM_TRAJ alone does not handle cases where initial conditions do not respect bounds.
 */
class SmTrajWrapper 
{
 public:
  SmTrajWrapper();

 public:
  int computeTraj( std::vector<SM_COND> IC, std::vector<SM_COND> FC, std::vector<SM_LIMITS> limits, SM_TRAJ::SM_TRAJ_MODE mode );
  int getMotionCond( double time, std::vector<SM_COND> & cond );
  double getDuration();
  
 protected:
  SM_TRAJ sm_traj_;  
  SM_TRAJ brake_sm_traj_;
  double brake_duration_ = 0.0;
  /** \brief the ICs from the parameters */
  SM_COND ic_;
  /** \brief the ICs used by the SM_TRAJ, i.e. after braking (if needed) */
  SM_COND ic_smtraj_;
};

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



struct ArcTrajGenSoftMotion::impl {
  impl();
  SmTraj_t sm_traj_;
  bool sm_traj_compute( ArcTrajGenSoftMotion* parent, SmTraj_t& traj, const std::vector<SM_COND>& c_i );
  bool sm_traj_conditions_at_time( ArcTrajGenSoftMotion* parent, SmTraj_t& traj, double time, std::vector<SM_COND>& c_out );
};


bool ArcTrajGenSoftMotion::do_init()
{
  impl_ = std::make_shared< ArcTrajGenSoftMotion::impl > ();
}

bool ArcTrajGenSoftMotion::do_compute()
{
  bool ret = true;
  
  SM_COND c_i;
  ret &= toSoftMotion( this->c_i_, c_i );
  std::vector< SM_COND > c_i_s;
  c_i_s.push_back( c_i );
  if ( ret )
    ret &= this->impl_->sm_traj_compute( this, this->impl_->sm_traj_, c_i_s );
  
  if ( !ret )
    this->duration_ = std::nan("");
  else
    this->duration_ = this->impl_->sm_traj_.getDuration();
  
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
  
  if ( !this->impl_->sm_traj_compute( this, traj, c_i_s ) )
    return false;
  std::vector< SM_COND > conds;
  if ( !this->impl_->sm_traj_conditions_at_time( this, traj, this->dt_, conds ) )
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
  if ( !this->impl_->sm_traj_conditions_at_time( this, this->impl_->sm_traj_, t, conds ) )
    return false;
  if ( !fromSoftMotion( conds[0], c_out, t ) )
    return false;
  return true;
}


double ArcTrajGenSoftMotion::do_get_duration()
{
  return this->duration_;
}

ArcTrajGenSoftMotion::impl::impl() 
{
  this->sm_traj_ = SmTraj_t();
}

bool ArcTrajGenSoftMotion::impl::sm_traj_compute( ArcTrajGenSoftMotion* parent, SmTraj_t& traj, const std::vector<SM_COND>& c_i )
{
  /** 
   * \todo check for nans in conditions and limits
   */
  bool ret = true;
  
  SM_COND c_f;
  SM_LIMITS c_max;
  
  ret &= toSoftMotion( parent->c_f_, c_f );
  ret &= toSoftMotion( parent->c_max_, c_max );
  if ( ret ) {
    std::vector<SM_COND> c_f_s{ c_f };
    std::vector<SM_LIMITS> c_max_s{ c_max };
    int cret = traj.computeTraj( c_i, c_f_s, c_max_s, SM_TRAJ::SM_INDEPENDANT );
    ret &= (cret == SM_STATUS::SM_OK);
  }
  
  return ret;
}

bool ArcTrajGenSoftMotion::impl::sm_traj_conditions_at_time( ArcTrajGenSoftMotion* parent, SmTraj_t& traj, double time, std::vector<SM_COND>& c_out )
{
  int cret = traj.getMotionCond( std::min( std::max( time, 0.0 ), parent->getDuration() ), c_out );
  return (cret == SM_STATUS::SM_OK);
}

