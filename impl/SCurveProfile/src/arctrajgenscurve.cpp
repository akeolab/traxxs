#include "arctrajgenscurve.hpp"

#include <stdexcept>

bool ArcTrajGenSCurve::do_init()
{
  this->scurve_profile_ = SCurveProfile();
}

bool ArcTrajGenSCurve::do_compute()
{
  /** 
   * \todo check for nans in conditions and limits
   */
  if ( std::isnan( this->dt_ ) )
    return false; // the s-curve algorithm needs a period
  this->scurve_profile_.set_period( this->dt_ );
  this->c_profile_.clear();
  this->duration_ = std::nan("");
  
  this->scurve_profile_.config(
    this->c_i_.s, this->c_i_.ds, this->c_i_.dds, 
    this->c_f_.s, this->c_f_.ds, this->c_f_.dds, 
    this->c_max_.ds, this->c_max_.dds, this->c_max_.j );
  
  this->scurve_profile_.compute_curves();
  
  this->duration_ = this->scurve_profile_.t_vect_[this->scurve_profile_.t_vect_.size()-1];
  
  ArcConditions c_tmp;
  this->c_profile_.resize( this->scurve_profile_.t_vect_.size() ) ;
  for ( int i=0 ; i<this->scurve_profile_.t_vect_.size() ; i++ ) {
    c_tmp.s   = this->scurve_profile_.s_vect_[i];
    c_tmp.ds  = this->scurve_profile_.v_vect_[i];
    c_tmp.dds = this->scurve_profile_.a_vect_[i];
    c_tmp.j   = this->scurve_profile_.j_vect_[i];
    c_tmp.t   = this->scurve_profile_.t_vect_[i];
    this->c_profile_[i] = c_tmp;
  }
  return true;
}

bool ArcTrajGenSCurve::do_compute_next_conditions( const ArcConditions& c_in, ArcConditions& c_out )
{
  /** 
   * \fixme this is vastly suboptimal 
   */
  SCurveProfile scurve;
  if ( std::isnan( this->dt_ ) )
    return false; // the s-curve algorithm needs a period
  scurve.set_period( this->dt_ );
  
  scurve.config(
    c_in.s, c_in.ds, c_in.dds, 
    this->c_f_.s, this->c_f_.ds, this->c_f_.dds, 
    this->c_max_.ds, this->c_max_.dds, this->c_max_.j );
  scurve.compute_curves();
  if ( scurve.t_vect_.size() < 1 )
    return false;
  c_out.s   = scurve.s_vect_[0+1];
  c_out.ds  = scurve.v_vect_[0+1];
  c_out.dds = scurve.a_vect_[0+1];
  c_out.j   = scurve.j_vect_[0+1];
  c_out.t   = scurve.t_vect_[0+1];
  return true;
}

bool ArcTrajGenSCurve::do_get_conditions_at_time(double t, ArcConditions& c_out)
{
  /**
   * \todo implement this. 
   */
  throw std::runtime_error("Not implemented");
  return false;
}


double ArcTrajGenSCurve::do_get_duration()
{
  return this->duration_;
}
