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
  
  traxxs::arc::ArcConditions c_tmp;
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

bool ArcTrajGenSCurve::do_compute_next_conditions( const traxxs::arc::ArcConditions& c_in, traxxs::arc::ArcConditions& c_out )
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

bool ArcTrajGenSCurve::do_get_conditions_at_time(double t, traxxs::arc::ArcConditions& c_out)
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
