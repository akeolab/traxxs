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

#ifndef TRAXXS_ARCTRAJGEN_H
#define TRAXXS_ARCTRAJGEN_H

#include <vector>
#include <cmath>
#include <iostream>

namespace traxxs {
namespace arc {
/** 
 * \brief 1-D trajectory conditions 
 * Members are s, ds, dds, j, t defaulting to nan
 */
struct ArcConditions 
{
  ArcConditions( double s = std::nan(""), double ds = std::nan(""), double dds = std::nan(""), double j = std::nan(""), double t = std::nan("") )
      : s( s ), ds( ds ), dds( dds ), j( j ), t( t ) {}
  double s = std::nan("");
  double ds = std::nan("");
  double dds = std::nan("");
  /** \brief optional jerk */
  double j = std::nan("");
  /** \brief optional time, defaults to nan */
  double t  = std::nan("");
};

std::ostream& operator<<(std::ostream& os, const ArcConditions& obj); 

/** 
 * \brief An abstract trajectory generator interface over arc length 
 */
class ArcTrajGen
{
 public:
  virtual ArcTrajGen* clone() const = 0;
  virtual ~ArcTrajGen() = default;
  
 public: // the non-virtual public interface
  // main
  /** \brief performs some initial setup */
  virtual bool init() final { 
    bool ret = do_init(); 
    is_computed_ = false;
    return ret;
  }
  /** \brief computes the complete profile */
  virtual bool compute() final { 
    is_computed_ = do_compute(); 
    return is_computed_;
  }
  /** 
   * \brief computes the next conditions from current conditions to the final conditions 
   * \param[in] c_in the current conditions
   * \param[out] c_out the next conditions
   */
  virtual bool computeNextConditions( const ArcConditions& c_in, ArcConditions& c_out ) final { return do_compute_next_conditions( c_in, c_out ); }
  /** 
   * \brief get the conditions at a given time from a previously computed profile
   * \param[in] t the time in the trajectory timeframe, i.e. t \in [0, getDuration()] 
   * \param[out] c_out the conditions
   */
  virtual bool getConditionsAtTime( double t, ArcConditions& c_out ) final { return do_get_conditions_at_time( t, c_out ); }
  /** \brief get the complete duration of a previously computed profile. Returns std::nan() on uncomputed, or error. */
  virtual double getDuration() final { return do_get_duration(); }
  /** \brief whether or not it is computed. */
  virtual bool isComputed() final { return this->is_computed_; }
  
  // setters
  virtual void setDt( double dt ) final                               { this->dt_ = dt; }
  virtual void setInitialConditions( const ArcConditions& c_i ) final { this->c_i_ = c_i; this->is_computed_ = false; }
  virtual void setFinalConditions( const ArcConditions& c_f ) final   { this->c_f_ = c_f; this->is_computed_ = false; }
  virtual void setMaxConditions( const ArcConditions& c_max ) final   { this->c_max_ = c_max; this->is_computed_ = false; }
  // getters
  virtual double        getDt() final                                 { return this->dt_; }
  virtual ArcConditions getInitialConditions() final                  { return this->c_i_; }
  virtual ArcConditions getFinalConditions() final                    { return this->c_f_; }
  virtual ArcConditions getMaxConditions() final                      { return this->c_max_; }
  virtual std::vector< ArcConditions > getConditionsProfile() final   { return this->c_profile_; }
  
 protected: // the (pure virtual) implementation
  /** \brief implementation of init() */
  virtual bool do_init() = 0;
  /** \brief implementation of compute() */
  virtual bool do_compute() = 0;
  /** \brief implementation of computeNextConditions() */
  virtual bool do_compute_next_conditions( const ArcConditions& c_in, ArcConditions& c_out ) = 0;
  /** \brief implementation of getConditionsAtTime() */
  virtual bool do_get_conditions_at_time( double t, ArcConditions& c_out ) = 0;
  /** \brief implementation of getDuration(). Should return std::nan() on uncomputed, or error. */
  virtual double do_get_duration() = 0;
  
protected:
  /** \brief the initial conditions */
  ArcConditions c_i_;
  /** \brief the final conditions */
  ArcConditions c_f_;
  /** \brief the max conditions */
  ArcConditions c_max_;
  /** \brief the complete profile of conditions, from initial (included) to final (included if reachable) */
  std::vector< ArcConditions > c_profile_;
  /** \brief the time-discretization step, if any */
  double dt_ = std::nan("");
  /** \brief whether or not it is computed */
  bool is_computed_ = false;
};

} // namespace arc
} // namespace traxxs

#endif // TRAXXS_ARCTRAJGEN_H
