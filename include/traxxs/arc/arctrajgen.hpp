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
  virtual ~ArcTrajGen() = default;
  
 public: // the non-virtual public interface
  // main
  /** \brief performs some initial setup */
  virtual bool init() final { return do_init(); }
  /** \brief computes the complete profile */
  virtual bool compute() final { return do_compute(); }
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
  
  // setters
  virtual void setDt( double dt ) final                               { this->dt_ = dt; }
  virtual void setInitialConditions( const ArcConditions& c_i ) final { this->c_i_ = c_i; }
  virtual void setFinalConditions( const ArcConditions& c_f ) final   { this->c_f_ = c_f; }
  virtual void setMaxConditions( const ArcConditions& c_max ) final   { this->c_max_ = c_max; }
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
};

} // namespace arc
} // namespace traxxs

#endif // TRAXXS_ARCTRAJGEN_H
