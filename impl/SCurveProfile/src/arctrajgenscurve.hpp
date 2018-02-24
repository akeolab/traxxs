#include <SCurveProfile.hpp>
#include <traxxs/arc/arc.hpp>
#include <traxxs/crtp.hpp>
#include <cmath>

/**
 * \brief an ArcTrajGen implementation using SCurve
 */
class ArcTrajGenSCurve : public traxxs::Cloneable< traxxs::arc::ArcTrajGen, ArcTrajGenSCurve > {
  
 protected: // the implementation
  virtual bool do_init() override;
  virtual bool do_compute() override;
  virtual bool do_compute_next_conditions( const traxxs::arc::ArcConditions& c_in, traxxs::arc::ArcConditions& c_out ) override;
  virtual bool do_get_conditions_at_time(double t, traxxs::arc::ArcConditions & c_out) override;
  virtual double do_get_duration() override;
  
 protected:
   SCurveProfile scurve_profile_;
   double duration_ = std::nan("");
};
