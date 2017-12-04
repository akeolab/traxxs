#include <softMotion.h>
#include <traxxs/arc/arc.hpp>
#include <cmath>

bool toSoftMotion( const ArcConditions& c_in, SM_COND& c_out );
bool toSoftMotion( const ArcConditions& c_in, SM_LIMITS& c_out );

bool fromSoftMotion( const SM_COND& c_in, ArcConditions& c_out, double time = std::nan("") );

/**
 * \brief an ArcTrajGen implementation using softMotion
 */
class ArcTrajGenSoftMotion : public ArcTrajGen 
{
 protected: // the implementation
  virtual bool do_init() override;
  virtual bool do_compute() override;
  virtual bool do_compute_next_conditions( const ArcConditions& c_in, ArcConditions& c_out ) override;
  virtual bool do_get_conditions_at_time(double t, ArcConditions & c_out) override;
  virtual double do_get_duration() override;
  
protected: 
  bool sm_traj_compute( SM_TRAJ& traj, const std::vector<SM_COND>& c_i );
  bool sm_traj_conditions_at_time( SM_TRAJ& traj, double time, std::vector<SM_COND>& c_out );
  
 protected:
   SM_TRAJ sm_traj_;
   double duration_ = std::nan("");
};
