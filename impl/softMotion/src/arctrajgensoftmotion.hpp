#include <softMotion.h>
#include <traxxs/arc/arc.hpp>
#include <cmath>


static const double kSoftMotionInfinityBound = 1.e12;

bool toSoftMotion( const traxxs::arc::ArcConditions& c_in, SM_COND& c_out );
bool toSoftMotion( const traxxs::arc::ArcConditions& c_in, SM_LIMITS& c_out );

bool fromSoftMotion( const SM_COND& c_in, traxxs::arc::ArcConditions& c_out, double time = std::nan("") );
bool fromSoftMotion( const SM_LIMITS& limits_in, traxxs::arc::ArcConditions& c_out, double time = std::nan("") );

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

// using SmTraj_t = SM_TRAJ;
using SmTraj_t = SmTrajWrapper;

/**
 * \brief an ArcTrajGen implementation using softMotion
 */
class ArcTrajGenSoftMotion : public traxxs::arc::ArcTrajGen 
{
 protected: // the implementation
  virtual bool do_init() override;
  virtual bool do_compute() override;
  virtual bool do_compute_next_conditions( const traxxs::arc::ArcConditions& c_in, traxxs::arc::ArcConditions& c_out ) override;
  virtual bool do_get_conditions_at_time(double t, traxxs::arc::ArcConditions & c_out) override;
  virtual double do_get_duration() override;
  
protected: 
  bool sm_traj_compute( SmTraj_t& traj, const std::vector<SM_COND>& c_i );
  bool sm_traj_conditions_at_time( SmTraj_t& traj, double time, std::vector<SM_COND>& c_out );
  
 protected:
   SmTraj_t sm_traj_;
   double duration_ = std::nan("");
};
