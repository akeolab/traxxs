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
#include <traxxs/arc/arc.hpp>
#include <traxxs/crtp.hpp>
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
class ArcTrajGenSoftMotion : public traxxs::Cloneable< traxxs::arc::ArcTrajGen, ArcTrajGenSoftMotion >
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
