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

#include <traxxs/arc/arc.hpp>
#include <traxxs/crtp.hpp>
#include <cmath>
#include <memory>

static const double kSoftMotionInfinityBound = 1.e12;

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

 private: 
   struct impl;
 protected:
   std::shared_ptr< impl > impl_;
  
 protected:
   
   double duration_ = std::nan("");
};
