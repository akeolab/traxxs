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

#ifndef TRAXXS_TRACKER_TRACKER_TIME_H
#define TRAXXS_TRACKER_TRACKER_TIME_H

#include <traxxs/tracker/core.hpp>

namespace traxxs {
namespace tracker {

/** 
 * \brief A Time-pursuit trajectory tracker
 * Simply, the tracker keeps track of the virtual time.
 * On every call to TrackerTimePursuit::next(), the virtual time will be incremented by the input dt until the end of the trajectory is met
 */
class TrackerTimePursuit : public TrackerBase
{
 public:
  TrackerTimePursuit() : TrackerBase() {};
  
 protected:
  virtual TrackerStatus do_next( double dt, const trajectory::TrajectoryState& current_state, trajectory::TrajectoryState& new_state_out ) override {
    bool is_beyond;
    if ( this->trajectory_ == nullptr )
      return TrackerStatus::NotInitialized;
    bool ret = this->trajectory_->getState( this->current_virtual_t_ + dt, new_state_out, nullptr, &is_beyond );
    if ( !ret )
      return TrackerStatus::Error;
    // if beyond
    if ( is_beyond )
      return TrackerStatus::Finished;
    // keep track of the increment
    this->current_virtual_t_ += dt;
    return TrackerStatus::Incremented;
  }
};

} // namespace tracker
} // namespace traxxs

#endif //TRAXXS_TRACKER_TRACKER_TIME_H
