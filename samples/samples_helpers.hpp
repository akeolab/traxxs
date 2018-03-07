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


#include <chrono>
#include <limits>
#include "samples_helpers_io.hpp"

template < class T >
struct MinMaxAvg
{
  MinMaxAvg() { this->reset(); }
  void reset() {
    count_ = 0;
    min_ = std::numeric_limits< T >::infinity();
    max_ = -std::numeric_limits< T >::infinity();
    sum_ = 0;
  }
  
  void process( T value ) {
    min_ = std::min( min_, value );
    max_ = std::max( max_, value );
    sum_ += value;
    count_++;
  }
  
  const T& getMin() const { return this->min_; };
  const T& getMax() const { return this->max_; };
  T getAverage() const { if ( count_ <= 0 ) return std::nan(""); return this->sum_ * 1.0 / this->count_; };
  
 protected:
  long unsigned int count_;
  T min_, max_, sum_;
};



class StopWatch 
{
 public:
  StopWatch(){};
  virtual ~StopWatch(){};
 public:
  void start() {  t_begin_ = std::chrono::steady_clock::now(); }
  void stop() {    t_end_ = std::chrono::steady_clock::now(); }
  double get_ms() { return this->get<double, std::milli>(); }
  template < class T, class time_t >
  T get() { return std::chrono::duration <T, time_t> (t_end_-t_begin_).count(); }
  
 protected:
  std::chrono::time_point<std::chrono::steady_clock> t_begin_, t_end_;
  
};

void dummySystemTranslationKp( const traxxs::trajectory::TrajectoryState& desired_state, traxxs::trajectory::TrajectoryState& current_state, double Kp = 0.05 )
{
  traxxs::trajectory::TrajectoryState state_tmp;
  state_tmp = current_state;
  current_state = desired_state;
  current_state.x.segment(0,3) = state_tmp.x.segment(0,3);
  current_state.x.segment(0,3) += Kp * ( desired_state.x.segment(0,3) - state_tmp.x.segment(0,3) );
}
