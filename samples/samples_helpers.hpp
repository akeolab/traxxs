
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
