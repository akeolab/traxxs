
#include <sstream>
#include <chrono>
#include <limits>

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


template < class T >
std::string toCSV(const T& obj) { 
  std::stringstream ss;
  for ( unsigned int i = 0; i < obj.size() ; ++i ) {
    ss << obj[i];
    if ( i != obj.size() -1 ) ss << ";";
  }
  return ss.str();
}

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
