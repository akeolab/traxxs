
#include <sstream>
#include <chrono>

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
