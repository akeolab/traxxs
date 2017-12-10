
#include <sstream>

template < class T >
std::string toCSV(const T& obj) { 
  std::stringstream ss;
  for ( unsigned int i = 0; i < obj.size() ; ++i ) {
    ss << obj[i];
    if ( i != obj.size() -1 ) ss << ";";
  }
  return ss.str();
}
