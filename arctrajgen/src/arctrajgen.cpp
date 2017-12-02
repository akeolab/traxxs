#include "arctrajgen/arctrajgen.hpp"

std::ostream& operator<<(std::ostream& os, const ArcConditions& obj) 
{
      os << obj.t << " :\t" << obj.s << " ,\t" << obj.ds << " ,\t" << obj.dds << " ,\t" << obj.j ;
      return os;
}
