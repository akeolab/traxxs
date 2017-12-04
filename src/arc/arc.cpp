#include "traxxs/arc/arc.hpp"

std::ostream& traxxs::arc::operator<<(std::ostream& os, const traxxs::arc::ArcConditions& obj) 
{
      os << obj.t << " :\t" << obj.s << " ,\t" << obj.ds << " ,\t" << obj.dds << " ,\t" << obj.j ;
      return os;
}
