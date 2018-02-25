#ifndef TRAXXS_CRTP_H
#define TRAXXS_CRTP_H

#include <limits>

namespace traxxs {

template <typename Base, typename Derived>
class Cloneable : public Base
{
public:
    using Base::Base;
 
    virtual Base *clone() const {
        return new Derived(static_cast<Derived const &>(*this));
    }
};
  
} // namespace traxxs


#endif // TRAXXS_CRTP_H

