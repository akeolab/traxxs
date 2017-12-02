#ifndef TRAJGEN_PATH_PATH_H
#define TRAJGEN_PATH_PATH_H

#include "path/segment.hpp"

/** 
 * Will ensure:
 * - Arc bounds on each segments are compatible with Path bounds on each segments
 * - Arc start/end conditions ensure dx continuity at all costs, i.e. through dx=0 if needed.
 * - Arc start/end conditions ensure ddx and dddx continuity if possible
 */
class Path
{
 public:
  Path( std::vector< std::shared_ptr < PathSegment > > segments );
 protected: 
  std::vector< std::shared_ptr < PathSegment > > segments_;
};

#endif // TRAJGEN_PATH_PATH_H

