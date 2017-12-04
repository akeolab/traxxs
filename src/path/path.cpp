#include <traxxs/path/path.hpp>
#include <traxxs/constants.hpp>

traxxs::path::Path::Path( std::vector< std::shared_ptr < PathSegment > > segments )
    : segments_( segments )
{
  
}

bool traxxs::path::Path::init()
{
  // first update the segments arc bounds and start/end arc conditions
  PathBounds path_bounds;
  arc::ArcConditions arc_bounds;
  arc::ArcConditions arc_cond_start, arc_cond_end;
  for ( auto& seg : segments_ ) {
    path_bounds = seg->getPathBounds();
    arc_cond_start = seg->getStartArcConditions();
    arc_cond_end = seg->getEndArcConditions();
    
    bool is_line = true;
    is_line = ( seg->getDerivativeCwiseAbsMax(2).norm() <= constants::kZero ) && ( seg->getDerivativeCwiseAbsMax(3).norm() <= constants::kZero );
    // start by updating the arc bounds
    arc_bounds.ds = std::numeric_limits<double>::max();
    arc_bounds.dds = std::numeric_limits<double>::max();
    arc_bounds.j = std::numeric_limits<double>::max();
    
    if ( is_line ) {
      // if it is a line, we have f''(s) = 0, f'''(s) = 0
      // i.e. dq = f'.ds, ddq = f'.dds + 0, dddq = f'.ddds + 0 + 0
      for ( unsigned int dim = 0; dim < path_bounds.dx.size() ; ++dim ){
        if ( seg->getDerivativeCwiseAbsMax( 1 )[dim] < constants::kZero )
          continue;
        arc_bounds.ds = std::fmin( arc_bounds.ds, path_bounds.dx[dim] / seg->getDerivativeCwiseAbsMax( 1 )[dim] );
        arc_bounds.dds = std::fmin( arc_bounds.dds, path_bounds.ddx[dim] / seg->getDerivativeCwiseAbsMax( 1 )[dim] );
        arc_bounds.j = std::fmin( arc_bounds.j, path_bounds.j[dim] / seg->getDerivativeCwiseAbsMax( 1 )[dim] );
      }
    } else {
      /** 
       * in this case, we have f''(s) != 0, f'''(s) != 0
       *    dx    = f'.ds
       *    ddx   = f'.dds + f''.ds^2
       *    dddx  = f'.ddds + 3.f''.ds.dds + f'''.ds^3
       * And we constrain as follow, with the notation #g = max(|g|)
       *    { |ds|  < #ds   < ( #dx ) / #f'
       *    { |dds| < #dds  < ( #ddx - #f''.#ds^2 ) / #f'
       *    { |ddds|< #ddds < ( #dddx - 3.#f''.#ds.#dds - #f'''.#ds^3 ) / #f'
       * A possible approach: 
       */
      
      // first, handle the case dds = 0 && ddds = 0
      for ( unsigned int dim = 0; dim < path_bounds.dx.size() ; ++dim ){
        if ( seg->getDerivativeCwiseAbsMax( 1 )[dim] >= constants::kZero )
          arc_bounds.ds = std::fmin( arc_bounds.ds, path_bounds.dx[dim] / seg->getDerivativeCwiseAbsMax( 1 )[dim] );
        if ( seg->getDerivativeCwiseAbsMax( 2 )[dim] >= constants::kZero )
          arc_bounds.ds = std::fmin( arc_bounds.ds, std::sqrt( path_bounds.ddx[dim] / seg->getDerivativeCwiseAbsMax( 2 )[dim] ) );
        if ( seg->getDerivativeCwiseAbsMax( 3 )[dim] >= constants::kZero )
          arc_bounds.ds = std::fmin( arc_bounds.ds, std::sqrt( path_bounds.j[dim] / seg->getDerivativeCwiseAbsMax( 3 )[dim] ) );
      }
      
      double ds, dds, ddds;
      double fp, fpp, fppp;
      double dx, ddx, dddx;
      if ( seg->getDerivativeCwiseAbsMax(1).norm() > constants::kZero ) { 
        
        /** \todo should we reset arc_bounds at this point ? */
        
        for ( unsigned int dim = 0; dim < path_bounds.dx.size() ; ++dim ) {
          dx = path_bounds.dx[dim];
          ddx = path_bounds.ddx[dim];
          fp = seg->getDerivativeCwiseAbsMax( 1 )[dim];
          if ( fp < constants::kZero )  continue; // in this case, dds has no influence on acceleration, same for ds on velocity
          fpp = seg->getDerivativeCwiseAbsMax( 2 )[dim];
          if ( fpp < constants::kZero ) {
            // simple case, like a line (at least for vel and acc, not for jerk)
            ds = dx / fp;
            dds = ddx / fp;
          } else {
            // assume ds = dds, and solve the acc constraint
            double discr = fp*fp + 4 * ddx * fpp; // this is always >= 0
            double dds1, dds2;
            dds1 = ( -fp - std::sqrt( discr ) ) / ( 2.0 * fpp );
            dds2 = ( -fp + std::sqrt( discr ) ) / ( 2.0 * fpp );
            
            dds = std::fmax( dds1, dds2 ); // the min will be the negative value, so we take the max (hopefully positive)
            /** 
             * \note this is arbitrary, in order to maximize equally ds and dds
             * \fixme find something better
             */
            ds = dds;
            
            if ( fp * ds >= dx || dds <= constants::kZero ) { // this solution does not work (dds <= 0 should not happen)
              // the solution is to take the intersection of the acc and vel constraints
              ds = dx / fp;
              dds = ( ddx - fpp * ds*ds ) / fp;
            }
          } // end if fpp not null
          
          // take the min over all dimensions
          arc_bounds.ds = std::fmin( arc_bounds.ds, ds );
          arc_bounds.dds = std::fmin( arc_bounds.dds, dds );
        } // end for loop on dim
        
        // at this point, we have compatible velocity and acceleration constraints
        // we need to go further and comply with the jerk constraint
        for ( unsigned int dim = 0; dim < path_bounds.dx.size() ; ++dim ) {
          fp = seg->getDerivativeCwiseAbsMax( 1 )[dim];
          if ( fp < constants::kZero )  continue; // in this case, ddds has no influence on jerk
          fpp = seg->getDerivativeCwiseAbsMax( 2 )[dim];
          fppp = seg->getDerivativeCwiseAbsMax( 3 )[dim];
          ds = arc_bounds.ds;
          dds = arc_bounds.dds;
          dddx = path_bounds.j[dim];
          
          // it can happen that the ds and dds values chosen above are too large and yield ddds <= 0.
          // we need to pick other (compatible) values that will yield ddds > 0
          /** \hack this is relatively badly handled */
          if ( ds > constants::kZero ) {
            int N = 10; // this is arbitrary
            double ds_decr = ds * 1.0f / N;
            double k = dds / ds;
            double ds_0 = ds;
            for ( unsigned int iter = 0; iter < N+1; ++iter ) {
              ds = ds_0 - iter * ds_decr; // we decrease ds
              dds = k * ds; // we remain on the same line in the (ds, dds) plane to keep vel/acc constraints compliance
              ddds = (dddx - 3.0*fpp * ds*dds - fppp * ds*ds*ds) / fp;
              if ( ddds > 0 ) // this is valid
                break;
            } 
          } else {
            /** \fixme throw a warning here ! ds_max = 0 -> no motion ! */
          }
          // we store the result, keep the minimum to be compliant in all directions
          arc_bounds.ds = std::fmin( arc_bounds.ds, ds );
          arc_bounds.dds = std::fmin( arc_bounds.dds, dds );
          arc_bounds.j = std::fmin( arc_bounds.j, ddds );
        } // end for loop on dim 
        
      } // end if tangent null
      
    }
    
    // then the start/end arc conditions. If defined by the user conform to the bounds, otherwise set to max for fastest travel
    // start conditions
    if ( std::isnan( arc_cond_start.ds ) )  
      arc_cond_start.ds = arc_bounds.ds;
    else                                
      arc_cond_start.ds = ( (arc_cond_start.ds > 0) ? +1 : -1 ) * std::fmin( std::fabs(arc_cond_start.ds), arc_bounds.ds );
    if ( std::isnan( arc_cond_start.dds ) ) 
      arc_cond_start.dds = arc_bounds.dds;
    else                                
      arc_cond_start.dds = ( (arc_cond_start.dds > 0) ? +1 : -1 ) * std::fmin( std::fabs(arc_cond_start.dds), arc_bounds.dds );
    if ( std::isnan( arc_cond_start.j ) )   
      arc_cond_start.j = arc_bounds.j;
    else                                
      arc_cond_start.j = ( (arc_cond_start.j > 0) ? +1 : -1 ) * std::fmin( std::fabs(arc_cond_start.j), arc_bounds.j );
    // end conditions
    if ( std::isnan( arc_cond_end.ds ) )    
      arc_cond_end.ds = arc_bounds.ds;
    else                                
      arc_cond_end.ds = ( (arc_cond_end.ds > 0) ? +1 : -1 ) * std::fmin( std::fabs(arc_cond_end.ds), arc_bounds.ds );
    if ( std::isnan( arc_cond_end.dds ) )   
      arc_cond_end.dds = arc_bounds.dds;
    else                                
      arc_cond_end.dds = ( (arc_cond_end.dds > 0) ? +1 : -1 ) * std::fmin( std::fabs(arc_cond_end.dds), arc_bounds.dds );
    if ( std::isnan( arc_cond_end.j ) )     
      arc_cond_end.j = arc_bounds.j;
    else                                
      arc_cond_end.j = ( (arc_cond_end.j > 0) ? +1 : -1 ) * std::fmin( std::fabs(arc_cond_end.j), arc_bounds.j );
    
    
    seg->setArcBounds( arc_bounds );
    seg->setStartArcConditions( arc_cond_start );
    seg->setEndArcConditions( arc_cond_end );
  } // end for loop on segments
  
  /** 
   * Then adapt start/end arc conditions for velocity continuity between segments
   * dx velocity continuity can be ensured by:
   *    { ds(-) = ds(+) = 0           if cross( f'(-), f'(+) ) != 0 (i.e. tangents are not aligned)
   *    { f'(-).ds(-) = f'(+).ds(+)   otherwise
   * We give priority to dx continituity.
   * If f'(-) = k.f'(+),      this will yield k.ds(-) = ds(+)
   * If f'(-) × k.f'(+) != 0, this will yield ds(-) = ds(+) == 0
   * Then, to ensure ddx acceleration continuity under these conditions, you should use segments ensuring both
   *    { f'(-)  = f'(+)
   *    { f''(-) = f''(+)
   * And for jerk
   *    { f'''(-) = f'''(+)
   * 
   * This simplified approach consists in 
   * - ds will be deduced from dx continuity ( dx = f'.ds )
   * - dds will be deduced from ddx continuity ( f'.dds term in ddx)
   * - ddds will be deduced from dddx continuity ( f'.ddds term in dddx)
   */
  std::shared_ptr < PathSegment > cur, prev;
  arc::ArcConditions arc_cond_end_prev;
  Eigen::VectorXd fp, fpp, fppp;
  Eigen::VectorXd fp_prev, fpp_prev, fppp_prev;
  for ( unsigned int iseg = 0; iseg < this->segments_.size(); ++iseg ) {
    cur = this->segments_[iseg];
    prev = nullptr;
    if ( iseg > 0 ) 
      prev = this->segments_[iseg-1];
    // do something on the current segment 
    arc_cond_start = cur->getStartArcConditions(); // ds, dds, ddds (+)
    if ( prev != nullptr ) {
      arc_cond_end_prev = prev->getEndArcConditions(); //  // ds, dds, ddds (-)
      fp_prev   = prev->getDerivative( 1, prev->getLength() ); // f'(-)
      fpp_prev  = prev->getDerivative( 2, prev->getLength() ); // f''(-)
      fppp_prev = prev->getDerivative( 3, prev->getLength() ); // f'''(-)
      fp    = cur->getDerivative( 1, 0.0 ); // f'(+)
      fpp   = cur->getDerivative( 2, 0.0 ); // f''(+)
      fppp  = cur->getDerivative( 3, 0.0 ); // f'''(+)
      // dx continuity
      if ( std::fabs( arc_cond_start.ds ) < constants::kZero || std::fabs( arc_cond_end_prev.ds ) < constants::kZero  // if any of ds(-), ds(+) is null, 
        || 
        ( normalizedOrZero(fp) - normalizedOrZero(fp_prev) ).norm() > constants::kZero // f' are not aligned
      ) { 
        // only solution is dx = 0
        arc_cond_start.ds = 0.0;
        arc_cond_end_prev.ds = 0.0;
        // for ddx and dddx, easier cf. f'.dds and f'.ddds terms
        arc_cond_start.dds = 0.0;
        arc_cond_end_prev.dds = 0.0;
        arc_cond_start.j = 0.0;
        arc_cond_end_prev.j = 0.0;
      } else {
          double k = std::nan("");
          for ( unsigned int dim = 0; dim < fp.size() ; ++dim ) {
            if ( std::fabs( fp[dim] ) < constants::kZero )
              continue; // wait for non-null component
            k = fp_prev[dim] / fp[dim]; // k s.t. f'(-) = k.f'(+)
            break; // one is enough, since valid in all directions
          }
          if ( std::isnan( k ) || std::fabs( k ) < constants::kZero ) { // one of the tangents is null
            /** \todo check this */
            // fall back to dx = 0 continuity condition
            arc_cond_start.ds = 0.0;
            arc_cond_end_prev.ds = 0.0;
            // for ddx and dddx, easier cf. f'.dds and f'.ddds terms
            arc_cond_start.dds = 0.0;
            arc_cond_end_prev.dds = 0.0;
            arc_cond_start.j = 0.0;
            arc_cond_end_prev.j = 0.0;
          } else {
            // we need k.ds(-) = ds(+), i.e.  k * arc_cond_end_prev.ds = arc_cond_start.ds
            // with k.ds(-) <= ds(+)_0
            // and ds(-) <= ds(-)_0
            if ( std::fabs( k ) > std::fabs( arc_cond_start.ds / arc_cond_end_prev.ds ) ) {
              // arc_cond_start.ds = arc_cond_start.ds;  // as-is
              arc_cond_end_prev.ds = arc_cond_start.ds / k;
            } else {
              arc_cond_start.ds = k * arc_cond_end_prev.ds;
              // arc_cond_end_prev.ds = arc_cond_end_prev.ds; // as-is
            }
            // same for dds (cf. f'.dds term in ddx)
            if ( std::fabs( k ) > std::fabs( arc_cond_start.dds / arc_cond_end_prev.dds ) ) {
              arc_cond_end_prev.dds = arc_cond_start.dds / k;
            } else {
              arc_cond_start.dds = k * arc_cond_end_prev.dds;
            }
            // same for ddds (cf. f'.ddds term in dddx)
            if ( std::fabs( k ) > std::fabs( arc_cond_start.j / arc_cond_end_prev.j ) ) {
              arc_cond_end_prev.j = arc_cond_start.j / k;
            } else {
              arc_cond_start.j = k * arc_cond_end_prev.j;
            }
          }
      }
      
      
      
      prev->setEndArcConditions( arc_cond_end_prev );
    }
    cur->setStartArcConditions( arc_cond_start );
  }
}

