#include "path/segment.hpp"

Eigen::VectorXd normalizedOrZero( const Eigen::VectorXd& vec, double norm_tolerance /*= kZero*/ )
{
  double norm = vec.norm();
  if ( norm < norm_tolerance )
    return Eigen::VectorXd::Zero( vec.size() );
  return vec.normalized() ; // let Eigen handle the rest (NaNs, etc.)
}

Eigen::VectorXd stack( const Eigen::VectorXd& vec_top, const Eigen::VectorXd& vec_bottom )
{
  Eigen::VectorXd vstack = Eigen::VectorXd( vec_top.rows() + vec_bottom.rows() );
  vstack.segment( 0, vec_top.rows() )             = vec_top;
  vstack.segment( vec_top.rows(), vec_bottom.rows() ) = vec_bottom;
  return vstack;
}

StackedSegments::StackedSegments(const std::vector<std::shared_ptr<PathSegment> >& segments)
{
  this->init_stack( segments );
}

StackedSegments::StackedSegments( std::shared_ptr< PathSegment >& segment_top, std::shared_ptr< PathSegment >& segment_bottom )
    : StackedSegments( std::vector<std::shared_ptr<PathSegment> >{segment_top, segment_bottom} )
{}

bool StackedSegments::init_stack( const std::vector<std::shared_ptr<PathSegment> >& segments )
{
  this->length_ = 1.0; // unit arc variable used for StackedSegments
  this->segments_ = segments;
  for ( const auto& seg : segments_ ) {
    this->path_bounds_.dx = stack( this->path_bounds_.dx, seg->getPathBounds().dx );
    this->path_bounds_.ddx = stack( this->path_bounds_.ddx, seg->getPathBounds().ddx );
    this->path_bounds_.j = stack( this->path_bounds_.j, seg->getPathBounds().j );
  }
  this->cond_start_.position = this->getConfiguration( 0.0 ); // use the newly implemented interface
  this->cond_end_.position = this->getConfiguration( 0.0 );
  
  /** 
   * \todo now adapt conditions.arcConditions 
   */
  
  return true;
}

Eigen::VectorXd StackedSegments::do_get_derivative(unsigned int order, double s) const
{
  Eigen::VectorXd der;
  double L, mult;
  for ( const auto& seg : segments_ ) {
    L = seg->getLength();
    mult = std::pow( L, mult );
    der = stack( der, mult * seg->getDerivative( order, s * L ) );
  }
  return der;
}

Eigen::VectorXd StackedSegments::do_get_derivative_cwise_abs_max(unsigned int order) const
{
  Eigen::VectorXd der;
  double L, mult;
  for ( const auto& seg : segments_ ) {
    L = seg->getLength();
    mult = std::pow( L, mult );
    der = stack( der, mult * seg->getDerivativeCwiseAbsMax( order ) );
  }
  return der;
}


LinearSegment::LinearSegment( const PathConditions& start, const PathConditions& end, const PathBounds& bounds )
    : PathSegment( start, end, bounds )
{
  this->length_ = (end.position - start.position).norm();
}

Eigen::VectorXd LinearSegment::do_get_derivative( unsigned int order, double s ) const
{
  switch ( order ) {
    case 0: {
      return this->get_derivative_0( s );
      break;
    }
    case 1: {
      return this->get_derivative_1( s );
      break;
    }
    default: {
      return Eigen::VectorXd::Zero( this->cond_start_.position.size() );
      break;
    }
  }
}

Eigen::VectorXd LinearSegment::do_get_derivative_cwise_abs_max( unsigned int order ) const
{
  switch ( order ) {
    case 1: {
      return this->get_derivative_1( 0.0 ).cwiseAbs(); // is constant over s
      break;
    }
    default: {
      return Eigen::VectorXd::Zero( this->cond_start_.position.size() );
      break;
    }
  }
}

Eigen::VectorXd LinearSegment::get_derivative_0( double s ) const
{
  if ( this->getLength() < kZero )
    return this->cond_start_.position;
  
  s /= this->getLength();
  return ( 1.0 - s ) * this->cond_start_.position + s * this->cond_end_.position;
}

Eigen::VectorXd LinearSegment::get_derivative_1( double s ) const
{
  (void) s; // for linear segments, the tangent (1st) is constant, so arc length is unused
  /** \fixme what to do when length zero ? */
  if ( this->getLength() < kZero )
    return Eigen::VectorXd::Ones( this->cond_start_.position.size() );
  
  return ( this->cond_end_.position - this->cond_start_.position ) / this->getLength();
}


SmoothStep7::SmoothStep7( const PathConditions& start, const PathConditions& end, const PathBounds& bounds )
    : PathSegment( start, end, bounds )
{
  this->length_ = 1.0;
}

Eigen::VectorXd SmoothStep7::do_get_derivative( unsigned int order, double s ) const
{
  switch ( order ) {
    case 0: {
      return this->get_derivative_0( s );
      break;
    }
    case 1: {
      return this->get_derivative_1( s );
      break;
    }
    case 2: {
      return this->get_derivative_2( s );
      break;
    }
    case 3: {
      return this->get_derivative_3( s );
      break;
    }
    default: {
      throw std::runtime_error("Derivatives above 3 are not implemented for SmoothStep7.");
      break;
    }
  }
}

Eigen::VectorXd SmoothStep7::do_get_derivative_cwise_abs_max( unsigned int order ) const
{
  switch ( order ) {
    case 1: {
      return this->getDerivative( order, 1.0/2.0 ).cwiseAbs();
      break;
    }
    case 2: {
      return this->getDerivative( order, (5.0 + std::sqrt(5.0))/10.0 ).cwiseAbs();
      break;
    }
    case 3: {
      return this->getDerivative( order, 1.0/2.0 ).cwiseAbs();
      break;
    }
    default: {
      throw std::runtime_error("Derivatives above 3 are not implemented for SmoothStep7.");
      break;
    }
  }
}

Eigen::VectorXd SmoothStep7::get_derivative_0( double s ) const
{
  s = std::min( std::max( s, 0.0 ), 1.0 ); // clamp
  double s_4 = s * s * s * s;
  double f = s_4 * ( -20*s*s*s + 70*s*s - 84*s + 35 );
  return (1.0 - f) * this->cond_start_.position + f * this->cond_end_.position;
}

Eigen::VectorXd SmoothStep7::get_derivative_1( double s ) const
{
  s = std::min( std::max( s, 0.0 ), 1.0 ); // clamp
  double s_3 = s * s * s;
  double fp = s_3 * ( -140*s*s*s + 420*s*s - 420*s + 140 );
  return -fp * this->cond_start_.position + fp * this->cond_end_.position;
}

Eigen::VectorXd SmoothStep7::get_derivative_2( double s ) const
{
  s = std::min( std::max( s, 0.0 ), 1.0 ); // clamp
  double s_2 = s * s;
  double fpp = s_2 * ( -840*s*s*s + 2100*s*s - 1680*s + 420 );
  return -fpp * this->cond_start_.position + fpp * this->cond_end_.position;
}

Eigen::VectorXd SmoothStep7::get_derivative_3( double s ) const
{
  s = std::min( std::max( s, 0.0 ), 1.0 ); // clamp
  double fppp = s * ( -4200*s*s*s + 8400*s*s - 5040*s + 840 );
  return -fppp * this->cond_start_.position + fppp * this->cond_end_.position;
}


CircularBlend::CircularBlend( const PathConditions& start, const PathConditions& end, const PathBounds& bounds, 
                              const Eigen::VectorXd& waypoint, 
                              double maxDeviation )
    : BlendSegment< double >( start, end, bounds, waypoint, maxDeviation )
{
  const Eigen::VectorXd startDirection = normalizedOrZero(waypoint - start.position);
  const Eigen::VectorXd endDirection = normalizedOrZero(end.position - waypoint);
  
  if ( ( waypoint - start.position ).norm() < kZero || ( end.position - waypoint ).norm() < kZero 
        ||
        (startDirection - endDirection).norm() < kZero // this is a i.Pi corner, no blend possible
  ) {
    length_ = 0.0;
    radius_ = 1.0;
    center_ = waypoint;
    x_ = Eigen::VectorXd::Zero(center_.size());
    y_ = x_;
  } else {
    double distance = std::min((start.position - waypoint).norm(), (end.position - waypoint).norm());
    double cosangle = startDirection.dot(endDirection);
    cosangle = std::min( 1.0, std::max( -1.0, cosangle ) );
    const double angle = std::acos(cosangle);
    if( std::isnan( angle ) )
      throw std::runtime_error( "Path angle is invalid" );

    distance = std::min(distance, maxDeviation * sin(0.5 * angle) / (1.0 - std::cos(0.5 * angle)));  // enforce max deviation

    radius_ = distance / tan(0.5 * angle);
    length_ = angle * radius_;

    center_ = waypoint + normalizedOrZero(endDirection - startDirection) * radius_ / std::cos(0.5 * angle);
    x_ = normalizedOrZero(waypoint - distance * startDirection - center_);
    y_ = startDirection;
  }
  
  // store the effective start/end conditions 
  this->cond_start_.position = this->getConfiguration(0.0);
  this->cond_end_.position = this->getConfiguration(0.0);
}


Eigen::VectorXd CircularBlend::do_get_derivative( unsigned int order, double s ) const
{
  switch ( order ) {
    case 0: {
      return this->get_derivative_0( s );
      break;
    }
    case 1: {
      return this->get_derivative_1( s );
      break;
    }
    case 2: {
      return this->get_derivative_2( s );
      break;
    }
    case 3: {
      return this->get_derivative_3( s );
      break;
    }
    default: {
      throw std::runtime_error("Derivatives above 3 are not implemented for CircularBlend.");
      break;
    }
  }
}

Eigen::VectorXd CircularBlend::do_get_derivative_cwise_abs_max( unsigned int order ) const
{
  Eigen::VectorXd tmp = Eigen::VectorXd::Zero(x_.size());
  switch ( order ) {
    case 1: {
      for ( unsigned int i=0; i<x_.size(); ++i )
        tmp[i] = sqrt( x_[i]*x_[i] + y_[i]*y_[i] );
      return tmp;
      break;
    }
    case 2: {
      for ( unsigned int i=0; i<x_.size(); ++i )
        tmp[i] = 1.0/radius_ * sqrt( x_[i]*x_[i] + y_[i]*y_[i] );
      return tmp;
      break;
    }
    case 3: {
      for ( unsigned int i=0; i<x_.size(); ++i )
        tmp[i] = 1.0/(radius_*radius_) * sqrt( x_[i]*x_[i] + y_[i]*y_[i] );
      return tmp;
      break;
    }
    default: {
      throw std::runtime_error("Derivatives above 3 are not implemented for CircularBlend.");
      break;
    }
  }
}

Eigen::VectorXd CircularBlend::get_derivative_0(double s) const
{
  const double angle = s / radius_;
  return center_ + radius_ * (x_ * std::cos(angle) + y_ * std::sin(angle));
}

Eigen::VectorXd CircularBlend::get_derivative_1(double s) const
{
  const double angle = s / radius_;
  return - x_ * std::sin(angle) + y_ * std::cos(angle);
}

Eigen::VectorXd CircularBlend::get_derivative_2(double s) const
{
  const double angle = s / radius_;
  return - 1.0 / radius_ * (x_ * std::cos(angle) + y_ * std::sin(angle));
}

Eigen::VectorXd CircularBlend::get_derivative_3(double s) const
{
  const double angle = s / radius_;
  return - 1.0 / (radius_*radius_) * (x_ * -std::sin(angle) + y_ * std::cos(angle));
}


CartesianSegmentBase::CartesianSegmentBase( std::shared_ptr< PathSegment > segment_position, std::shared_ptr< PathSegment > segment_orientation, 
                        const CartesianPoint& start, const CartesianPoint& end ) 
{
  this->init_cartesianbase( segment_pos_, segment_or_, cart_start_, cart_end_ );
}

bool CartesianSegmentBase::init_cartesianbase( std::shared_ptr<PathSegment> segment_position, std::shared_ptr<PathSegment> segment_orientation, 
                                               const CartesianPoint& start, const CartesianPoint& end )
{
  this->init_stack( std::vector< std::shared_ptr< PathSegment > >{ segment_pos_, segment_or_ } );
  cart_start_ = start, cart_end_ = end; 
  segment_pos_ = segment_position, segment_or_ = segment_orientation;
  
  /** \fixme should not throw exception ( used in constructors ! ) */
  // check dimensions
  if ( segment_or_->getConfiguration(segment_or_->getLength()).size() != 1 )
    throw std::runtime_error( "Orientation segment segment_orientation should be of dimension 1 in CartesianSegmentBase." );
  
  // check that we have a match for position solely (orientation representation is different)
  if ( 
      ( segment_pos_->getConfiguration(0.0) - start.p ).norm() > kZero 
      ||
      ( segment_pos_->getConfiguration(segment_pos_->getLength()) - end.p ).norm() > kZero )
    throw std::runtime_error( "Inconsistent position segment and start/end points in CartesianSegmentBase." );
  
  or_trans_ = Eigen::AngleAxisd( end.q * start.q.inverse() );
  
  if (
      std::fabs( 0.0 - segment_or_->getConfiguration(0.0)[0] ) > kZero // not starting at zero
      ||
      std::fabs( or_trans_.angle() - segment_or_->getConfiguration(segment_or_->getLength())[0] )  > kZero ) // not ending at angle
    throw std::runtime_error( "Inconsistent orientation segment and start/end points in CartesianSegmentBase." );
  return true;
}


Eigen::VectorXd CartesianSegmentBase::getPosition( const ArcConditions& arc_conditions ) const {
  Eigen::VectorXd v4d = this->getConfiguration( arc_conditions.s );
  CartesianPoint p;
  p.p = v4d.segment( 0, 3 );
  /** \todo check that if v4d[3] == 0 => q = cart_start_.q AND v4d[3] = or_trans_.angle() => q = cart_end_.q */
  p.q = Eigen::AngleAxisd( v4d[3], or_trans_.axis() ) * cart_start_.q ; 
  return p.toVector();
}
Eigen::VectorXd CartesianSegmentBase::getVelocity( const ArcConditions& arc_conditions ) const {
  Eigen::VectorXd v4d = StackedSegments::getVelocity( arc_conditions );
  Eigen::VectorXd v( 6 );
  v.segment( 0, 3 ) = v4d.segment( 0, 3 );
  v.segment( 3, 3 ) = v4d[3] * or_trans_.axis();
  return v;
}
Eigen::VectorXd CartesianSegmentBase::getAcceleration( const ArcConditions& arc_conditions ) const {
  Eigen::VectorXd v4d = StackedSegments::getAcceleration( arc_conditions );
  Eigen::VectorXd a( 6 );
  a.segment( 0, 3 ) = v4d.segment( 0, 3 );
  a.segment( 3, 3 ) = v4d[3] * or_trans_.axis();
  return a;
}
Eigen::VectorXd CartesianSegmentBase::getJerk( const ArcConditions& arc_conditions ) const {
  Eigen::VectorXd v4d = StackedSegments::getJerk( arc_conditions );
  Eigen::VectorXd j( 6 );
  j.segment( 0, 3 ) = v4d.segment( 0, 3 );
  j.segment( 3, 3 ) = v4d[3] * or_trans_.axis();
  return j;
}


