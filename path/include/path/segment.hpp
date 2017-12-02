#ifndef TRAJGEN_PATH_SEGMENT_H
#define TRAJGEN_PATH_SEGMENT_H

#include <limits>
#include <list>
#include <memory>

#include <Eigen/Dense>

#include <path/data.hpp>
#include <arctrajgen/arctrajgen.hpp>

static const unsigned int kNCinf = std::numeric_limits<unsigned int>::max();
static const double kZero = 1.e-12;

/** \brief returns the normalized vector, or a zero-vector if the norm of the vector is inferior to tolerance */
Eigen::VectorXd normalizedOrZero( const Eigen::VectorXd& vec, double norm_tolerance = kZero );

/** \brief stacks two vectors */
Eigen::VectorXd stack( const Eigen::VectorXd& vec_top, const Eigen::VectorXd& vec_bottom );

/** 
 * \brief a sink to consume a parameter pack. Used to avoid unused parameter packs 
 * Example usage: sink{ args ... };
 */
struct sink { template<typename ...Args> sink(Args const & ... ) {} };

/** 
 * \brief path conditions 
 */
struct PathConditions 
{
  Eigen::VectorXd position;
  /** 
   * \brief the desired conditions on the arc, if supported by the generator. Will be updated w.r.t. bounds on the segment.
   * \note arcConditions.s will not be used
   */
  ArcConditions arcConditions;
};

struct CartesianPathConditions
{
  CartesianPoint position;
  ArcConditions arcConditionsPosition;
  ArcConditions arcConditionsOrientation;
};

struct PathBounds
{
  Eigen::VectorXd dx;
  Eigen::VectorXd ddx;
  Eigen::VectorXd j;
};

struct PathBounds1d : PathBounds
{
  PathBounds1d() {
    dx = Eigen::Matrix<double, 1, 1>();
    ddx = Eigen::Matrix<double, 1, 1>();
    j = Eigen::Matrix<double, 1, 1>();
  }
};

struct PathBounds3d : PathBounds
{
  PathBounds3d() {
    dx = Eigen::Vector3d();
    ddx = Eigen::Vector3d();
    j = Eigen::Vector3d();
  }
};

struct PathBounds4d : PathBounds
{
  PathBounds4d() {
    dx = Eigen::Vector4d();
    ddx = Eigen::Vector4d();
    j = Eigen::Vector4d();
  }
};



class PathSegment 
{
 public:
  /** \brief default constructor */
  PathSegment(){}
  /**
   * \param[in] start   the start conditions, with a position and optionally desired arc velocity, acceleration, jerk
   * \param[in] end     the end conditions, with a position and optionally desired arc velocity, acceleration, jerk
   * \param[in] bounds  the arc desired bounds (effective bounds may be changed later)
   */
  PathSegment( const PathConditions& start, const PathConditions& end, const PathBounds& bounds ) 
      : cond_start_( start ), cond_end_ ( end ), path_bounds_( bounds ) {
  }
  
  template< typename... Args >
  PathSegment( const PathConditions& start, const PathConditions& end, const PathBounds& bounds, Args... args ) 
      : PathSegment( start, end, bounds ) {
        sink{ args ... };
  }
  
 public: // the non-virtual public interface
  /** \brief returns the total arc length of the segment */ 
  virtual double getLength() const final { return this->do_get_length(); }
   
  /** 
   * \brief returns the configuration (position) at a given arc length 
   * \note shortcut to getDerivative( 0, s )
   */
  virtual Eigen::VectorXd getConfiguration( double s ) const final { return this->getDerivative( 0, s ); }
  /** 
   * \brief returns the order-th derivative of the configuration (position) at a given arc length 
   * \param[in] order order in [0, nContinuousDerivatives]
   * \param[in] s     the arc length at which to evaluate
   */
  virtual Eigen::VectorXd getDerivative( unsigned int order, double s ) const final { return this->do_get_derivative( order, s ) ; }
  
  /** 
   * \brief returns the component-wise maximum absolute value of the derivatives of the configuration over the entire segment 
   * \param[in] order order in [1, nContinuousDerivatives]
   * \note order=0 does not need to be implemented, makes little sense
   */
  virtual Eigen::VectorXd getDerivativeCwiseAbsMax( unsigned int order ) const final { return this->do_get_derivative_cwise_abs_max( order ) ; }
  
 public: // the virtual public interface
  virtual Eigen::VectorXd getPosition( const ArcConditions& arc_conditions ) const {
    return this->getConfiguration( arc_conditions.s );
  }
  virtual Eigen::VectorXd getVelocity( const ArcConditions& arc_conditions ) const {
    return arc_conditions.ds * this->getDerivative( 1, arc_conditions.s );
  }
  virtual Eigen::VectorXd getAcceleration( const ArcConditions& arc_conditions ) const {
    return arc_conditions.dds * this->getDerivative( 1, arc_conditions.s )
            + arc_conditions.ds*arc_conditions.ds * this->getDerivative( 2, arc_conditions.s );
  }
  virtual Eigen::VectorXd getJerk( const ArcConditions& arc_conditions ) const {
    return arc_conditions.j * this->getDerivative( 1, arc_conditions.s )
            + 3.0 * arc_conditions.dds*arc_conditions.ds * this->getDerivative( 2, arc_conditions.s )
            + arc_conditions.ds*arc_conditions.ds*arc_conditions.ds * this->getDerivative( 3, arc_conditions.s );
  }
  
  
 public: // getters and setters
  virtual ArcConditions getStartArcConditions() const { return this->cond_start_.arcConditions; }
  virtual ArcConditions getEndArcConditions() const { return this->cond_end_.arcConditions; }
  virtual PathBounds getPathBounds() const { return this->path_bounds_; }
  virtual ArcConditions getArcBounds() const { return this->arc_bounds_; }
  
  virtual bool setStartArcConditions( const ArcConditions& start_arc_conditions ) { this->cond_start_.arcConditions = start_arc_conditions; return true; }
  virtual bool setEndArcConditions( const ArcConditions& end_arc_conditions ) { this->cond_end_.arcConditions = end_arc_conditions; return true; }
  virtual bool setArcBounds( const ArcConditions& arc_bounds ) { this->arc_bounds_ = arc_bounds; return true; }
 
 protected: // the virtual implementation
  /** \brief implementation of getLength() */
  virtual double do_get_length() const { return this->length_; }
  
 protected: // the pure virtual implementation
  /** \brief implementation of getDerivative() */
  virtual Eigen::VectorXd do_get_derivative( unsigned int order, double s ) const = 0;
  /** \brief implementation of getDerivativeCwiseAbsMax() */
  virtual Eigen::VectorXd do_get_derivative_cwise_abs_max( unsigned int order ) const = 0;
  
 protected:
  /** \warning adding members here should be reverberated to StackedSegments */
  PathConditions cond_start_, cond_end_;
  PathBounds path_bounds_ ;
  ArcConditions arc_bounds_;
  double length_ = std::nan("");
};

/**
 * A blend segment computes by itself its start and end conditions position
 * \note start and end conditions position are not guaranteed to be equal to start and end parameters for blends.
 * \warning Derived classes should take care of resetting start/end conditions to the newly computed values
 */
template< typename... Args >
class BlendSegment : public PathSegment
{
 public:
  /** 
   */
  BlendSegment(  const PathConditions& start, const PathConditions& end, const PathBounds& bounds, 
                 const Eigen::VectorXd& waypoint,
                 Args... args) 
      : PathSegment( start, end, bounds ) {
        // avoid unused argument warning
        (void) waypoint;
        sink{ args ... };
      }
};

/** 
 * A stacked segments stacks multiple segments.
 * New unit arc variable u is used, u \in [0, 1]
 * stacked->getLength() = 1.0
 * \warning \todo start/end arcConditions from segments are ignored for now, and left undefined
 * \warning all derived classes should call init_stack() in their constructor
 */
class StackedSegments : public PathSegment
{
 public:
  StackedSegments(){};
  StackedSegments( const std::vector< std::shared_ptr< PathSegment > >& segments );
  StackedSegments( std::shared_ptr< PathSegment >& segment_top, std::shared_ptr< PathSegment >& segment_bottom );
 
 protected:
  virtual bool init_stack( const std::vector< std::shared_ptr< PathSegment > >& segments ) final;
  
 protected: // the interface implementation
  virtual Eigen::VectorXd do_get_derivative( unsigned int order, double s ) const override;
  virtual Eigen::VectorXd do_get_derivative_cwise_abs_max( unsigned int order ) const override;
  
 protected:
  std::vector< std::shared_ptr< PathSegment > > segments_;
};


class LinearSegment : public PathSegment
{
 public:
  LinearSegment( const PathConditions& start, const PathConditions& end, const PathBounds& bounds );
  
 protected: // the interface implementation
  virtual Eigen::VectorXd do_get_derivative( unsigned int order, double s ) const override;
  virtual Eigen::VectorXd do_get_derivative_cwise_abs_max( unsigned int order ) const override;
  
 protected:
  virtual Eigen::VectorXd get_derivative_0( double s ) const; 
  virtual Eigen::VectorXd get_derivative_1( double s ) const; 
};

/**
 * A 7-th order Smoothstep segment, with first three derivatives null at start and end
 * Arc length is between 0 and 1 
 */
class SmoothStep7 : public PathSegment 
{
 public:
  SmoothStep7( const PathConditions& start, const PathConditions& end, const PathBounds& bounds );
  
 protected: // the interface implementation
  virtual Eigen::VectorXd do_get_derivative( unsigned int order, double s ) const override;
  virtual Eigen::VectorXd do_get_derivative_cwise_abs_max( unsigned int order ) const override;
  
 protected:
  virtual Eigen::VectorXd get_derivative_0( double s ) const; 
  virtual Eigen::VectorXd get_derivative_1( double s ) const; 
  virtual Eigen::VectorXd get_derivative_2( double s ) const; 
  virtual Eigen::VectorXd get_derivative_3( double s ) const; 
};


/** 
 * A circular segment tangent to [start,waypoint] and [waypoint, end]. Serves as a corner blend.
 * \warning the segment will not start at start nor end at end points !
 * \note ddx continuity will not be ensured with a linear segment, except if dds = 0
 */ 
class CircularBlend : public BlendSegment< double >
{
 public:
  /**
   * \param[in] start the virtual starting point. Will not result in the effective start point in general (use getConfiguration(0.0) instead)
   * \param[in] end the virtual ending point. Will not result in the effective end point in general (use getConfiguration(getLength()) instead)
   * \param[in] bounds the bounds on the segment
   * \param[in] waypoint the desired waypoint of the circular blend. The segment will not pass through this point
   * \param[in] maxDeviation the maximum deviation from the waypoint. The radius of the blend will be computed accordingly
   */
  CircularBlend( const PathConditions& start, const PathConditions& end, const PathBounds& bounds, 
                 const Eigen::VectorXd& waypoint, 
                 double maxDeviation);
  
 protected: // the interface implementation
  virtual Eigen::VectorXd do_get_derivative( unsigned int order, double s ) const override;
  virtual Eigen::VectorXd do_get_derivative_cwise_abs_max( unsigned int order ) const override;
  
 protected:
  virtual Eigen::VectorXd get_derivative_0( double s ) const; 
  virtual Eigen::VectorXd get_derivative_1( double s ) const; 
  virtual Eigen::VectorXd get_derivative_2( double s ) const; 
  virtual Eigen::VectorXd get_derivative_3( double s ) const; 
  
 protected:
  Eigen::VectorXd center_;  // the center of the circle
  Eigen::VectorXd x_, y_;   // main axes
  double radius_; // the radius of the circle
};


class CartesianSegmentBase : public StackedSegments 
{
 public:
  CartesianSegmentBase(){};
  /**
  * \param[in] segment_position a segment of Vector3d
  * \param[in] segment_orientation a segment of Vector1d, representing the angle between start and end (so from 0 to angle)
  * \warning all derived classes should call init_cartesianbase() in their constructor
  */
  CartesianSegmentBase( std::shared_ptr< PathSegment > segment_position, std::shared_ptr< PathSegment > segment_orientation, 
                        const CartesianPoint& start, const CartesianPoint& end );
 protected:
  virtual bool init_cartesianbase( std::shared_ptr< PathSegment > segment_position, std::shared_ptr< PathSegment > segment_orientation, 
                        const CartesianPoint& start, const CartesianPoint& end ) final ;
                        
 public: // the interface implementation
  virtual Eigen::VectorXd getPosition( const ArcConditions& arc_conditions ) const override;
  virtual Eigen::VectorXd getVelocity( const ArcConditions& arc_conditions ) const override;
  virtual Eigen::VectorXd getAcceleration( const ArcConditions& arc_conditions ) const override;
  virtual Eigen::VectorXd getJerk( const ArcConditions& arc_conditions ) const override;
  
 protected:
  std::shared_ptr< PathSegment > segment_pos_;
  std::shared_ptr< PathSegment >  segment_or_;
  CartesianPoint cart_start_, cart_end_;
  Eigen::AngleAxisd or_trans_; // the transformation from start.q to end.q
};

template< class PosSegment_t, class OrSegment_t >
class CartesianSegment : public CartesianSegmentBase 
{
 public:
  /**
   * \param[in] path_bounds the bounds on the path, size 4. First 3 for position, last for angle bounds
   */
  template< typename... Args >
  CartesianSegment( const CartesianPathConditions& start, const CartesianPathConditions& end, const PathBounds& path_bounds, Args... pos_segment_args )
      : start_( start ), end_(end), bounds_( path_bounds ) {
    // first build the segments for position and orientation
    this->init_cartesian_pre();
    this->segment_pos_ = std::make_shared< PosSegment_t >( pos_start, pos_end, pos_bounds, pos_segment_args... );
    this->segment_or_ = std::make_shared< OrSegment_t >( or_start, or_end, or_bounds  );
    this->init_cartesian_post();
    
    // now do the rest of the setup
    this->init_cartesianbase( this->segment_pos_, this->segment_or_, start_.position, end_.position );
  }
  
  CartesianSegment( const CartesianPathConditions& start, const CartesianPathConditions& end, const PathBounds& path_bounds ) 
      : start_( start ), end_(end), bounds_( path_bounds ) {        
    // first build the segments for position and orientation
    this->init_cartesian_pre();
    this->segment_pos_ = std::make_shared< PosSegment_t >( pos_start, pos_end, pos_bounds );
    this->segment_or_ = std::make_shared< OrSegment_t >( or_start, or_end, or_bounds  );
    this->init_cartesian_post();
    
    // now do the rest of the setup
    this->init_cartesianbase( this->segment_pos_, this->segment_or_, start_.position, end_.position );
  }
  
 protected:
  /** \brief extracts position and orientation segment parameters from Cartesian parameters */
  virtual bool init_cartesian_pre() {
    /** \fixme do not throw exception in constructors */
    if ( bounds_.dx.size() != 4 || bounds_.ddx.size() != 4 || bounds_.j.size() != 4 )
      throw std::invalid_argument( "Bounds in CartesianSegment should be of size 4." );
    // extract start/end for position segment
    pos_start.position = start_.position.p;
    pos_end.position = end_.position.p;
    
    pos_start.arcConditions = start_.arcConditionsPosition;
    pos_end.arcConditions = end_.arcConditionsPosition;
    
    // extract start/end for orientation segment (parameterizes the angle solely)
    or_trans_ = Eigen::AngleAxisd( end_.position.q * start_.position.q.inverse() );
    or_start.position = Eigen::Matrix<double, 1, 1>();
    or_start.position << 0.0;
    or_end.position = Eigen::Matrix<double, 1, 1>();
    or_end.position << or_trans_.angle();
    
    or_start.arcConditions = start_.arcConditionsOrientation;
    or_end.arcConditions = end_.arcConditionsOrientation;
    
    // extract bounds for position segment
    PathBounds3d pos_bounds;
    pos_bounds.dx   = bounds_.dx.segment(0,3);
    pos_bounds.ddx  = bounds_.ddx.segment(0,3);
    pos_bounds.j    = bounds_.j.segment(0,3);
    
    // extract bounds for orientation segment
    PathBounds1d or_bounds;
    or_bounds.dx  = bounds_.dx.segment(3,1);
    or_bounds.ddx = bounds_.ddx.segment(3,1);
    or_bounds.j   = bounds_.j.segment(3,1);
    
    return true;
  }
  
  /** \brief recompacts position and orientation segment parameters into Cartesian parameters, in case there were some changes (blends for example!) */
  virtual bool init_cartesian_post() {
    start_.position.p = this->segment_pos_->getConfiguration( 0.0 );
    end_.position.p = this->segment_pos_->getConfiguration( this->segment_pos_->getLength() );
    
    // the same with orientations
    Eigen::Quaterniond start_orig_q = start_.position.q;
    start_.position.q = Eigen::AngleAxisd( this->segment_or_->getConfiguration( 0.0 )[0], or_trans_.axis() ) * start_orig_q ; 
    end_.position.q = Eigen::AngleAxisd( this->segment_or_->getConfiguration( this->segment_or_->getLength() )[0], or_trans_.axis() ) * start_orig_q ; 
    
    return true;
  }
  
 protected:
   CartesianPathConditions start_, end_;
   PathBounds bounds_;
   PathConditions pos_start, pos_end;
   PathBounds3d pos_bounds;
   PathConditions or_start, or_end;
   PathBounds1d or_bounds;
   
};


#endif // TRAJGEN_PATH_SEGMENT_H
