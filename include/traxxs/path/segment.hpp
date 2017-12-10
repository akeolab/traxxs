#ifndef TRAXXS_PATH_SEGMENT_H
#define TRAXXS_PATH_SEGMENT_H

#include <limits>
#include <list>
#include <memory>

#include <Eigen/Dense>

#include "data.hpp"
#include <traxxs/arc/arc.hpp>
#include <traxxs/constants.hpp>

namespace traxxs {
namespace path {

static const unsigned int kNCinf = std::numeric_limits<unsigned int>::max();

/** \brief returns the normalized vector, or a zero-vector if the norm of the vector is inferior to tolerance */
Eigen::VectorXd normalizedOrZero( const Eigen::VectorXd& vec, double norm_tolerance = traxxs::constants::kZero );

/** \brief stacks two vectors */
Eigen::VectorXd stack( const Eigen::VectorXd& vec_top, const Eigen::VectorXd& vec_bottom );

/** 
 * \brief a sink to consume a parameter pack. Used to avoid unused parameter packs 
 * Example usage: sink{ args ... };
 */
struct sink { template<typename ...Args> sink(Args const & ... ) {} };


struct PathConditions 
{
  PathConditions( unsigned int sz = 0 ) {
    this->dx.setConstant(  sz, std::nan("") );
    this->ddx.setConstant( sz, std::nan("") );
    this->j.setConstant( sz, std::nan("") );
  }
  Eigen::VectorXd dx;
  Eigen::VectorXd ddx;
  Eigen::VectorXd j;
};

/** 
 * \brief path waypoint 
 */
struct PathWaypoint 
{
  PathWaypoint( unsigned int sz = 0 ) {
    this->x.setConstant(   sz, std::nan("") );
    this->pathConditions = PathConditions( sz );
  }
  Eigen::VectorXd x;
  PathConditions pathConditions;
 
  arc::ArcConditions getArcConditions() const { return this->arc_conditions_; }
  bool setArcConditions( const arc::ArcConditions& arc_conditions ) { this->arc_conditions_ = arc_conditions; return true; }
  
 protected:
  /** 
   * \brief the desired conditions on the arc, if supported by the generator. Will be updated w.r.t. bounds on the segment.
   * \note arcConditions.s will not be used
   */
  arc::ArcConditions arc_conditions_;
};

struct CartesianPathWaypoint
{
  Pose x;
  PathConditions pathConditionsPosition = PathConditions( 3 );
  PathConditions pathConditionsOrientation = PathConditions( 1 );
};

struct PathBounds
{
  PathBounds( unsigned int sz = 0 ) {
    this->dx.setConstant(   sz, std::nan("") );
    this->ddx.setConstant(  sz, std::nan("") );
    this->j.setConstant(    sz, std::nan("") );
  }
  Eigen::VectorXd dx;
  Eigen::VectorXd ddx;
  Eigen::VectorXd j;
};

struct PathBounds1d : PathBounds
{
  PathBounds1d() : PathBounds( 1 ) {}
};

struct PathBounds3d : PathBounds
{
  PathBounds3d() : PathBounds( 3 ) {}
};

struct PathBounds4d : PathBounds
{
  PathBounds4d() : PathBounds( 4 ) {}
};

class StackedSegments;

class PathSegment 
{
  friend class StackedSegments;
  
 public:
  /** \brief default constructor */
  PathSegment(){}
  /**
   * \param[in] start   the start conditions, with a position and optionally desired path velocity, acceleration, jerk
   * \param[in] end     the end conditions, with a position and optionally desired path velocity, acceleration, jerk
   * \param[in] bounds  the arc desired bounds (effective bounds may be changed later)
   * 
   * \note for start/end conditions, the arc conditions will be computed automatically by the segment based on desired path vel/acc/jerk
   */
  PathSegment( const PathWaypoint& start, const PathWaypoint& end, const PathBounds& bounds ) {
    this->set( start, end, bounds );
  }
  
  template< typename... Args >
  PathSegment( const PathWaypoint& start, const PathWaypoint& end, const PathBounds& bounds, Args... args )
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
  virtual bool set( const PathWaypoint& start, const PathWaypoint& end, const PathBounds& bounds ) {
    cond_start_ = start, cond_end_ = end, path_bounds_ = bounds; 
    i_cond_start_ = start, i_cond_end_ = end, i_path_bounds_ = bounds;
  }
  
  virtual bool init(); 
   
  virtual Eigen::VectorXd getPosition( const arc::ArcConditions& arc_conditions ) const {
    return this->getConfiguration( arc_conditions.s );
  }
  virtual Eigen::VectorXd getVelocity( const arc::ArcConditions& arc_conditions ) const {
    return arc_conditions.ds * this->getDerivative( 1, arc_conditions.s );
  }
  virtual Eigen::VectorXd getAcceleration( const arc::ArcConditions& arc_conditions ) const {
    return arc_conditions.dds * this->getDerivative( 1, arc_conditions.s )
            + arc_conditions.ds*arc_conditions.ds * this->getDerivative( 2, arc_conditions.s );
  }
  virtual Eigen::VectorXd getJerk( const arc::ArcConditions& arc_conditions ) const {
    return arc_conditions.j * this->getDerivative( 1, arc_conditions.s )
            + 3.0 * arc_conditions.dds*arc_conditions.ds * this->getDerivative( 2, arc_conditions.s )
            + arc_conditions.ds*arc_conditions.ds*arc_conditions.ds * this->getDerivative( 3, arc_conditions.s );
  }
  
  
 public: // getters and setters
  virtual arc::ArcConditions getStartArcConditions() const { return this->cond_start_.getArcConditions(); }
  virtual arc::ArcConditions getEndArcConditions() const { return this->cond_end_.getArcConditions(); }
  virtual PathBounds getPathBounds() const { return this->path_bounds_; }
  virtual arc::ArcConditions getArcBounds() const { return this->arc_bounds_; }
  
  virtual bool setStartArcConditions( const arc::ArcConditions& start_arc_conditions ) { return this->cond_start_.setArcConditions( start_arc_conditions ); }
  virtual bool setEndArcConditions( const arc::ArcConditions& end_arc_conditions ) { return this->cond_end_.setArcConditions( end_arc_conditions ); }
  virtual bool setArcBounds( const arc::ArcConditions& arc_bounds ) { this->arc_bounds_ = arc_bounds; return true; }
 
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
  PathWaypoint cond_start_, cond_end_;
  PathBounds path_bounds_ ;
  arc::ArcConditions arc_bounds_;
  double length_ = std::nan("");
  
 protected:
  PathWaypoint i_cond_start_, i_cond_end_; 
  PathBounds i_path_bounds_;
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
  BlendSegment(  const PathWaypoint& start, const PathWaypoint& end, const PathBounds& bounds, 
                 const Eigen::VectorXd& waypoint,
                 Args... args ) 
      : PathSegment( start, end, bounds ){
    (void) waypoint;
    sink{ args ... };
  }
 public: 
  virtual bool init() override {
    // store the effective start/end conditions 
    this->cond_start_.x = this->getConfiguration(0.0);
    this->cond_end_.x = this->getConfiguration( this->getLength() );
    return ( true && PathSegment::init() );
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
 
 public:
  void setSegments( const std::vector< std::shared_ptr< PathSegment > >& segments ) { this->segments_ = segments; }
  
 public:
  virtual bool init() override;
  
 protected: // the interface implementation
  virtual Eigen::VectorXd do_get_derivative( unsigned int order, double s ) const override;
  virtual Eigen::VectorXd do_get_derivative_cwise_abs_max( unsigned int order ) const override;
  
 protected:
  std::vector< std::shared_ptr< PathSegment > > segments_;
};


class LinearSegment : public PathSegment
{
 public:
  LinearSegment( const PathWaypoint& start, const PathWaypoint& end, const PathBounds& bounds );
  
 public:
  virtual bool init() override;
  
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
  SmoothStep7( const PathWaypoint& start, const PathWaypoint& end, const PathBounds& bounds );
  
 public:
  virtual bool init() override;
  
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
  CircularBlend( const PathWaypoint& start, const PathWaypoint& end, const PathBounds& bounds, 
                 const Eigen::VectorXd& waypoint, 
                 double maxDeviation);
  
 public:
  virtual bool init() override;
  
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
  
 private:
   Eigen::VectorXd i_waypoint_;
   double i_max_deviation_;
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
                        const Pose& start, const Pose& end );
 public:
  virtual bool set( std::shared_ptr< PathSegment > segment_position, std::shared_ptr< PathSegment > segment_orientation, 
                        const Pose& start, const Pose& end );
  virtual bool init() override;
                        
 public: // the interface implementation
  virtual Eigen::VectorXd getPosition( const arc::ArcConditions& arc_conditions ) const override;
  virtual Eigen::VectorXd getVelocity( const arc::ArcConditions& arc_conditions ) const override;
  virtual Eigen::VectorXd getAcceleration( const arc::ArcConditions& arc_conditions ) const override;
  virtual Eigen::VectorXd getJerk( const arc::ArcConditions& arc_conditions ) const override;
  
 protected:
  std::shared_ptr< PathSegment > segment_pos_;
  std::shared_ptr< PathSegment >  segment_or_;
  /** \brief the start point of the cartesian segment */
    Pose cart_start_;
  /** \brief the end point of the cartesian segment */
    Pose cart_end_;
  Eigen::AngleAxisd or_trans_; // the transformation from cart_start_.q to cart_end_.q
};

template< class PosSegment_t, class OrSegment_t >
class CartesianSegment : public CartesianSegmentBase 
{
 public:
  /**
   * \param[in] path_bounds the bounds on the path, size 4. First 3 for position, last for angle bounds
   */
  template< typename... Args >
  CartesianSegment( const CartesianPathWaypoint& start, const CartesianPathWaypoint& end, const PathBounds& path_bounds, Args... pos_segment_args )
      : start_( start ), end_(end), bounds_( path_bounds ) {
    this->segment_pos_ = std::make_shared< PosSegment_t >( PathWaypoint(), PathWaypoint(), PathBounds(), pos_segment_args... );
    this->segment_or_ = std::make_shared< OrSegment_t >( PathWaypoint(), PathWaypoint(), PathBounds() );
  }
  
  CartesianSegment( const CartesianPathWaypoint& start, const CartesianPathWaypoint& end, const PathBounds& path_bounds ) 
      : start_( start ), end_(end), bounds_( path_bounds ) {        
    this->segment_pos_ = std::make_shared< PosSegment_t >( PathWaypoint(), PathWaypoint(), PathBounds() );
    this->segment_or_ = std::make_shared< OrSegment_t >( PathWaypoint(), PathWaypoint(), PathBounds() );
  }
  
 public:
  virtual bool init() override {
    bool ret = true;
    
    // first build the segments for position and orientation
    ret &= this->init_cartesian_pre();
    
    this->segment_pos_->set( pos_start, pos_end, pos_bounds ) ;
    this->segment_or_->set( or_start, or_end, or_bounds ) ;
    
    // init the segments ! Important, otherwise we cannot get their start/end configurations
    ret &= this->segment_pos_->init();
    ret &= this->segment_or_->init();
    
    ret &= this->init_cartesian_post();
    
    // now do the rest of the setup
    ret &= CartesianSegmentBase::set( segment_pos_, segment_or_, start_.x, end_.x );
    ret &= CartesianSegmentBase::init();
    
    return ret;
  }
  
 protected:
  /** \brief extracts position and orientation segment parameters from Cartesian parameters */
  virtual bool init_cartesian_pre() {
    /** \fixme do not throw exception in constructors */
    if ( bounds_.dx.size() != 4 || bounds_.ddx.size() != 4 || bounds_.j.size() != 4 )
      throw std::invalid_argument( "Bounds in CartesianSegment should be of size 4." );
    // extract start/end for position segment
    pos_start.x = start_.x.p;
    pos_start.pathConditions = start_.pathConditionsPosition;
    pos_end.x = end_.x.p;
    pos_end.pathConditions= end_.pathConditionsPosition; 
    
    // extract start/end for orientation segment (parameterizes the angle solely)
    or_trans_ = Eigen::AngleAxisd( end_.x.q * start_.x.q.inverse() );
    
    or_start = PathWaypoint( 1 );
    or_end = PathWaypoint( 1 );
    or_start.x << 0.0;
    or_start.pathConditions = start_.pathConditionsOrientation;
    or_end.x << or_trans_.angle();
    or_end.pathConditions = end_.pathConditionsOrientation;
    
    // extract bounds for position segment
    pos_bounds.dx   = bounds_.dx.segment(0,3);
    pos_bounds.ddx  = bounds_.ddx.segment(0,3);
    pos_bounds.j    = bounds_.j.segment(0,3);
    
    // extract bounds for orientation segment
    or_bounds.dx  = bounds_.dx.segment(3,1);
    or_bounds.ddx = bounds_.ddx.segment(3,1);
    or_bounds.j   = bounds_.j.segment(3,1);
    
    return true;
  }
  
  /** \brief recompacts position and orientation segment parameters into Cartesian parameters, in case there were some changes (blends for example!) */
  virtual bool init_cartesian_post() {
    start_.x.p = this->segment_pos_->getConfiguration( 0.0 );
    end_.x.p = this->segment_pos_->getConfiguration( this->segment_pos_->getLength() );
    
    // the same with orientations
    Eigen::Quaterniond start_orig_q = start_.x.q;
    start_.x.q = Eigen::AngleAxisd( this->segment_or_->getConfiguration( 0.0 )[0], or_trans_.axis() ) * start_orig_q ; 
    end_.x.q = Eigen::AngleAxisd( this->segment_or_->getConfiguration( this->segment_or_->getLength() )[0], or_trans_.axis() ) * start_orig_q ; 
    
    return true;
  }
  
 protected:
  CartesianPathWaypoint start_, end_;
  PathBounds bounds_; // the PathSegment::path_bounds_ will be computed in StackedSegment::init(). These ones are just for storage
  PathWaypoint pos_start, pos_end;
  PathBounds3d pos_bounds;
  PathWaypoint or_start, or_end;
  PathBounds1d or_bounds;
   
};

} // namespace traxxs 
} // namespace path 

#endif // TRAXXS_PATH_SEGMENT_H
