#include <sstream>

#include <Eigen/Core>
#include "traxxs/trajectory/trajectory.hpp"

static const Eigen::IOFormat kJSONFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " [ ", " ] ");

template < class T >
std::string toCSV(const T& obj) { 
  std::stringstream ss;
  for ( unsigned int i = 0; i < obj.size() ; ++i ) {
    ss << obj[i];
    if ( i != obj.size() -1 ) ss << ";";
  }
  return ss.str();
}

template < class T > 
std::stringstream& jsonKeyValue( std::stringstream& ss, const std::string& key, const T& value )
{
  ss << "\"" << key << "\"" << ":" << value;
  return ss;
}
template <>  // explicit specialization for Eigen::VectorXd
std::stringstream& jsonKeyValue<Eigen::VectorXd>( std::stringstream& ss, const std::string& key, const Eigen::VectorXd& value )
{
  ss << "\"" << key << "\"" << ":" << value.format( kJSONFormat );
  return ss;
}

std::string arcConditionsToJSON( const traxxs::arc::ArcConditions& arc_conditions, const std::string& suffix="" )
{
  std::stringstream ss;
  jsonKeyValue( ss, "s" + suffix, arc_conditions.s );
  ss << ",";
  jsonKeyValue( ss, "ds" + suffix, arc_conditions.ds );
  ss << ",";
  jsonKeyValue( ss, "dds" + suffix, arc_conditions.dds );
  ss << ",";
  jsonKeyValue( ss, "ddds" + suffix, arc_conditions.j );
  return ss.str();
}

std::string stateToJSON( const traxxs::trajectory::TrajectoryState& state, const std::string& suffix="" )
{
  std::stringstream ss;
  jsonKeyValue( ss, "x" + suffix, state.x );
  ss << ",";
  jsonKeyValue( ss, "dx" + suffix, state.pathConditions.dx );
  ss << ",";
  jsonKeyValue( ss, "ddx" + suffix, state.pathConditions.ddx );
  ss << ",";
  jsonKeyValue( ss, "dddx" + suffix, state.pathConditions.j );
  return ss.str();
}

std::string trajectoryFrameToJSON( double time, const traxxs::trajectory::TrajectoryState& current_state )
{
  std::string strg = "";
  strg += "\"time\":" + std::to_string( time );
  strg += "," + stateToJSON( current_state, "" );
  return strg;
}

std::string trajectoryFrameToJSON( double time, const traxxs::arc::ArcConditions& arc_conditions, const traxxs::trajectory::TrajectoryState& current_state )
{
  std::string strg = trajectoryFrameToJSON( time, current_state );
  strg += "," + arcConditionsToJSON( arc_conditions, "" );
  return strg;
}

std::string trajectoryFrameToJSON( double time, int segment_idx, const traxxs::arc::ArcConditions& arc_conditions, const traxxs::trajectory::TrajectoryState& current_state )
{
  std::string strg = trajectoryFrameToJSON( time, arc_conditions, current_state );
  strg += ",";
  strg += "\"seg_idx\":" + std::to_string( segment_idx );
  return strg;
}

std::string trajectoryFrameToJSON( double time, const traxxs::trajectory::TrajectoryState& current_state, const traxxs::trajectory::TrajectoryState& desired_state )
{
  std::string strg = trajectoryFrameToJSON( time, current_state );
  strg += "," + stateToJSON( desired_state, "_target" );
  return strg;
}



