/*
 * RoadNetwork.hpp
 *
 *  Initially Created on: Jul 29, 2016
 *      Author: healysd1
 * 
 *  Modified by thompkc1
 */

#ifndef ROADNETWORK_HPP_
#define ROADNETWORK_HPP_

#include <limits>
#include <tuple>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>
#include <boost/graph/adj_list_serialize.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/unordered_set.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/tuple.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/filesystem.hpp>
#include <geo/dted/Geoid.hpp>
#include <geo/dted/Map.hpp>
#include "save_rnf.hpp"
#include "load_rnf.hpp"

using namespace boost;
using namespace std;
namespace boost
{
  enum edge_length_t {edge_length = 111};
  enum edge_slope_t {edge_slope = 112};
  enum vertex_position_t {vertex_position = 113};
  enum vertex_label_t {vertex_label = 114};
  BOOST_INSTALL_PROPERTY(edge, length);
  BOOST_INSTALL_PROPERTY(edge, slope);
  BOOST_INSTALL_PROPERTY(vertex, position);
  BOOST_INSTALL_PROPERTY(vertex, label);
}

//One way to implement the BGL Graph concept is through the
//adjacency_list.  The fourth and fifth template parameters are used
//to define vertex and edge properties, respectively.  If we want to
//define multiple properties for a vertex we do this by using nested properties.
//In particular, the property class template has three template parameters:
//the property itself, the type, and property<> template of the next parameter.
//I.e.
//     property<prop_1, prop_1_type, property<prop_2, prop_2_type> >
//The same structure applies to edge properties.
typedef adjacency_list<vecS, vecS, bidirectionalS,
		                 property<vertex_position_t, std::tuple<double, double, double>,
		                 property<vertex_label_t, string> >,
		                 property<edge_length_t, double,
		                 property<edge_slope_t, double> > > Graph;

//Property maps are the structures used to hold properties (or features) of
//the vertices and edges.
typedef property_map<Graph, edge_length_t>::type E_LengthMap;
typedef property_map<Graph, edge_slope_t>::type E_SlopeMap;
typedef property_map<Graph, vertex_position_t>::type V_PositionMap;
typedef property_map<Graph, vertex_label_t>::type V_LabelMap;
typedef property_map<Graph, vertex_index_t>::type V_IndexMap;

typedef graph_traits<Graph>::adjacency_iterator Neighbor;
typedef std::pair<int, int> EdgePair;
typedef std::vector< EdgePair > EdgePair_vec;

typedef std::tuple<double, double,double> VPosition;

//A vertex_descriptor is what uniquely identifies a vertex.
//Internally, a vertex_discriptor is probably a pointer or
//reference to a vertex.
typedef graph_traits<Graph>::vertex_descriptor Vertex;

//A vertex_iterator allows the client program to iterate through
//the vertices.  It is like any other Standard Template Library (STL) iterator.
typedef graph_traits<Graph>::vertex_iterator VertexIter;

//An edge_descriptor is what uniquely identifies an edge.
//Internally, an edge_discriptor is probably a pointer or
//reference to an edge.
typedef graph_traits<Graph>::edge_descriptor Edge;

//An edge_iterator allows the client program to iterate through
//the edges.  It is like any other Standard Template Library (STL) iterator.
typedef graph_traits<Graph>::edge_iterator EdgeIter;

typedef boost::geometry::model::d2::point_xy<double> Point; //Brian's code; I want to use boost distance formula

typedef boost::tokenizer<boost::char_separator<char> > Tokenizer;
typedef boost::tokenizer<boost::escaped_list_separator<char> > Tokenizer_c;


//RoadNetwork structure
struct RoadNetwork
{
    std::string kml_name;
    std::vector< std::vector < std::tuple<double,double,double> > > road_map;
    std::vector< std::vector < int > > road_map_index;
    std::vector<std::tuple<double,double,double> > road;
    std::vector<int> road_index;
    boost::unordered_set < std::tuple<double,double,double> > node_list;
    boost::unordered_map< std::tuple<double,double,double>, int> coordtoNode;
    boost::unordered_map< int, std::tuple<double,double,double> > nodetoCoord;
    
    //Graph variables/functions
    Graph GRAPH;
    Neighbor vl;
    Neighbor vr;
    EdgePair edgePAIR;
    EdgePair_vec edgePAIR_vec;
    std::vector< double> edgeLENGTH_vec;
    std::vector< double> edgeSLOPE_vec;
    std::tuple<double,double,double> point1; //lat,long,altitude
    std::tuple<double,double,double> point2; //lat,long,altitude
    double dist;
    Point p1;
    Point p2;

    E_LengthMap e_length_map = get(edge_length, RoadNetwork::GRAPH);
    E_SlopeMap e_slope_map = get(edge_slope, GRAPH);
    V_PositionMap v_position_map = get(vertex_position, GRAPH);
    V_LabelMap v_label_map = get(vertex_label, GRAPH);
    V_IndexMap index = get(vertex_index, GRAPH);
    VPosition vpos;
    std::pair<VertexIter, VertexIter> vp;
    Vertex v;
    EdgeIter ei, ei_end;
    
    //Slope function
    double EdgeSLOPE(Point p1, Point p2)
    {
      double delta_x = p2.x() - p1.x();
      double delta_y = p2.y() - p1.y();
      
      double slope = delta_y/delta_x;
      if(std::isinf(std::abs(slope)) == true)
      {
	slope = std::numeric_limits<double>::max();
      }
      return slope;
    }
  
    template<class SavingArchive>
    void save(SavingArchive& ar, const unsigned int version) const
    {
      ar & kml_name;
      ar & road_map;
      ar & road_map_index;
      ar & node_list;
      ar & coordtoNode;
      ar & nodetoCoord;
    
      //Graph variables/functions
      ar & GRAPH;
      ar & edgePAIR_vec;
      ar & edgeLENGTH_vec;
      ar & edgeSLOPE_vec;
      ar & v;  
    }

    template<class LoadingArchive>
    void load(LoadingArchive& ar, const unsigned int version)
    {
      ar & kml_name;
      ar & road_map;
      ar & road_map_index;
      ar & node_list;
      ar & coordtoNode;
      ar & nodetoCoord;
    
      //Graph variables/functions
      ar & GRAPH;
      ar & edgePAIR_vec;
      ar & edgeLENGTH_vec;
      ar & edgeSLOPE_vec;
      ar & v;
    }
  BOOST_SERIALIZATION_SPLIT_MEMBER()
};

BOOST_CLASS_VERSION(RoadNetwork,1)


#endif /* ROADNETWORK_HPP_ */


