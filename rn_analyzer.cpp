#include <geo/dted/Geoid.hpp>
#include <geo/dted/Map.hpp>
#include <geo/Ecef.hpp>
#include <geo/Lla.hpp>
#include <geo/convert.hpp>
#include "rn_analyzer.hpp"

//****************************************************************************
//Distance Functions
//****************************************************************************
double rn_analyzer::great_circle_Distance(Point p1, Point p2)
{
  
  double rad = constants::rad;
  double earthRadius = constants::earthRadius;
  
  double phi_1 = p1.x()*rad;
  double phi_2 = p2.x()*rad;
  
  double lambda_1 = p1.y()*rad;
  double lambda_2 = p2.y()*rad;
  
  double a = pow( sin( (phi_2-phi_1)/2 ), 2 ) + cos(phi_1) * cos(phi_2) *  pow( sin( (lambda_2-lambda_1)/2 ), 2 );
  double c = 2 * atan2( sqrt(a), sqrt(1-a) );
  
  double gcd = earthRadius * c;
  return gcd;
}

double rn_analyzer::great_circle_Distance(Vertex v1, Vertex v2, Graph GRAPH)
{
  RoadNetwork rn;
  rn.point1 = get(vertex_position,GRAPH,v1);
  rn.point2 = get(vertex_position,GRAPH,v2); 
  
  boost::geometry::set<0>(rn.p1,std::get<0>(rn.point1));
  boost::geometry::set<1>(rn.p1,std::get<1>(rn.point1));
  boost::geometry::set<0>(rn.p2,std::get<0>(rn.point2));
  boost::geometry::set<1>(rn.p2,std::get<1>(rn.point2));	  
  
  double rad = constants::rad;
  double earthRadius = constants::earthRadius;
  
  double phi_1 = rn.p1.x()*rad;
  double phi_2 = rn.p2.x()*rad;
  
  double lambda_1 = rn.p1.y()*rad;
  double lambda_2 = rn.p2.y()*rad;
  
  double a = pow( sin( (phi_2-phi_1)/2 ), 2 ) + cos(phi_1) * cos(phi_2) *  pow( sin( (lambda_2-lambda_1)/2 ), 2 );
  double c = 2 * atan2( sqrt(a), sqrt(1-a) );
  
  double gcd = earthRadius * c;
  return gcd;
}

//****************************************************************************
//Vertex Functions
//****************************************************************************
std::tuple<double,double,double> rn_analyzer::get_vertex_position(Vertex v, Graph GRAPH)
{
  return get(vertex_position,GRAPH,v);
}

std::tuple<double,double,double> rn_analyzer::get_vertex_position(int vertex, struct RoadNetwork rn)
{
  return rn.nodetoCoord.at(vertex);
}

//****************************************************************************
//Edge Slope Functions
//****************************************************************************
double rn_analyzer::slope_of_edge(Vertex v1, Vertex v2, Graph GRAPH)
{
  Edge e = edge(v1,v2,GRAPH).first;
  return get(edge_slope,GRAPH,e);
}

double rn_analyzer::slope_of_edge(Edge e, Graph GRAPH)
{
  return get(edge_slope,GRAPH,e);
}

double rn_analyzer::slope_of_edge(int edge, struct RoadNetwork rn)
{
  return rn.edgeSLOPE_vec[edge];
}

//****************************************************************************
//Edge Length Functions
//****************************************************************************
double rn_analyzer::length_of_edge(Vertex v1, Vertex v2, Graph GRAPH)
{
  Edge e = edge(v1,v2,GRAPH).first;
  return get(edge_length,GRAPH,e);
}

double rn_analyzer::length_of_edge(Edge e, Graph GRAPH)
{
  return get(edge_length,GRAPH,e);
}

double rn_analyzer::length_of_edge(int edge, struct RoadNetwork rn)
{
  return rn.edgeLENGTH_vec[edge];
}

//****************************************************************************
//Road Functions
//****************************************************************************
std::vector<int> rn_analyzer::get_longest_road(struct RoadNetwork rn)
{
  int idx;
  int k = 0;
  double length1;
  for(int i = 0; i < rn.road_map_index.size(); i++)
  {
    double length2;
    for (int j = 0; j < rn.road_map_index[i].size(); j++)
    {
      idx = rn.road_map_index[i][j];
      length2 += rn.edgeLENGTH_vec[idx];
    }
    
    if(length2 > length1)
    {
      length1 = length2;
      k = i;
    }
  }
   std::cout<< "Length of road " << k << " = " << length1 << "m" << "\n";
  return rn.road_map_index[k];
}

void rn_analyzer::get_road_gog(std::vector<int> road_index, struct RoadNetwork rn)
{
  //Extract kml name
  std::size_t found = rn.kml_name.find_last_of(".");
  std::string gog_name = rn.kml_name.substr(0,found) + "_road.gog";
 
  //Create GOG file 
  //***********************************
  std::string filename;
  std::ostringstream oss;
  oss << gog_name;
  filename=oss.str();
  std::ofstream gog_file;
  gog_file.open(filename.c_str(),std::ofstream::trunc);
  //***********************************
  
  std::cout << "Constructing " << gog_name << "..." << std::endl;
  gog_file << "start" << std::endl;
  gog_file << "line" << std::endl; 
  gog_file << "linewidth 3" << std::endl; 
  for(int i = 0; i < road_index.size(); i++)
  {
    rn.point1 = rn.nodetoCoord.at(i);
    gog_file << std::fixed << "ll " << std::get<0>(rn.point1) << " " << 
           std::get<1>(rn.point1) << std::endl;
  }
  gog_file << "linecolor red" << std::endl; 
  gog_file << "end" << std::endl; 
  gog_file << "\n" << std::endl;
 
   std::cout << "gog for road generated." << "\n";
}

std::vector<int> rn_analyzer::get_intersections(struct RoadNetwork rn)
{
  std::cout << "get intersections: " << "\n";
  std::vector<int> node_deg;
  int sum;
  for (rn.vp = vertices(rn.GRAPH); rn.vp.first != rn.vp.second; ++rn.vp.first)
  {
    rn.v = *rn.vp.first;
    sum = in_degree(rn.v, rn.GRAPH) + out_degree(rn.v, rn.GRAPH);
     //std::cout << get(vertex_label, rn.GRAPH, rn.v) << " Degree = " << sum << "\n"; 
    if(sum > 2)
    {
	std::cout << get(vertex_label, rn.GRAPH, rn.v) << " Degree = " << sum << "\n";
    }
    node_deg.push_back(sum);
    sum = 0;
   
  }
    return node_deg;
}

void rn_analyzer::print_rn_map_index(struct RoadNetwork rn)
{
  std::ofstream file;
  file.open("rn_index.txt");
  for(int j=0; j < rn.road_map_index.size(); j++)
  {
    for(int i = 0; i < rn.road_map_index[j].size(); i++)
    {
      file << rn.road_map_index[j][i] << "\t";
    }
    file << "\n";
  }
}

//****************************************************************************
//Curvature Functions
//****************************************************************************
double rn_analyzer::road_curvature(Edge e, struct RoadNetwork rn)
{
   Graph::out_edge_iterator out_begin, out_end;
   Graph::in_edge_iterator in_begin, in_end;
   
  std::vector<std::vector<Vertex>> adj_nods;
  
  for(int i = 0; i < 2; i++)
  {
    std::vector<Vertex> adj_nods_v;
    if(i==0)
    {
      rn.v = source(e, rn.GRAPH);
    }
    else
    {
      rn.v = target(e, rn.GRAPH);
    }
    
    for (boost::tie(out_begin, out_end) = out_edges(rn.v,rn.GRAPH); out_begin != out_end; ++out_begin)
    {   
      //std::cout << target(*out_begin,rn.GRAPH) << std::endl;
      adj_nods_v.push_back(target(*out_begin,rn.GRAPH));
    }
    //std::cout << std::endl;

    for (boost::tie(in_begin, in_end) = in_edges(rn.v,rn.GRAPH); in_begin != in_end; ++in_begin)
    {   
      //std::cout << source(*in_begin,rn.GRAPH) << std::endl;
      adj_nods_v.push_back(source(*in_begin,rn.GRAPH));
    }
    
    //std::cout << std::endl;
    
    //Delete source or target from vector
    if(i==0)
    {
      adj_nods_v.erase(std::remove(adj_nods_v.begin(), adj_nods_v.end(), target(e, rn.GRAPH)), adj_nods_v.end());
    }
    else
    {
            adj_nods_v.erase(std::remove(adj_nods_v.begin(), adj_nods_v.end(), source(e, rn.GRAPH)), adj_nods_v.end());
    }
    
    adj_nods.push_back(adj_nods_v);
    
    std::cout << "Node " << rn.v_label_map[rn.v] << ": ";
    for(int j=0; j < adj_nods[i].size(); j++)
    {
      std::cout << rn.v_label_map[adj_nods[i][j]] << ", ";
    }
    std::cout << std::endl;
  }
 

 /*rn.index = get(vertex_index, rn.GRAPH);
 //  std::cout << "Node " << rn.index[v1] << " --> " ;
  //std::cout << "source: " << rn.index[source(f, rn.GRAPH)] << "\n";
//  std::cout << "target: " << rn.index[target(f, rn.GRAPH)] << "\n";
  
  /*std::cout << get(vertex_label, rn.GRAPH, v1) << " --> { ";
  for (boost::tie(rn.vl, rn.vr) = adjacent_vertices(v1, rn.GRAPH); rn.vl != rn.vr; ++rn.vl)
  {
    std::cout << get(vertex_label, rn.GRAPH, *rn.vl) << " ";
    std::cout << "}" << std::endl;
  }*/
  /*rn.vl = adjacent_vertices(v1, rn.GRAPH);
  Vertex v = *rn.vl.first;
  //std::tie(rn.vl,rn.vr) = boost::adjacent_vertices(v1,rn.GRAPH);*/
   //std::vector<int> v1_l;
   //std::vector<int> v2_r;
  /*Vertex v1_l;
   Vertex v2_r;
  for(tie(rn.vl,rn.vr) = adjacent_vertices(v1, rn.GRAPH); rn.vl != rn.vr; ++rn.vl)
  {
    v1_l = *rn.vl;
    std::cout << rn.index[*rn.vl] << " " ;
    //std::cout << std::endl;
  }
  for(tie(rn.vl,rn.vr) = adjacent_vertices(v2, rn.GRAPH); rn.vl != rn.vr; ++rn.vl)
  {
    v2_r = *rn.vl;
    //std::cout << rn.index[*rn.vl] << " " ;
    //std::cout << std::endl;
  }
  std::cout <<  rn.index[v1_l] << "<--- " << rn.index[v1] << "----" << rn.index[v2] << " ---> " << rn.index[v2_r] << 
"\n";*/
  return 0.0;
}


/*
//Taken from www.adamfranco.com/2012/12/05/curvature-py
std::vector<std::vector<double> > road_curvature(RoadNetwork.road_map)
{
  std::vector<std::vector<double> > curvature;
  
  for(int i=0; i <  rd_network.road_map.size(); i++)
  {
    for(int j=0; j <  rd_network.road_map[i].size(); j++)
    {
	
    }
  }
  return curvature;
}
  

void printEdges(Graph g)
{
  //** PRINT_TEST: Edges of road_network.GRAPH 
  std::cout << "edges(road_network.GRAPH) = ";

  /*for (boost::tie(road_network.ei, road_network.ei_end) = edges(road_network.GRAPH); road_network.ei != 
      road_network.ei_end; ++road_network.ei)
  {
    std::cout << "(" << road_network.index[source(*road_network.ei, road_network.GRAPH)]
		  << "," << road_network.index[target(*road_network.ei, road_network.GRAPH)] << ") ";
  }*/
  
