#ifndef RN_ANALYZER_H
#define RN_ANALYZER_H

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <algorithm>
#include <geo/convert.hpp>
#include "RoadNetwork.hpp"

#define _USE_MATH_DEFINES

namespace constants
{
  const double PI = M_PI; //3.14159265; 
  const double rad = PI / 180;
  const double earthRadius = 6371; //apprx km
}

class rn_analyzer
{
  public:
    //Distance Functions
    double great_circle_Distance(Point p1, Point p2);
    double great_circle_Distance(Vertex v1, Vertex v2, Graph GRAPH);
    
    //Vertex Functions
    std::tuple<double,double,double> get_vertex_position(Vertex v, Graph GRAPH);
    std::tuple<double,double,double> get_vertex_position(int vertex, struct RoadNetwork rn);
    
    //Edge Slope Functions
    double slope_of_edge(Vertex v1, Vertex v2, Graph GRAPH);
    double slope_of_edge(Edge e, Graph GRAPH);
    double slope_of_edge(int edge, struct RoadNetwork rn);
    
    //Edge Length Functions
    double length_of_edge(Vertex v1, Vertex v2, Graph GRAPH);
    double length_of_edge(Edge e, Graph GRAPH);
    double length_of_edge(int edge, struct RoadNetwork rn);
   
    //Road Functions
    std::vector<int> get_longest_road(struct RoadNetwork rn);
    void get_road_gog(std::vector<int> road_index, struct RoadNetwork rn);
    std::vector<int> get_intersections(struct RoadNetwork rn);
    void print_rn_map_index(struct RoadNetwork rn);
    
    //Road Curvature Functions
    //double road_curvature(Vertex v, Graph GRAPH);
    double road_curvature(Edge e, struct RoadNetwork rn);
   
};

#endif //RN_ANALYZER_H