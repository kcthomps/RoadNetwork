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
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <utility>
#include <geo/dted/Geoid.hpp>
#include <geo/dted/Map.hpp>
#include <geo/Ecef.hpp>
#include <geo/Lla.hpp>
#include <geo/convert.hpp>
#include "RoadNetwork.hpp"
#include "rn_analyzer.hpp"

Point dest_point(double fract, double gcd, Point p1, Point p2)
{
  double rad = constants::rad;
  double earthRadius = constants::earthRadius;
  
  double phi_1 = p1.x()*rad;
  double phi_2 = p2.x()*rad;
  
  double lambda_1 = p1.y()*rad;
  double lambda_2 = p2.y()*rad;
 
  double a = sin( (1-fract)*(gcd/earthRadius)) / sin(gcd/earthRadius);
  double b = sin(fract*gcd/earthRadius) / sin(gcd/earthRadius);
  
  double x = a*cos(phi_1)*cos(lambda_1) + b*cos(phi_2)*cos(lambda_2);
  double y = a*cos(phi_1)*sin(lambda_1) + b*cos(phi_2)*sin(lambda_2);
  double z = a*sin(phi_1) + b*sin(phi_2);
  
  phi_1 = atan2(z, sqrt(pow(x,2)+pow(y,2))) * (1/rad);
  lambda_1 = atan2(y,x) * (1/rad);
  
  Point p;
  boost::geometry::set<0>(p,phi_1);
  boost::geometry::set<1>(p,lambda_1);
  return p;
}

double altitude(Point p)
{
 // create dted interface
  //****************************************************************************
  const geo::dted::Geoid& geoid = geo::dted::the_geoid();
  // create the map manager
  int maxLevel = 2, targetSize = 2840e3 * 3; // each cell is 2.8MB, allow 3
  std::vector<std::string> P;
  char* dted_path = std::getenv("DTED_PATH");
  if (dted_path)
  {
      //p = std::string(dted_path);
      P.push_back(std::string(dted_path));
  }
  if (P.empty())
  {
      char* home = std::getenv("HOME");
      if (home)
      {
          boost::filesystem::path tmp(std::string(home) + "/data/dem/dted");
          //p = tmp.string();
          P.push_back(tmp.string());
      }
  }
  geo::dted::Map map(P, maxLevel, targetSize);
 
  double alt = map.elevation(p.x(),p.y()) + geoid.elevation(p.x(),p.y());
  return alt;
}

std::vector<std::vector< std::tuple<double, double, double> > > check_road_nodes(struct RoadNetwork rd_network, 
double node_spacing)
{
  double num_pts_added = 0;
  rn_analyzer rn_analyzer1;
  
  for(int i = 0; i < rd_network.road_map.size(); i++)
  {
    rd_network.dist = 0;
    double dx = 0;
    double dy = 0;
    double gcd = 0;
    //std::cout << "Road: " << i << " ---> Nodes: " << rd_network.road_map[i].size() << "\n";
    //Check number of nodes on the road_map
    int points_to_add;
    for(int j = 0; j < rd_network.road_map[i].size() - 1; j++)
    {
      boost::geometry::set<0>(rd_network.p1,std::get<0>(rd_network.road_map[i][j]));
      boost::geometry::set<1>(rd_network.p1,std::get<1>(rd_network.road_map[i][j]));
      boost::geometry::set<0>(rd_network.p2,std::get<0>(rd_network.road_map[i][j+1]));
      boost::geometry::set<1>(rd_network.p2,std::get<1>(rd_network.road_map[i][j+1]));
      gcd = rn_analyzer1.great_circle_Distance(rd_network.p1,rd_network.p2);
      
      //** PRINT_TEST: Point p1, Point p2, great_circle_Distance, and total distance
      /*std::cout << std::fixed << "[" << rd_network.p1.x() << ", " << rd_network.p1.y() << "] :: " << "[" << 
	  rd_network.p2.x() << ", " << rd_network.p2.y() << "]: [" << j << ", " << j+1 << "] --> " << d << "\n";
      */
      
      //Insert number of points based on distance
      points_to_add = (int) std::floor(gcd / node_spacing);
      num_pts_added += points_to_add;
      if(points_to_add > 0)
      {
	//std::cout << "Road " << i << " has " << rd_network.road_map[i].size() << " nodes." << "\n";
	//** PRINT_TEST: j value correct?
	//std::cout << j << " before [" << rd_network.p2.x() << ", " << rd_network.p2.y() << "]" << "\n";
	//** PRINT_TEST: distance, points_to_add
	//std::cout << gcd << "/" << node_spacing << "=" << points_to_add << "\n";
	//std::cout << rd_network.p1.x() << "," << rd_network.p1.y() << "\n";
	Point new_pt;
	for(int k = 0; k < points_to_add; k++)
	{
	  //std::cout << "(" << rd_network.p1.x() << ", " << rd_network.p1.y() << ")";
	  double fract = (double) (k+1)/(points_to_add + 1);
	  new_pt = dest_point(fract,gcd,rd_network.p1,rd_network.p2);
	  
	  double alt = altitude(new_pt);
	  
	  rd_network.road_map[i].insert(rd_network.road_map[i].begin() + (j+k+1), 
		    std::make_tuple(new_pt.x(), new_pt.y(), alt) );
	  //std::cout << " ---> (" << new_pt.x() << ", " << new_pt.y() << ")" << "\n";
	}
	//update j
	j+=points_to_add;
	
	//** PRINT_TEST: j update value correct?
	/*std::cout << j << " before [" << rd_network.p2.x() << ", " << rd_network.p2.y() << "]" << "\n";
	  std::cout << j << " after [" << std::get<0>(rd_network.road_map[i][j+1]) << ", " << 
	  std::get<1>(rd_network.road_map[i][j+1]) << "]" << "\n";*/

	  //std::cout << rd_network.p2.x() << "," << rd_network.p2.y() << "\n";
	  //std::cout << "---------------------------------------------------------------------" << "\n";
	
      }
      
      rd_network.dist += gcd;
    }//j-loop
    //std::cout << rd_network.road_map[i].size() + points_to_add << "\n";
    /*if(num_pts_added > 0)
    {std::cout << num_pts_added << " point(s) added to Road " << i << "\n";}*/
    /*std::cout << "Total Distance: " << rd_network.dist << "km" << "\n";
    std::cout << "---------------------------------------------------------------------" << "\n";
    */
    //reset num_pts_added
    num_pts_added = 0;
  }//i-loop
  return rd_network.road_map;
}