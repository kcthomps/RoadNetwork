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
#include <boost/graph/graphviz.hpp>
#include <boost/tokenizer.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/program_options.hpp>
#include <boost/system/error_code.hpp>
#include <boost/geometry/multi/geometries/multi_polygon.hpp>
#include <boost/assign.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/property_map.hpp>
#include <vector>
#include <boost/program_options.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>
#include <boost/geometry/multi/geometries/multi_polygon.hpp>
#include <boost/assign.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <utility>
#include <geo/dted/Geoid.hpp>
#include <geo/dted/Map.hpp>
#include <geo/Ecef.hpp>
#include <geo/Lla.hpp>
#include <geo/convert.hpp>
#include "RoadNetwork.hpp"
#include "check_road_nodes.hpp"
#include "rn_analyzer.hpp"

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

 struct RoadNetwork kml_to_rn(std::string kml_file, double node_spacing)
{
 
  // create dted interface
  //****************************************************************************
  const geo::dted::Geoid& geoid = geo::dted::the_geoid();
  // create the map manager
  int maxLevel = 2, targetSize = 2840e3 * 3; // each cell is 2.8MB, allow 3
  std::vector<std::string> p;
  char* dted_path = std::getenv("DTED_PATH");
  if (dted_path)
  {
      //p = std::string(dted_path);
      p.push_back(std::string(dted_path));
  }
  if (p.empty())
  {
      char* home = std::getenv("HOME");
      if (home)
      {
          boost::filesystem::path tmp(std::string(home) + "/data/dem/dted");
          //p = tmp.string();
          p.push_back(tmp.string());
      }
  }
  geo::dted::Map map(p, maxLevel, targetSize);

  //Parsing kml file
  //*****************************************************************************
  std::cout << "\n" << "Parsing " << kml_file << "..." << std::endl;
  
  std::ifstream in(kml_file.c_str());
  std::string linestr = "<LineString";
  std::string coord = "<coordinates";
  std::string coords = "</coordinates>";
  boost::char_separator<char> obj("<coordinates/>");
 
  
  //Create a RoadNetwork 
  RoadNetwork rn;
 
  //Set RoadNetwork kml name
  rn.kml_name = kml_file;
 
  //Set up variables to read in kml file
  std::string line;
  std:string line2;
  std::vector<double> coordinate(3);
  int k = 1;
  
  //Read from kml file
  while ( std::getline(in,line) )
  {
    //LineString defines roads. Only want <coordinate> blocks contained within <LineString> blocks
    if(strstr(line.c_str(),linestr.c_str()))
    {
      //std::cout << line << "\n";
      while(std::getline(in,line))
      {
	//if line contains "<coordinates" parse string
	if(strstr(line.c_str(),coord.c_str()))
	{
	    //std::cout << line << "\n";
	  //Sometimes lon/lat/alt values appear on the next line
	  //Check the line length and iterate until exit tag "</coordinates>" is reached
	  line = boost::trim_copy(line);
	  if ( line.length() == 13 )
	  {
	    line2 = line;
	    while(std::getline(in,line))
	    {
	      line2 += line;
	      if(strstr(line.c_str(),coords.c_str())) 
	      {
		break;
	      } 
	    }
	    line = line2;
	  }
      
	  //Initialize boost tokenizer to tokenize string by obj = "<coordinates/>"
	  Tokenizer tok(line,obj);
	  
	  //Extract data between coordinate tags <coordinates>...</coordinates> 
	  for(boost::tokenizer<boost::char_separator<char> >::iterator beg=tok.begin(); beg!=tok.end();++beg)
	  {
	    //Remove leading and trailing spaces after the string
	    std::string rd = boost::trim_copy(*beg); 
	    
	    //Replace spaces between altitude and next longitude values with commas
	    boost::replace_all(rd, " ", ",");
	    
	    //"rd" is now a comma separated list. Parse list by comma put lon,lat,alt values into a triplet
	    //Initialize boost csv tokenizer for "rd"
	    Tokenizer_c tok2(rd);
	    
	    std::string temp;
	    
	    //As defined in "http://gis.stackexchange.com/questions/8650/measuring-accuracy-of-latitude-and-longitude",
	    /*The sixth decimal place is worth up to 0.11 m: you can use this for laying out structures in detail, for 
	      designing landscapes, building roads. It should be more than good enough for tracking movements of 
	      glaciers and rivers. This can be achieved by taking painstaking measures with GPS, such as differentially 
	      corrected GPS.
	    */
	    //Truncation scalar
	    double scale = 0.000001;
	    //put lon,lat,alt values into a tuple
	    for(boost::tokenizer<boost::escaped_list_separator<char> >::iterator 
		beg2=tok2.begin();beg2!=tok2.end();++beg2)
	    {
	      temp = *beg2;
	      //Truncate lat,lon,alt to six decimal places 
	      coordinate[k-1] = std::floor(std::atof(temp.c_str()) / scale + 0.5) * scale;
	      
	      if(k==3)
	      {
		//An associated lon,lat,alt value has been added.
		//Before making a tuple and pushing the coordinate back to the road check altitude
		//Most of the time lon,lat values are clamped to the grown so alt=0.
		//Check for this case. If altitude clamped to grown compute altitude using dted.
		
		//dted altitude calculation
		double alt = map.elevation(coordinate[1],coordinate[0]) + geoid.elevation(coordinate[1],coordinate[0]);
		if(coordinate[2] == 0 && alt != 0)
		{
		  coordinate[2] = alt;
		}
		
		//Push coordinate tuple to road
		rn.road.push_back(std::make_tuple(coordinate[1],coordinate[0],coordinate[2]));
	      
		//Reset k
		k=0;
	      }
	      //Increase k
	      k++;
	    }
	  }
	
	  //RoadNetwork struct
	  rn.road_map.push_back(rn.road);
	  rn.road.erase(rn.road.begin(),rn.road.end());
	  break;
	}
      }
    }
   }
   //Check validity of KML
   if(rn.road_map.size() == 0)
   {
     std::cout << "kml_to_graph failed because " << kml_file << " does not contain any roads.\n";
     std::cout << "Program terminated.\n";
     exit (EXIT_FAILURE);
   }
      
   //std::cout << rn.road_map.size() << "\n";
   
   
   //Checking node distance: Iterate through the nodes of rn.road_map and 
   //add nodes if distance exceeds node_spacing 
   //****************************************************************************
   std::cout << "Checking road node spacing > " << node_spacing << "km..." << std::endl;
   rn.road_map = check_road_nodes(rn,node_spacing);
   
   //** PRINT_TEST: rn.road_map size **
   //std::cout << "Road has "<< rn.road_map.size() << " roads" << "\n";
  
   //Creating boost graph: Use road_network and node lists to create boost graph
   //****************************************************************************
    std::cout << "Transforming "<< kml_file << " to graph..." << std::endl;
  
   //Generate a list of nodes
    for(int i=0; i <  rn.road_map.size(); i++)
    {
      for(int j=0; j <  rn.road_map[i].size(); j++)
      {
	rn.node_list.insert( rn.road_map[i][j]);
      }
    }
    
    //Generate a coordinate-to-node and node-to-coordinate map
    k=0;
    for ( auto it = rn.node_list.begin(); it != rn.node_list.end(); it++ )
    {
        rn.coordtoNode[*it] = k;
	rn.nodetoCoord[k] = *it;
	k++;
    }

    //create road_map_index
    for(int i=0; i < rn.road_map.size();i++)
    {
      for(int j=0; j< rn.road_map[i].size();j++)
      {
	rn.point1 = rn.road_map[i][j];
	rn.road_index.push_back(rn.coordtoNode.at(rn.point1));
      }
      rn.road_map_index.push_back(rn.road_index);
      rn.road_index.erase(rn.road_index.begin(),rn.road_index.end());
    }
    
    
    //** PRINT_TEST: rn.coordtoNode values **
    /*for (auto& x: rn.coordtoNode) 
    {
      std::cout << "value = "<< x.second << " key = " << std::fixed << std::get<0>(x.first) << ", " << 
      std::get<1>(x.first) << ", " << std::get<2>(x.first) << "\n";
      break;
    }*/
    
    //Create EdgePAIRS, compute Edge distance, and compute Edge slope
    for(int i=0; i < rn.road_map.size(); i++)
    {
      for(int j=0; j < rn.road_map[i].size()-1; j++)
      {
	//Creating EdgePair
	rn.point1 = rn.road_map[i][j];
	rn.point2 = rn.road_map[i][j+1];
	
	//Check for duplicate points before creating edge
	if(rn.point1 != rn.point2)
	{
	  rn.edgePAIR.first = rn.coordtoNode.at(rn.point1); //returns value for key = point1
	  rn.edgePAIR.second = rn.coordtoNode.at(rn.point2); //returns value for key = point2
	  rn.edgePAIR_vec.push_back(rn.edgePAIR);
	  
	  //Computing EdgeDistance
	  boost::geometry::set<0>(rn.p1,std::get<0>(rn.road_map[i][j]));
	  boost::geometry::set<1>(rn.p1,std::get<1>(rn.road_map[i][j]));
	  boost::geometry::set<0>(rn.p2,std::get<0>(rn.road_map[i][j+1]));
	  boost::geometry::set<1>(rn.p2,std::get<1>(rn.road_map[i][j+1]));
	  
	  //Create road network analyzer to compute spherical distance
	  rn_analyzer rn_analyzer1;
	  rn.dist = rn_analyzer1.great_circle_Distance(rn.p1,rn.p2);
	  
	  rn.edgeLENGTH_vec.push_back(rn.dist);
	  rn.edgeSLOPE_vec.push_back(rn.EdgeSLOPE(rn.p1,rn.p2));
	  
	  //** PRINT_TEST: rn.p1, rn.p2, and rn.EdgeSLOPE(rn.p1,rn.p2) values
	  /*std::cout << std::fixed << rn.p1.x() << "," << rn.p1.y() << "-->" << road_network.p2.x() << 
	    "," << rn.p2.y() << ", " << "M = " << rn.EdgeSLOPE_vec[k] << "\n";
	  */
	}
      }
    }
    
    int num_edges = rn.edgePAIR_vec.size();
    
    //Add edges to graph
    for (int i = 0; i < num_edges; ++i)
    {
      add_edge(rn.edgePAIR_vec[i].first, rn.edgePAIR_vec[i].second, rn.GRAPH);
    }
 
    //Add vertex positions and vertext labels to graph
    k=0;
    std::stringstream int2str;
    std::string s;
    
    //testing write to dot
     std::vector<std::string> names(rn.coordtoNode.size());
    for (rn.vp = vertices(rn.GRAPH); rn.vp.first != rn.vp.second; ++rn.vp.first)
    {
      //Convert vertex iterator pair vp to a Vertex
      rn.v = *rn.vp.first;
      
      //Extract node tuple from nodetoCoord by index (or ID)
      rn.point1 = rn.nodetoCoord.at(k);
      
      //Add the position of the lat, lon, and altitude values from the node tuple to double pair vpos
      //rn.vpos = std::make_tuple(std::get<0>(rn.point1), std::get<1>(rn.point1), std::get<2>(rn.point1));
      
      //Add the Vertex and associated vertex position to the vertex position map for the rn.GRAPH
      put(rn.v_position_map, rn.v, rn.point1);
     
      //Labels for the graph are 1...N. Labels have to be strings.
      //Convert the numerical label to a string and pass it to the rn label map
     
      s = std::to_string(rn.coordtoNode.at(rn.point1));
      put(rn.v_label_map, rn.v, s);
      names[k] = s;
      
      //** PRINT_TEST: Vertex, vertex lat, vertex lon, node lat, node lon
      /*std::cout << rn.v << "-> " << rn.vpos.first << ", " << rn.vpos.second << "<--->";
        std::get<0>(rn.point1) << ", " << std::get<1>(rn.point1) << 
	"---->" << rn.coordtoNode.at(point1)  << "\n";
      */
      //Increase k
      k++;
    }
 
    //Associate each edge with its corresponding length and slope
    int i = 0;
    for (boost::tie(rn.ei, rn.ei_end) = edges(rn.GRAPH); rn.ei != rn.ei_end; 
	++rn.ei)
    {
      put(rn.e_length_map, *rn.ei, rn.edgeLENGTH_vec.at(i) );
      put(rn.e_slope_map, *rn.ei, rn.edgeSLOPE_vec.at(i));
      ++i;
    }
   
    //Write the boost graph to dot file for later use
    //****************************************************************************
    /*std::ofstream DATAFILE;
    string file = kml_file +".rnf";
    DATAFILE.open(file.c_str());
    
    for i*/
    
    //string file = kml_file +".dot";
    //boost::write_graphviz(DATAFILE,rn.GRAPH,boost::make_label_writer(&names[0])) ;

    //Convert dot to png, run from the command line to view boost graph
    //Note this is suitable for small networks
    //dot -v -Tpng input.dot > output.png
    
  std::cout << kml_file << " to graph complete" << std::endl;
   
  return rn;
}

  