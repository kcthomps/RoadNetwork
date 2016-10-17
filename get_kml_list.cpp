#include <fstream>
#include <iostream>
#include <string>
#include <cstdlib>
#include <vector>

std::vector<std::string> get_kml_list()
{
    std::cout << "Generating kml list..." << std::endl;
    //std::system("ls | grep \"../.*.kml$\" > kml_file_list.txt"); 
    std::system("ls ../*.kml > kml_file_list.txt");
    std::system("wc -l kml_file_list.txt | awk '{print $1}' > num_kml_files.txt");
    
    std::ifstream input_file;
    /*input_file.open("num_kml_files.txt");
    int num_kml_files = 0;
    input_file >> num_kml_files;
    input_file.close();*/
    
    std::vector<std::string> kml_files;
    std::string file;
    
    input_file.open("kml_file_list.txt");
    while(input_file >> file) {
        kml_files.push_back(file);
    }
    
    /*for(int i = 0; i < num_kml_files; i++)
    {
      input_file >> kml_files[i];
    }*/
    input_file.close();
    
    //std::cout << kml_files[1] << std::endl;
    return kml_files;
   
}