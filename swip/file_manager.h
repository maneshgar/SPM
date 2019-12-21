#ifndef FILE_MANAGER_H
#define FILE_MANAGER_H

#include <iostream>
#include <string>
#include <vector>
#include <dirent.h>
#include <fstream>
#include <algorithm>
#include "sstream"


class File_Manager
{
public:

    File_Manager();
    File_Manager(std::string input_root);

    int get_dir_list(std::vector<std::__cxx11::string> &output_flist, std::__cxx11::string sub_directory);
    bool write_to_file(std::string path, std::string str);
    bool set_root(std::string input_root);
    bool read_matrix_file(std::string sub_directory, cv::Mat &output_mat);


    std::string mat_to_string(cv::Mat input_matrix);
    std::string mat_to_string_file_format(cv::Mat input_matrix);
    std::string vec3f_to_string(cv::Vec3f input_vector);
    std::string get_root();

    void export_obj(std::vector<cv::Mat> point_cloud);
    void export_Mat32FC3(cv::Mat data, std::string name);

    std::vector<cv::Mat> import_obj(std::string path);
    void import_Mat32F(std::__cxx11::string path, cv::Mat &imported_mat);

private:
    std::string root;


};

#endif // FILE_MANAGER_H
