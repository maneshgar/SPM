#ifndef RECONSTRUCTOR_H
#define RECONSTRUCTOR_H

#include <math.h>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <iostream>
#include <string>

#include "pattern.h"
#include "pattern_sequence.h"
#include "file_manager.h"
#include "camera.h"
#include "structures.h"
#include "opencv/cv.h"
#include "Geometry/GeometricObject.h"
#include "Geometry/GeometryProcessing.h"
#include "Geometry/GeometryExporter.h"
#include "projector.h"

struct Dot_sequence{
    int ID;
    std::string name;
    Pattern black;
    Pattern white;
    Pattern mask;
    std::vector<std::string> dot_sequence_names;
};

class Reconstructor
{
public:
//    Reconstructor(Camera &input_camera_left, Camera &input_camera_right, GeometricObject &geo_obj, Projector &Proj);
    Reconstructor(Camera &input_camera_left, Camera &input_camera_right, Camera &input_camera_top, Projector &Proj);
    Reconstructor(cv::Size projector_res);
    ~Reconstructor();


    bool display_dot_pattern();
    bool reconstruct();
    bool PCloudToMesh(bool holeFilling, bool check);
    std::vector<cv::Mat> point_cloud;
    std::vector<Match> matches_points;
    bool get_projector_location();
    bool store_data();
    bool Triangulate();
    cv::Mat normal_Map;
    cv::Mat xyz_Map;

    GeometricObject *geometric_Obj;

private:
    cv::Size projector_resolution;
    cv::Size camera_resolution;
    Camera camera_left;
    Camera camera_right;
    Camera camera_top;
    Projector projector;

    bool find_mask_image(Dot_sequence &dot_seq);
    bool find_matches_points(Dot_sequence left_captured_images, Dot_sequence right_captured_images, int threshold);
    int  get_points(cv::Mat input_image, Pattern black_image, Pattern mask_image, std::vector<cv::Point2f> &matches, cv::Point2f &mean_matches);
    bool load_dot_sequence(Dot_sequence &dot_sequence, std::string directory);
    bool store_matches();
    cv::Mat find_mid_point_Bkp(Ray r1, Ray r2, float &dist);
};

#endif // RECONSTRUCTOR_H
