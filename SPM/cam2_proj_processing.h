//#ifndef CAM2_PROJ_PROCESSING_H
//#define CAM2_PROJ_PROCESSING_H

//#include "camera.h"
//#include "projector.h"
//#include "GeometricObject.h"
//#include "structures.h"
//#include "Utilities.h"
//#include "Color.h"
//#include "Image.h"
//#include "cminpack/cminpak.h"
//#include <cmath>

//class Cam2_Proj_Processing
//{
//public:
//    Cam2_Proj_Processing(Camera &cam_left, Camera &cam_right, Projector &proj, GeometricObject &Geometric_Obj, std::vector<Match> &matches, cv::Mat xyz, cv::Mat normal_map);
//    void compute_diffuse_map_grayScale(Color input_color, cv::Mat &cos, cv::Mat &left_output, cv::Mat &right_output);
//    void compute_diffuse_map_grayScale(Color input_color, cv::Mat &cos, cv::Mat &output, int mode);
//    void compute_specular_map_grayScale(Color input_color, cv::Mat Diffuse_Part, cv::Mat &Specular_Map_output, cv::Mat &rdotv, int mode);

//    void apply_LM(Color Intensity, cv::Mat Diffuse_Map, cv::Mat nDOTl, cv::Mat Specular_Map, cv::Mat rDOTv, float alpha, cv::Mat &LM_diffuse_output, cv::Mat &LM_specular_output);

//    cv::Mat render_image(cv::Mat Light, cv::Mat kd, cv::Mat cos);
//    cv::Mat render_image(Color Light, cv::Mat kd, cv::Mat cos);


//private:
//    Camera camera_left;
//    Camera camera_right;
//    Projector projector;
//    GeometricObject Geo_Object;
//    std::vector<Match> Matches_Points;
//    cv::Mat xyz_Map;
//    cv::Mat normal_Map;

//};

//#endif // CAM2_PROJ_PROCESSING_H
