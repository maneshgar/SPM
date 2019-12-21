#ifndef LIGHTPROCCESSING_H
#define LIGHTPROCCESSING_H

#include "Image/Image.h"

#include <iostream>
#include "structures.h"
#include "file_manager.h"
#include "camera.h"
#include "Utilities.h"
#include "projector.h"
#include "cminpack/cminpak.h"


using namespace cv;

class lightProccessing
{
public:
    lightProccessing(){}
    lightProccessing(Camera &left, Camera &right, Camera &top, Projector &p);
    ~lightProccessing();

    void set_xyzMap(Mat data, bool check);
    void set_normal(float data, int i, int j);
    void set_normals(Mat data, bool check);
    void set_alpha(float data);
    void load_matches(bool check);
    void load_scene(bool check);
    int  load_cameras(bool check);

    Size get_resolution();
    int get_height();
    int get_width();

    float get_cosNL(int i, int j);
    float get_cosRV(int i, int j, int camera_number);

    float get_alpha();
    float get_type(int i, int j);
    Vec3f get_normal(int i, int j);
    Vec3f get_point(int i, int j);
    Vec3f get_lightSource_Vec3f(int i, int j);

    float get_grayKd(int i, int j);
    float get_grayKs(int i, int j);
    Color get_lightSource(int i, int j);
    float get_lightSource_grayScale(int i, int j);

    bool is_Node(int i, int j);
    bool is_typeX(int i, int j, int type);

    void initialize_colors(std::string pathes, bool check);
    float compute_Alpha(Mat diffuse_part);
    void compute_Mask_Image(bool check);
    void compute_CosNL(bool check);
    void compute_CosRVs(bool check);

    void compute_kd(Mat Intensity, Mat LightSource, Mat &Kd_output, bool check);
    void Interpolate_Diffuse_map(int mode, bool check);
    void compute_Diffuse_map(int mode, bool check);

    void compute_Diffuse_map_grayScale(int mode, bool check);
    void compute_Specular_map_grayScale(int Camera_number, Mat Diffuse_Part, int specular_id, bool check);

    void test_Levenberg_Marquardt(bool check);
    void create_grayScale_test(bool check);

    void create_MacbetChart_testCase(bool check);

    void render_channel(Mat Diffuse, Mat Specular, Mat light, int camera_position, Mat &output, bool check);

    void fill_type_table(bool check);
    void compute_mask_type(bool check);
    void compute_ASD_parameters(bool check);
    void compute_compensate_image(bool check);
    void project_image_on_mesh(vector<Mat> Inp_Image, vector<Mat> &Out_Image, bool check);
    void create_testCase(bool check);
    void create_testCase(std::vector<Mat> Inp_Image, bool check);
    void extract_samples(bool check);
    Vec3f getMidColor(Mat &image, Point2f &point, bool check);


    void project(std::vector<Mat> kd, std::vector<Mat> ks, std::vector<Mat> light, Mat cosTeta, Mat cosPhi, float alpha, Mat &resutl);

//    Mat render_image(bool check);
    void render_image(std::vector<Mat> Diffuse, std::vector<Mat> Specular, std::vector<Mat> Light, Mat CosNL, Mat CosRV, Mat alpha, Mat &rendered, bool check);
//    Mat render_image(int mode, int camera_number, bool check);

    Mat render_image_grayScale(bool check);
    Mat render_image_grayScale(Mat Diffuse, Mat Specular, int camera_position, bool check);
    Mat render_image_grayScale(int mode, int camera_number, bool check);

    void get_kd_ks_Alpha(bool check);

    //movaghat
    std::vector<Mat> diffuse_map;
    std::vector<Mat> specular_map;
    std::vector<Mat> diffuse_map_computed;
    std::vector<Mat> specular_map_computed;
private:

//    Vec3f ks = {187/255.0, 189/255.0, 36/255.0};
//    Vec3f kd = {0.0/255.0, 0.0/255.0, 0.0/255.0};

    Vec3f kd = {198/255.0, 24/255.0, 167/255.0};
    Vec3f ks = {56/255.0, 197/255.0, 232/255.0};
    float alpha = 8;

    int nodes = 0;
    int numberOfCameras = 3;
    int numberOfChannels = 3;

    std::vector<Camera> cameras;
    std::vector<std::vector<Vec3f>> samples;
    Projector projector;

    Mat Alpha_grayScale;
    Mat type_table;
    Mat normals;
    Mat xyz_map;
    Mat cosNL_map;
    std::vector<Mat> cosRV_map;
    std::vector<std::vector<Match>> matches_points;

    Mat Mask;
    std::vector<Mat> Mask_Type;

    std::vector<Mat> light_source;

    std::vector<Mat> Observed_map;

    Mat light_source_grayScale;
    Mat diffuse_map_grayScale;
    Mat specular_map_grayScale;

    Mat Cx, Cy, Cz;
    std::vector<Mat> Ux, Uy, Uz;
    std::vector<Mat> Vx, Vy, Vz;

    std::vector<Mat> Observed_grayScale;
};

#endif // LIGHTPROCCESSING_H
