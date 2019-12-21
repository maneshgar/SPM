#include "imaging_device.h"

Imaging_Device::Imaging_Device(){
    distortion_coeffs   = cv::Mat::zeros(1,5,CV_32FC1);
    intrinsics          = cv::Mat::eye(3,3,CV_32FC1);
    rotation            = cv::Mat::eye(3,3,CV_32FC1);
    translation         = cv::Mat::eye(3,1,CV_32FC1);
}

Imaging_Device::Imaging_Device(const Imaging_Device &input_device){
    distortion_coeffs = input_device.distortion_coeffs.clone();
    intrinsics = input_device.intrinsics.clone();
    rotation = input_device.rotation.clone();
    translation = input_device.translation.clone();
}

Imaging_Device::Imaging_Device(int input_ID, std::string input_name) : Imaging_Device()
{
    this->ID = input_ID;
    this->name = input_name;
}


Imaging_Device::Imaging_Device(int input_ID, std::string input_name, cv::Size res) : Imaging_Device()
{
    translation.at<float>(0,0) = 1.0;
    this->ID = input_ID;
    this->name = input_name;
    this->resolution = res;
}

Imaging_Device::~Imaging_Device(){

}

cv::Mat Imaging_Device::getRotationMatrix(){
    return rotation;
}

cv::Mat Imaging_Device::getTranslationMatrix(){
    return translation;
}

cv::Mat Imaging_Device::getdistortion_coeffsMatrix(){
    return distortion_coeffs;
}

cv::Mat Imaging_Device::get_extrinsics(){

    cv::Mat extrinsics;
    cv::Mat translation = cv::Mat::eye(4,4,CV_32FC1);
    cv::Mat rotation = cv::Mat::eye(4,4,CV_32FC1);
    //    cv::Rodrigues(this->rotation, temp_rotation);

    translation.at<float>(0,3) = -this->translation.at<float>(0,0);
    translation.at<float>(1,3) = -this->translation.at<float>(1,0);
    translation.at<float>(2,3) = -this->translation.at<float>(2,0);

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rotation.at<float>(i,j) = this->rotation.at<float>(i,j);
        }
    }
    rotation = rotation.inv();
    extrinsics = rotation * translation;
    return extrinsics;
}

void Imaging_Device::get_RT_with_another_device(Imaging_Device device, cv::Mat &rotation, cv::Mat &translation){

    cv::Mat Rot(cv::Mat(3,3,CV_32FC1));
    cv::Mat Trans(cv::Mat(3,3,CV_32FC1));

    cv::Mat device_rotation_matrix;
    cv::Mat this_rotation_matrix;

    cv::Rodrigues(this->rotation, this_rotation_matrix);
    cv::Rodrigues(device.rotation, device_rotation_matrix);

    Rot = (this_rotation_matrix.t()) * device_rotation_matrix;
    Trans = (this_rotation_matrix.t()) * (device.translation - this->translation);

    rotation = Rot.clone();
    translation = Trans.clone();
    return;
}

void Imaging_Device::compute_world_coordinates(){
    float data[4] = {0.0, 0.0, 0.0, 1.0};
    cv::Mat point(4,1, CV_32FC1, &data);
    cv::Mat tempT = cv::Mat::eye(4,4,CV_32FC1);
    cv::Mat tempR = cv::Mat::eye(4,4,CV_32FC1);

    rotation.copyTo(tempR(cv::Range(0,3), cv::Range(0,3)));

    tempT.at<float>(0,3) = -translation.at<float>(0,0);
    tempT.at<float>(1,3) = -translation.at<float>(1,0);
    tempT.at<float>(2,3) = -translation.at<float>(2,0);

    this->world_coordinates = (tempR.t() * tempT) * point;
    return;
}

void Imaging_Device::compute_world_coordinates(cv::Mat &point){
    cv::Mat tempT(translation);
    tempT.at<float>(0,3) = -tempT.at<float>(0,3);
    tempT.at<float>(1,3) = -tempT.at<float>(1,3);
    tempT.at<float>(2,3) = -tempT.at<float>(2,3);

    this->world_coordinates = (rotation.t() * (tempT)) * point;
    return;
}

void Imaging_Device::set_world_coordinates(cv::Mat data){
    this->world_coordinates = data.clone();
    return;
}



cv::Mat Imaging_Device::get_world_coordinates(){
//    std::cout << "Warning :: be careful you may have a error in cv::Mat Imaging_Device::get_world_coordinates()"
//                 "since you have rotation in one place by vec3 and other place in matrix, the problem may happen when you calibrate and the ask for the location." << std::endl;
    return this->world_coordinates;
}

cv::Point2f Imaging_Device::undistort_Pixel(cv::Point2f &distortedPixel)
{
    float k[5] = {0.0};
    float fx, fy, ifx, ify, cx, cy;
    int iters = 1;

    k[0] = this->distortion_coeffs.at<float> (0,0);
    k[1] = this->distortion_coeffs.at<float> (0,1);
    k[2] = this->distortion_coeffs.at<float> (0,2);
    k[3] = this->distortion_coeffs.at<float> (0,3);
    k[4] = 0;

    iters = 5;

    fx = this->intrinsics.at<float>(0,0);
    fy = this->intrinsics.at<float>(1,1);
    ifx = 1.0/fx;
    ify = 1.0/fy;
    cx = this->intrinsics.at<float>(0,2);
    cy = this->intrinsics.at<float>(1,2);

    float x,y,x0,y0;

    x = distortedPixel.x;
    y = distortedPixel.y;

    x0 = x = (x - cx)*ifx;
    y0 = y = (y - cy)*ify;

    for(int jj = 0; jj < iters; jj++ )
    {
        float r2 = x*x + y*y;
        float icdist = 1./(1 + ((k[4]*r2 + k[1])*r2 + k[0])*r2);
        float deltaX = 2*k[2]*x*y + k[3]*(r2 + 2*x*x);
        float deltaY = k[2]*(r2 + 2*y*y) + 2*k[3]*x*y;
        x = (x0 - deltaX)*icdist;
        y = (y0 - deltaY)*icdist;
    }
    cv::Point2f output((float)(x*fx)+cx,(float)(y*fy)+cy);
    return output;
}

bool Imaging_Device::loadConfig(const std::string& configFile)
{
    enum CAMERA_MAT{
        CAMERA_MAT=0,
        DISTOR_MAT,
        ROT_MAT,
        TRANS_MAT,
        PARAM_COUNT
    };

    std::array<cv::Mat, PARAM_COUNT> params_;
    cv::FileStorage fs(configFile, cv::FileStorage::READ);
    fs.root()["Image_Device"]["Matrix"]>>params_[CAMERA_MAT];
    fs.root()["Image_Device"]["Distortion"]>>params_[DISTOR_MAT];
    fs.root()["Image_Device"]["Translation"]>>params_[TRANS_MAT];
    fs.root()["Image_Device"]["Rotation"]>>params_[ROT_MAT];
    fs.release();

    for (size_t i=0; i<PARAM_COUNT; i++)
        if (params_[i].empty())
            std::printf("Failed to load config %s\n", configFile.c_str());

    translation.at<float>(0,0)= params_[TRANS_MAT].at<double>(0);
    translation.at<float>(1,0)= params_[TRANS_MAT].at<double>(1);
    translation.at<float>(2,0)= params_[TRANS_MAT].at<double>(2);

    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            rotation.at<float>(j,i) = params_[ROT_MAT].at<double>(j,i);

    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            intrinsics.at<float>(j,i) = params_[CAMERA_MAT].at<double>(j,i);

    for (int i = 0; i < 5; ++i)
        distortion_coeffs.at<float>(i,0)= params_[DISTOR_MAT].at<double>(i);

}

void Imaging_Device::saveConfig(const std::__cxx11::string &configFile){

    cv::FileStorage fs("output/calibration/" + configFile, cv::FileStorage::WRITE);

    fs << "Image_Device" << "{:";
    fs<< "Calibrated" << true << "Matrix" << intrinsics << "Distortion" << distortion_coeffs <<"Translation"<< translation <<"Rotation"<< rotation;
    fs<<"Height" << resolution.height <<"Width" << resolution.width;
    fs<<"}";

    fs.release();
}









