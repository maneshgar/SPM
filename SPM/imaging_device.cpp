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

    std::cout << "translation" << std::endl << tempT << std::endl;
    std::cout << "rotation" << std::endl << tempR << std::endl;
    std::cout << "Transformation" << std::endl << (tempR.t() * tempT) << std::endl;
    std::cout << "Position" << std::endl << (tempR.t() * tempT) * point << std::endl;
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
    return this->world_coordinates;
}

bool Imaging_Device::save_device(){
    File_Manager file_m("data/calibration/"+ this->name+"/");
    file_m.write_to_file("Intrinsics.txt" , file_m.mat_to_string_file_format(this->intrinsics));
    file_m.write_to_file("rotation.txt" , file_m.mat_to_string_file_format(this->rotation));
    file_m.write_to_file("translation.txt" , file_m.mat_to_string_file_format(this->translation));
    file_m.write_to_file("distortion_coeffs.txt" , file_m.mat_to_string_file_format(this->distortion_coeffs));
}

bool Imaging_Device::load_device(){
    File_Manager file_m("data/calibration/" + this->name + "/");
    file_m.read_matrix_file("Intrinsics.txt", this->intrinsics);
    file_m.read_matrix_file("rotation.txt", this->rotation);
    file_m.read_matrix_file("translation.txt", this->translation);
    file_m.read_matrix_file("distortion_coeffs.txt", this->distortion_coeffs);
    compute_world_coordinates();
}






///
///
cv::Point2f UnDistort_Pixel_SFM(cv::Point2f point, double k1) {

    cv::Point2f pt(point.x, point.y);
    if (k1 == 0)
        return point;

    const double t2 = pt.y*pt.y;
    const double t3 = t2*t2*t2;
    const double t4 = pt.x*pt.x;
    const double t7 = k1*(t2+t4);
    if (k1 > 0) {
        const double t8 = 1.0/t7;
        const double t10 = t3/(t7*t7);
        const double t14 = sqrt(t10*(0.25+t8/27.0));
        const double t15 = t2*t8*pt.y*0.5;
        const double t17 = pow(t14+t15,1.0/3.0);
        const double t18 = t17-t2*t8/(t17*3.0);
        return cv::Point2f(t18*pt.x/pt.y, t18);
    } else {
        const double t9 = t3/(t7*t7*4.0);
        const double t11 = t3/(t7*t7*t7*27.0);
        const std::complex<double> t12 = t9+t11;
        const std::complex<double> t13 = sqrt(t12);
        const double t14 = t2/t7;
        const double t15 = t14*pt.y*0.5;
        const std::complex<double> t16 = t13+t15;
        const std::complex<double> t17 = pow(t16,1.0/3.0);
        const std::complex<double> t18 = (t17+t14/(t17*3.0))*std::complex<double>(0.0,sqrt(3.0));
        const std::complex<double> t19 = -0.5*(t17+t18)+t14/(t17*6.0);
        return cv::Point2f(t19.real()*pt.x/pt.y, t19.real());
    }
}

cv::Point2f Imaging_Device::undistort_SFM(const cv::Point2f &distortedPixel)
{
    double k[5] = {0.0};
    double fx, fy, ifx, ify, cx, cy;
    int iters = 5;

    k[0] = this->distortion_coeffs.at<float>(0,0);
    k[1] = this->distortion_coeffs.at<float>(0,1);
    k[2] = this->distortion_coeffs.at<float>(0,2);
    k[3] = this->distortion_coeffs.at<float>(0,3);
    k[4] = 0;

    if(k[1] == -1 && k[2] == -1 && k[3] == -1){
        return UnDistort_Pixel_SFM(distortedPixel, k[0]);
    }

    fx = this->focal_lenght[0];
    fy = this->focal_lenght[1];
    ifx = 1.0/fx;
    ify = 1.0/fy;
    cx = this->intrinsics.at<float>(0,2);
    cy = this->intrinsics.at<float>(1,2);

    double x,y,x0,y0;

    x = distortedPixel.x;
    y = distortedPixel.y;

    x0 = x = (x - cx)*ifx;
    y0 = y = (y - cy)*ify;

    for(int jj = 0; jj < iters; jj++ )
    {
        double r2 = x*x + y*y;
        double icdist = 1./(1 + ((k[4]*r2 + k[1])*r2 + k[0])*r2);
        double deltaX = 2*k[2]*x*y + k[3]*(r2 + 2*x*x);
        double deltaY = k[2]*(r2 + 2*y*y) + 2*k[3]*x*y;
        x = (x0 - deltaX)*icdist;
        y = (y0 - deltaY)*icdist;
    }
    return cv::Point2f((float)(x*fx)+cx,(float)(y*fy)+cy);
}

////////////////////////////////


Ray Imaging_Device::get_ray(cv::Point2f input_pixel){

    cv::Point2f undistorted = this->undistort_Pixel(input_pixel);
    Ray ray;
    ray.origin = cv::Mat(4,1,CV_32FC1);
    ray.dir = cv::Mat(4,1,CV_32FC1);
    ray.origin = this->get_extrinsics() * (cv::Mat_<float>(4,1) << 0.0 , 0.0 , 0.0 , 1.0);
    ray.dir.at<float>(0,0) = (undistorted.x - intrinsics.at<float>(0,2)) / intrinsics.at<float>(0,0);
    ray.dir.at<float>(1,0) = (undistorted.y - intrinsics.at<float>(1,2)) / intrinsics.at<float>(1,1);
    ray.dir.at<float>(2,0) = 1.0f;
    ray.dir.at<float>(3,0) = 0.0f;
    ray.dir = this->get_extrinsics() * ray.dir;
    cv::normalize(ray.dir, ray.dir);
    return ray;
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

    translation.at<float>(0,0)= params_[TRANS_MAT].at<float>(0);
    translation.at<float>(1,0)= params_[TRANS_MAT].at<float>(1);
    translation.at<float>(2,0)= params_[TRANS_MAT].at<float>(2);


    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            rotation.at<float>(j,i) = params_[ROT_MAT].at<float>(j,i);

    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            intrinsics.at<float>(j,i) = params_[CAMERA_MAT].at<float>(j,i);

    for (int i = 0; i < 5; ++i)
        distortion_coeffs.at<float>(0,i) = params_[DISTOR_MAT].at<float>(i);

    return true;
}

void Imaging_Device::saveConfig(const std::__cxx11::string &configFile){

    cv::FileStorage fs("data/calibration/" + configFile, cv::FileStorage::WRITE);

    fs << "Image_Device" << "{:";
    fs<< "Calibrated" << true << "Matrix" << intrinsics << "Distortion" << distortion_coeffs <<"Translation"<< translation <<"Rotation"<< rotation;
    fs<<"Height" << resolution.height <<"Width" << resolution.width;
    fs<<"}";

    fs.release();
}

void Imaging_Device::print(){
    std::cout << "\nCamera Name :: " << this->name << std::endl;
    std::cout << "Intrinsics" << this->intrinsics << std::endl;
    std::cout << "Rotation" << this->rotation << std::endl;
    std::cout << "Translation" << this->translation << std::endl;
    std::cout << "Distortion_coeffs" << this->distortion_coeffs << std::endl;
    std::cout << "World_coordinates" << this->world_coordinates << std::endl;
    std::cout << " ======================================= " << std::endl;

}
