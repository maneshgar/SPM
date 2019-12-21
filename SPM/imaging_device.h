#ifndef IMAGING_DEVICE_H
#define IMAGING_DEVICE_H

#include <opencv2/opencv.hpp>
#include <string>

#include "file_manager.h"
#include "structures.h"

class Imaging_Device
{

public:

    int ID;
    std::string name;
    cv::Mat intrinsics;

    cv::Size resolution;
    cv::Mat distortion_coeffs;
    cv::Mat translation;
    cv::Mat rotation;
    cv::Mat world_coordinates;

    Imaging_Device();
    Imaging_Device(const Imaging_Device &d);

    Imaging_Device(int input_ID, std::string input_name);
    Imaging_Device(int input_ID, std::string input_name, cv::Size resolution);
    ~Imaging_Device();

    void print();

    void get_RT_with_another_device(Imaging_Device device, cv::Mat &rotation, cv::Mat &translation);

    void compute_world_coordinates();
    void compute_world_coordinates(cv::Mat &point);

    void set_world_coordinates(cv::Mat data);
    cv::Mat get_world_coordinates();

    Ray get_ray(cv::Point2f input_pixel);
    cv::Mat get_extrinsics();
    bool save_device();
    bool load_device();
    bool loadConfig(const std::string& configFile);
    void saveConfig(const std::string& configFile);

    cv::Size getResolution(){
        return this->resolution;
    }

    int getWidth(){
        return this->resolution.width;
    }
    int getHeight(){
        return this->resolution.height;
    }

    cv::Mat getRotationMatrix();
    cv::Mat getTranslationMatrix();
    cv::Mat getdistortion_coeffsMatrix();


    int getID();
    bool set_image(cv::Mat &data){
        this->image = data.clone();
        return true;
    }

    cv::Mat get_image(){
        return this->image.clone();
    }
    cv::Vec2f focal_lenght;

    cv::Point2f undistort_SFM(const cv::Point2f &distortedPixel);

private:

    cv::Mat image;
    cv::Vec2f sensor_center;
    cv::Point2f undistort_Pixel(cv::Point2f &distortedPixel);

};

#endif // IMAGING_DEVICE_H
