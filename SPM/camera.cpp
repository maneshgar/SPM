#include "camera.h"

Camera::Camera(uint input_ID, std::string input_name) : Imaging_Device(input_ID, input_name)
{

}

Camera::Camera(uint input_ID, std::string input_name, cv::Size resolution) : Imaging_Device(input_ID, input_name, resolution)
{
}

Camera::Camera(){

}

Camera::~Camera(){

}


