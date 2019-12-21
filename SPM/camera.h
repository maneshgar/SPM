#ifndef CAMERA_H
#define CAMERA_H

#include <imaging_device.h>
#include <opencv2/opencv.hpp>

class Camera : public Imaging_Device
{
public:
    Camera();
    Camera(uint input_ID, std::string input_name);
    Camera(uint input_ID, std::string input_name, cv::Size resolution);
    ~Camera();

private:

};

#endif // CAMERA_H

