#ifndef PROJECTOR_H
#define PROJECTOR_H

#include <imaging_device.h>
#include <opencv2/opencv.hpp>

class Projector : public Imaging_Device
{
public:
    Projector();
    Projector(uint input_ID, std::string input_name);
    Projector(uint input_ID, std::string input_name, cv::Size resolution);
    ~Projector();

private:

};

#endif // PROJECTOR_H
