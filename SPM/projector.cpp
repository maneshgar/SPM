#include "projector.h"

Projector::Projector()
{

}

Projector::Projector(uint input_ID, std::string input_name)  : Imaging_Device(input_ID, input_name)
{

}

Projector::Projector(uint input_ID, std::string input_name, cv::Size resolution) : Imaging_Device(input_ID, input_name, resolution)
{
}

Projector::~Projector(){

}
