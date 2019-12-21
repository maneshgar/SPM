#include "pattern.h"

Pattern::Pattern(){

}

Pattern::Pattern(int input_id, std::string input_name, cv::Mat input_image)
{
    this->name = input_name;
    this->ID = input_id;
    this->image = input_image.clone();
    this->size = image.size();
    this->type = image.type();
}

Pattern::Pattern(int input_id, std::string input_name, cv::Size image_size, int input_type)
{
    this->name = input_name;
    this->ID = input_id;
    this->size = image_size;
    this->type = input_type;
    this->image = cv::Mat(image_size, input_type);
}

Pattern::Pattern(int input_id, std::string input_name)
{
    this->ID = input_id;
    this->name = input_name;
}

Pattern::~Pattern(){
}

bool Pattern::display(std::string window_name, uint flag)
{
    cv::namedWindow(window_name, flag);
    cv::imshow(window_name, image);
    cv::waitKey(0);
    return false;
}
