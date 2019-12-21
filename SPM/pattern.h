#ifndef PATTERN_H
#define PATTERN_H

#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <string>

class Pattern
{
public:

    int ID = 0;
    int type;
    std::string name;
    cv::Mat image;
    cv::Size size;

    Pattern();
    Pattern(int input_id, std::string input_name);
    Pattern(int input_id, std::string input_name, cv::Mat input_image);
    Pattern(int input_id, std::string input_name, cv::Size image_size, int input_type);
    ~Pattern();

    bool display(std::string window_name, uint flag=cv::WINDOW_NORMAL);
//    Pattern operator -(Pattern &input_pattern);

};

#endif // PATTERN_H
