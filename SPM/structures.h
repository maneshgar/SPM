#ifndef STRUCTURES_H
#define STRUCTURES_H

#include <opencv2/opencv.hpp>
#include "pattern.h"
#include "pattern_sequence.h"
#include "feature_point.h"
#include "Image/Color.h"

struct Phase_Pattern_Seq{
    char orientation;
    cv::Size pattern_size;
    Pattern phaseMap;
    Pattern_Sequence patterns;
    std::vector<Feature_Point> featurePoint_indicator;
};

struct Ray
{
    cv::Mat origin;
    cv::Mat dir;
};


struct Match{
    std::vector<cv::Point2f> points;
    std::vector<Color> Colors;
    Color mean_Color;
    cv::Point2f meanPoint;

    std::vector<cv::Point2f> left_points;
    std::vector<cv::Point2f> right_points;
    std::vector<Color> left_captured_Colors;
    std::vector<Color> right_captured_Colors;
    Color left_mean_Color;
    Color right_mean_Color;
    cv::Point2f left_mean_point;
    cv::Point2f right_mean_point;

    bool should_count;
};

struct Phase_Map_Point{
    int row;
    int col;
    int k;
    float phase_amount;
};

#endif // STRUCTURES_H
