#ifndef FEATURE_POINT_H
#define FEATURE_POINT_H

#include <opencv2/opencv.hpp>

class Feature_Point
{
public:

    int ID;
    float rX, rY;
    int pX, pY;
    float vPhase, hPhase;
    float vError, hError;

    Feature_Point();
    Feature_Point(int IDNumber);
    Feature_Point(float rX, float rY, int pX, int pY, float vPhase, float hPhase, float vError, float hError);

    cv::KeyPoint get_cv_KeyPoint();
    static bool display_feature_point_vector(std::vector<Feature_Point>);

};

#endif // FEATURE_POINT_H
