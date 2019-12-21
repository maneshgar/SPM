#include "feature_point.h"
#include "pattern.h"
Feature_Point::Feature_Point()
{

}

Feature_Point::Feature_Point(float realX, float realY, int pixelX, int pixelY, float verticalPhase, float horizontalPhase, float verticalError, float horizontalError){
    this->rX = realX;
    this->rY = realY;

    this->pX = pixelX;
    this->pY = pixelY;

    this->vPhase = verticalPhase;
    this->hPhase = horizontalPhase;

    this->vError = verticalError;
    this->hError = horizontalError;
}

cv::KeyPoint Feature_Point::get_cv_KeyPoint(){
    return cv::KeyPoint(pX , pY,10);
}

static bool featdisplay_feature_point_vector(std::vector<Feature_Point> features, cv::Size image_size){
    Pattern temp = Pattern(1,std::string("feature_points"), image_size, CV_8UC1);
    temp.image = cv::Scalar(0);
    std::vector<cv::KeyPoint> temp_vector;
    for (int var = 0; var < features.size(); ++var) {
        temp_vector.push_back(cv::KeyPoint(features[var].pX , features[var].pY,10));
    }
    cv::drawKeypoints(temp.image, temp_vector, temp.image);
    temp.display("features");
}

