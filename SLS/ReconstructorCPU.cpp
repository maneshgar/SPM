#include <iomanip>
#include "ReconstructorCPU.h"
#include "fileReader.h"
#include <limits>
#include <opencv2/opencv.hpp>

namespace SLS{

ReconstructorCPU::~ReconstructorCPU()
{
    for (auto &cam: cameras_)
        delete cam;
    delete projector_;
}
void ReconstructorCPU::initBuckets()
{
    size_t x,y;
    projector_->getSize(x,y);
    buckets_.resize(cameras_.size());
    for (auto& b: buckets_)
        b.resize(x*y);
    generateBuckets();
}

void ReconstructorCPU::exportBuckets(std::string fileName){
    LOG::startTimer("Exporting matches to %s ... ", fileName.c_str());
    size_t x, y;
    cameras_[0]->getResolution(x,y);
    for (size_t cameraID=0; cameraID<buckets_.size(); cameraID++)
    {
        std::string name = "Outputs/" + fileName + std::to_string(cameraID) + ".data";
        std::ofstream of(name, std::ofstream::out);
        of << x << " \t " << y << std::endl;
        for (size_t PPixelID = 0;  PPixelID< buckets_[cameraID].size(); PPixelID++)
        {
            if(buckets_[cameraID][PPixelID].size() > 0)
            {
                of << buckets_[cameraID][PPixelID].size() << " \t ";
                for (int CamPixelID = 0; CamPixelID < buckets_[cameraID][PPixelID].size(); ++CamPixelID)
                {
                    of << buckets_[cameraID][PPixelID][CamPixelID] << " \t ";
                }
            }else{
                of << "-1";
            }

            of << std::endl;
        }
        of.close();
    }

    LOG::endTimer();
}

void ReconstructorCPU::addCamera(Camera *cam)
{
    cameras_.push_back(cam);
}

void ReconstructorCPU::generateBuckets()
{
    //Generating reconstruction bucket for each camera
    //for each camera
    for (size_t camIdx = 0; camIdx < cameras_.size(); camIdx++) {
        const auto &cam = cameras_[camIdx];
        LOG::writeLog("Generating reconstruction bucket for \"%s\" ... \n", cam->getName().c_str());

        cam->computeShadowsAndThresholds();

        size_t x = 0, y = 0, xTimesY = 0;
        cam->getResolution(x, y);
        xTimesY = x * y;

        // For each camera pixel
        //assert(dynamic_cast<FileReader*>(cam)->getNumFrames() == projector_->getRequiredNumFrames()*2+2);

        for (size_t i = 0; i < xTimesY; i++) {
            if (!cam->queryMask(i)) // No need to process if in shadow
                continue;

            cam->getNextFrame();
            cam->getNextFrame();//skip first two frames
            Dynamic_Bitset bits(projector_->getRequiredNumFrames());

            bool discard = false;
            // for each frame
            for (int bitIdx = projector_->getRequiredNumFrames() - 1; bitIdx >= 0; bitIdx--) {
                //for (int bitIdx = 0; bitIdx <projector_->getRequiredNumFrames(); bitIdx++) {

                //std::cout<<((FileReader*)cam)->getCurrentIdx()<<std::endl;
                auto frame = cam->getNextFrame();
                //std::cout<<((FileReader*)cam)->getCurrentIdx()<<std::endl;
                auto invFrame = cam->getNextFrame();
                unsigned char pixel = frame.at<uchar>(i % y, i / y);
                unsigned char invPixel = invFrame.at<uchar>(i % y, i / y);

                // Not considering shadow mask. But the following test should be
                // more strict than shadow mask.
                if (invPixel > pixel && invPixel - pixel >= ((FileReader*)cam)->getWhiteThreshold(i)) {
                    // No need to do anything since the array is initialized as all zeros
                    bits.clearBit((size_t) bitIdx);
                    continue;
                    //std::cout<<"-----\n"<<bits<<std::endl;
                    //bits.clearBit(bitIdx);
                    //std::cout<<std::setw(bits.size()-bitIdx)<<"c"<<std::endl;
                    //std::cout<<bits<<std::endl;

                }
                else if (pixel > invPixel && pixel - invPixel >
                         ((FileReader*)cam)->getWhiteThreshold(i)) {
                    bits.setBit((size_t )bitIdx);

                }
                else {
                    cam->clearMask(i);
                    discard = true;
                    continue;
                }
            } // end for each frame
            if (!discard) {

                auto vec2Idx = bits.to_uint_gray();
                if ( projector_->getWidth() > vec2Idx.x &&
                     vec2Idx.y < projector_->getHeight()) {
                    buckets_[camIdx][vec2Idx.x * projector_->getHeight() + vec2Idx.y].push_back(i);
                }
            }
        }
        unsigned  maxCount = 0;
        for (const auto &bucket: buckets_[camIdx]){
            if (bucket.size() > maxCount)
                maxCount = bucket.size();
        }
        std::cout<<"Max count = "<<maxCount<<std::endl;
    }

}

void ReconstructorCPU::reconstruct()
{
    size_t x,y;
    cameras_[0]->getResolution(x,y);
    initBuckets();
    exportBuckets("matches");
    int c1 = buckets_.size();
    LOG::startTimer();

    for ( size_t i=0; i<buckets_[0].size(); i++)     // 1024*768
    {
        //Progress
        float PointDistAll = 0;
        glm::vec4 PointAvgAll(0.0f);
        glm::vec4 ColorAvgAll(0.0f);
        float counterAll = 0;

        for (int camNumber0 = 0; camNumber0 < c1; ++camNumber0) {
            for (int camNumber1 = camNumber0+1; camNumber1 < c1; ++camNumber1) {

                //Progress
                const auto &cam0bucket = buckets_[camNumber0][i];
                const auto &cam1bucket = buckets_[camNumber1][i];

                if ((!cam0bucket.empty()) && (!cam1bucket.empty()))
                {
                    float ptCount = 0.0;
                    float midPointDist = 0;
                    glm::vec4 midPointAvg(0.0f);
                    glm::vec4 midColorAvg(0.0f);

                    for (const auto& cam0P: cam0bucket)
                    {
                        for (const auto& cam1P: cam1bucket)
                        {
                            float dist=-1.0f;

                            auto midP = midPointBkp(cameras_[camNumber0]->getRay(cam0P), cameras_[camNumber1]->getRay(cam1P), dist);
                            if (dist > 0.0) // if dist is valid
                            {

                                ptCount += 1.0;
                                midPointDist += dist;
                                midPointAvg += midP;
                                unsigned char r0, g0, b0;
                                unsigned char r1, g1, b1;
                                cameras_[camNumber0]->getColor(cam0P, r0, g0, b0);
                                cameras_[camNumber1]->getColor(cam1P, r1, g1, b1);
                                glm::vec4 tempColor((float)((r0+r1)/2) , (float)((g0+g1)/2) , (float)((b0+b1)/2), 1.0f );
                                midColorAvg += tempColor;
                            }
                        }
                    }

                    midPointAvg = midPointAvg/ptCount;
                    midColorAvg = midColorAvg/ptCount;
                    midPointDist = midPointDist/ptCount;

                    if(ptCount > 0){
                        PointDistAll = PointDistAll + midPointDist;
                        PointAvgAll = PointAvgAll + midPointAvg;
                        ColorAvgAll = ColorAvgAll + midColorAvg;
                        counterAll ++;
                        std::cout << "inside :: one point added    " << counterAll << std::endl;
                    }
                }
            }
        }
        if(counterAll > 0){
            PointAvgAll = PointAvgAll/counterAll;
            ColorAvgAll = ColorAvgAll/counterAll;
            PointDistAll = PointDistAll/counterAll;

            std::cout << "outside :: one point added    " << std::endl;

            pointCloud_.push_back(PointAvgAll.x);
            pointCloud_.push_back(PointAvgAll.y);
            pointCloud_.push_back(PointAvgAll.z);
            pointCloud_.push_back(1);

            colorCloud_.push_back(ColorAvgAll.x);
            colorCloud_.push_back(ColorAvgAll.y);
            colorCloud_.push_back(ColorAvgAll.z);
            colorCloud_.push_back(1);

            std::cout << "Distance :: " << PointDistAll << std::endl;
            pointCloudDist_.insert(std::make_pair(PointDistAll, i));
        }else{
            std::cout << "outside :: No points added    " << std::endl;
            pointCloud_.push_back(0);
            pointCloud_.push_back(0);
            pointCloud_.push_back(0);
            pointCloud_.push_back(1);

            colorCloud_.push_back(0);
            colorCloud_.push_back(0);
            colorCloud_.push_back(0);
            colorCloud_.push_back(1);

            pointCloudDist_.insert(std::make_pair(float(10000), i));

        }

        std::printf("===========\n");

        //        std::cout << std::endl;
    }
    LOG::endTimer("Finished reconstruction in ");
}

void ReconstructorCPU::calibrate_projector(){

    std::vector<std::vector<cv::Point3f>> all_Obj_points;
    std::vector<std::vector<cv::Point2f>> all_Img_points;

    std::vector<cv::Point3f> Obj_points;
    std::vector<cv::Point2f> Img_points;

    std::vector<cv::Mat> rvec, tvec;
    cv::Mat intr = cv::Mat::eye(3,3,CV_32FC1);
    intr.at<float>(0,0) = 3000;
    intr.at<float>(1,1) = 3000;
    intr.at<float>(0,2) = projector_->getHeight()/2;
    intr.at<float>(1,2) = projector_->getWidth()/2;

    cv::Mat dis;

    std::map<float, size_t>::iterator it = pointCloudDist_.begin();
    size_t pointID = 0;

    // i is row number
    // j is colomn number
    int i, j;

    for (uint itCount = 0; itCount < 5000; ++itCount) {
//        std::cout << itCount << " :: " << it->first;
        if (it->first < 0.00001) {
//            std::cout << " \tPassed! \n";
            pointID = it->second;
            i = pointID % projector_->getHeight();
            j = pointID / projector_->getHeight();
            cv::Vec3f temp;
            temp[0] = pointCloud_[pointID*4];
            temp[1] = pointCloud_[pointID*4 + 1 ];
            temp[2] = pointCloud_[pointID*4 + 2 ];

            cv::Point3f temppoint(temp[0], temp[1], temp[2]); //X , Y , Z
            Obj_points.push_back(temppoint);
            Img_points.push_back(cv::Point2f(i,j));
//            cv::waitKey(1);
        }else {
            std::cout << "Not expected distance! \tFailed! ****************** \n";
        }
        //GO TO NEXT POINTS
        it++;
    }

    all_Img_points.push_back(Img_points);
    all_Obj_points.push_back(Obj_points);
    cv::calibrateCamera(all_Obj_points, all_Img_points, cv::Size(projector_->getHeight(), projector_->getWidth()), intr, dis, rvec, tvec, CV_CALIB_USE_INTRINSIC_GUESS);

    std::string fileR ="X,Y\n";
    std::vector<cv::Point2f> imgp;

    //Compute the reprojection error
    cv::projectPoints(all_Obj_points[0], rvec[0], tvec[0], intr, dis, imgp);
    std::vector<float> reprojectionError;
    float errorX, errorY;
    for (int it = 0; it < all_Img_points[0].size(); ++it) {
        errorX = all_Img_points[0][it].x - imgp[it].x;
        errorY = all_Img_points[0][it].y - imgp[it].y;

        if(it%150 == 0)
            fileR += std::to_string(errorX) + "," + std::to_string(errorY) + "\n";
    }

    cv::Mat rotation;
    cv::Rodrigues(rvec[0], rotation);

    cv::FileStorage fstr("Outputs/Projector.xml", cv::FileStorage::WRITE);

    fstr << "Image_Device" << "{:";
    fstr << "Calibrated" << true << "Matrix" << intr << "Distortion" << dis <<"Translation"<< tvec[0] <<"Rotation"<< rotation;
    fstr <<"Height" << (int)projector_->getHeight() <<"Width" << (int)projector_->getWidth();
    fstr <<"}";

    std::cout << "Calibration Saved!\n";

    fstr.release();
    return;
}

} // namespace SLS
