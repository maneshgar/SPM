/* Copyright (C) 
 * 2016 - Tsing Gu
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 * 
 */
#pragma once
#include "Camera.h"
namespace SLS
{
class FileReader: public Camera
{
protected:
     enum CAMERA_MAT{
         CAMERA_MAT=0,
         DISTOR_MAT,
         ROT_MAT,
         TRANS_MAT,
         PARAM_COUNT
     };
    std::vector<cv::Mat> images_;
    size_t frameIdx_;
    std::array<cv::Mat, PARAM_COUNT> params_;
    glm::mat4 camTransMat_; //Transformation matrix for camera
    std::vector<Ray> rayTable;
    glm::vec2 undistortPixel(const glm::vec2 &distortedPixel) const;
    glm::vec2 undistortPixel(const size_t &distortedIdx) const
    {
        return undistortPixel(glm::vec2( distortedIdx/resY_, distortedIdx % resY_));
    }
    
public:
    //Constructors
    FileReader() = delete;
    FileReader(const std::string& cName):
        Camera(cName),frameIdx_(0),camTransMat_(glm::mat4(1.0)){}


    //Extra functions
    void loadImages(const std::string& folder, bool isGL=false);
    void previousFrame() {frameIdx_=frameIdx_==0?frameIdx_:frameIdx_-1;}
    const cv::Mat& getCurrentFrame() const {return images_[frameIdx_];}
    size_t getNumFrames() const { return images_.size(); }
    size_t getCurrentIdx() const {return frameIdx_;}
    const std::array<cv::Mat, PARAM_COUNT>& getParams()const{return params_;}

    /**
     * @brief visualize raytable with point cloud, each ray is a unit vector
     *
     * @param fileName output obj file
     */
    void rayTableToPointCloud(std::string fileName) const;

    //Implementing interfaces
    ~FileReader() override{
       for (auto image: images_)
           image.release();
        for (auto param: params_)
            param.release();
    }
    Ray getRay(const size_t &x, const size_t &y) override;
    //Ray getRay(const size_t &idx) override{return rayTable[idx];}
    Ray getRay(const size_t &pixelIdx) override;

    void loadConfig(const std::string& configFile) override;
    void loadConfig(
            const std::string& distMat,
            const std::string& camMat,
            const std::string& transMat,
            const std::string& rotMat
            );
    const cv::Mat& getNextFrame() override;
    void undistort() override;
    void computeShadowsAndThresholds() override;
    void setResolution (const size_t &x, const size_t &y) override {resX_ = x; resY_ = y; rayTable.resize(resX_*resY_);}
    unsigned char getWhiteThreshold(size_t i) const { return thresholds_[i];}
};
}
