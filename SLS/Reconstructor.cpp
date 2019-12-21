#include "Reconstructor.h"
#include <fstream>

#include<opencv2/opencv.hpp>

namespace SLS
{
class Reconstructor;

void exportPLYGrid(std::string fileName, const Reconstructor& reconstructor)
{
    LOG::startTimer("Exporting PLY Grid to %s ... ", fileName.c_str());
    std::ofstream of(fileName, std::ofstream::out);
    const auto &pointCloud = reconstructor.pointCloud_;
    //Writing headers
    of<<"ply\n";
    of<<"format ascii 1.0\n";
    of<<"obj_info is_interlaced 0\n";
    of<<"obj_info num_cols " << reconstructor.projector_->getHeight() << "\n";
    of<<"obj_info num_rows " << reconstructor.projector_->getWidth() << "\n";
    of<<"element vertex " << pointCloud.size()/4 << "\n";
    of<<"property float x\n";
    of<<"property float y\n";
    of<<"property float z\n";
    of<<"element range_grid " << reconstructor.projector_->getHeight() *
        reconstructor.projector_->getWidth()<< "\n";
    LOG::endTimer();
    of<<"property list uchar int vertex_indices\n";
    of<<"end_header\n";
    for (size_t i=0; i<pointCloud.size(); i+=4)
    {
        of<<pointCloud[i+0]<<" "<<pointCloud[i+1]<<" "<<pointCloud[i+2]<<std::endl;
    }
    std::cout<<"PointCloud size: "<<pointCloud.size()<<std::endl;
    std::vector<bool> ranged_grid(reconstructor.projector_->getHeight()*reconstructor.projector_->getWidth(), false);
    for (size_t i=0; i<pointCloud.size(); i+=4)
    {
        unsigned clmBasedIdx = i/4;
        unsigned x = clmBasedIdx / reconstructor.projector_->getHeight();
        unsigned y = clmBasedIdx % reconstructor.projector_->getHeight();
        unsigned rowBasedIdx = x+y*reconstructor.projector_->getWidth();
        if (pointCloud[i+3] != 0)
            ranged_grid[rowBasedIdx] = true;
        else
            ranged_grid[rowBasedIdx] = false;
    }
    for (size_t i=0; i<ranged_grid.size(); i++)
        if (ranged_grid[i])
            of << "1 "<<i<<std::endl;
        else
            of<<"0"<<std::endl;
}
void exportPLY( std::string fileName ,const Reconstructor& reconstructor)
{
    LOG::startTimer("Exporting PLY to %s ... ", fileName.c_str());

    std::ofstream of(fileName, std::ofstream::out);
    const auto &pointCloud = reconstructor.pointCloud_;
    const auto &colorCloud = reconstructor.colorCloud_;
    // Writing Headers
    of <<"ply\n"<<
         "format ascii 1.0\n"<<
         "element vertex "<<pointCloud.size()/4<<std::endl<<
         "property float x\n"<<
         "property float y\n"<<
         "property float z\n"<<
         "property uchar red\n"<<
         "property uchar geen\n"<<
         "property uchar blue\n"<<
         "end_header\n";
    // Writing vertex list
    for (size_t i=0; i<pointCloud.size(); i+=4){
//        if(!(pointCloud[i] == 0 && pointCloud [i+1] == 0 && pointCloud[i+2] == 0)){
            of << std::fixed << pointCloud[i+0] << " " << pointCloud[i+1] << " " << pointCloud[i+2] << " " << (int)colorCloud[i+0] << " " << (int)colorCloud[i+1] << " " << (int)colorCloud[i+2] << std::endl;
//        }
    }
    of.close();
    // Writing face list

    LOG::endTimer();
}

void exportOBJ( std::string fileName ,const Reconstructor& reconstructor)
{
    LOG::startTimer("Exporting OBJ to %s ... ", fileName.c_str());

    cv::Mat msk(reconstructor.projector_->getHeight(), reconstructor.projector_->getWidth(), CV_32FC1, cv::Scalar(0.0));

    std::ofstream of(fileName, std::ofstream::out);

    const auto &pointCloud = reconstructor.pointCloud_;
    for (size_t i=0; i<pointCloud.size(); i+=4)
    {
        of<< "v " << std::fixed << pointCloud[i+0]<<" "<<pointCloud[i+1]<<" "<<pointCloud[i+2]<<std::endl;
        int row = (i/4)%(reconstructor.projector_->getHeight());
        int col = (i/4)/(reconstructor.projector_->getHeight());

        if(!(pointCloud[i+0] == 0 && pointCloud[i+1] == 0 && pointCloud[i+2] == 0)){
            msk.at<float>(row, col) = 1;
        }else{
            msk.at<float>(row, col) = 0;
        }
    }
    of.close();

    cv::namedWindow("Mask Image", cv::WINDOW_NORMAL);
    cv::imshow("Mask Image", msk);
    cv::imwrite("Mask.jpg", msk * 255);
    cv::waitKey(0);

    LOG::endTimer();
}

void exportOBJVec4( std::string fileName ,const Reconstructor& reconstructor)
{
    LOG::startTimer("Exporting OBJ vec4 to %s ... ", fileName.c_str());

    std::ofstream of(fileName, std::ofstream::out);
    const auto &pointCloud = reconstructor.pointCloud_;
    for (size_t i=0; i<pointCloud.size(); i+=4)
    {
        if (pointCloud[i+3] < 0.5 ) continue;
        of<<"v " << std::fixed <<pointCloud[i]<<" "<<pointCloud[i+1]<<" "<<pointCloud[i+2]<<std::endl;
    }
    of.close();

    LOG::endTimer();
}
}
