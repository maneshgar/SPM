#include <QCoreApplication>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <DataInterface.h>
#include "imaging_device.h"

using namespace cv;

bool LoadNVM(std::ifstream& in, std::vector<CameraT>& camera_data, std::vector<Point3D>& point_data,
             std::vector<Point2D>& measurements, std::vector<int>& ptidx, std::vector<int>& camidx,
             std::vector<string>& names, std::vector<int>& ptc)
{
    int rotation_parameter_num = 4;
    bool format_r9t = false;
    string token;
    std::cout << "READ :: " << in.peek() << std::endl;
    if(in.peek() == 'N')
    {
        in >> token; //file header
        if(strstr(token.c_str(), "R9T"))
        {
            rotation_parameter_num = 9;    //rotation as 3x3 matrix
            format_r9t = true;
        }
    }

    int ncam = 0, npoint = 0, nproj = 0;
    // read # of cameras
    in >> ncam;  if(ncam <= 1) return false;

    //read the camera parameters
    camera_data.resize(ncam); // allocate the camera data
    names.resize(ncam);
    for(int i = 0; i < ncam; ++i)
    {
        double f, q[9], c[3], d[2];
        in >> token >> f ;
        for(int j = 0; j < rotation_parameter_num; ++j) in >> q[j];
        in >> c[0] >> c[1] >> c[2] >> d[0] >> d[1];

        camera_data[i].SetFocalLength(f);
        if(format_r9t)
        {
            camera_data[i].SetMatrixRotation(q);
            camera_data[i].SetTranslation(c);
        }
        else
        {
            //older format for compability
            camera_data[i].SetQuaternionRotation(q);        //quaternion from the file
            camera_data[i].SetCameraCenterAfterRotation(c); //camera center from the file
        }
        camera_data[i].SetNormalizedMeasurementDistortion(d[0]);
        names[i] = token;
    }

    //////////////////////////////////////
    in >> npoint;   if(npoint <= 0) return false;

    //read image projections and 3D points.
    point_data.resize(npoint);
    for(int i = 0; i < npoint; ++i)
    {
        float pt[3]; int cc[3], npj;
        in  >> pt[0] >> pt[1] >> pt[2]
                >> cc[0] >> cc[1] >> cc[2] >> npj;
        for(int j = 0; j < npj; ++j)
        {
            int cidx, fidx; float imx, imy;
            in >> cidx >> fidx >> imx >> imy;

            camidx.push_back(cidx);    //camera index
            ptidx.push_back(i);        //point index

            //add a measurment to the vector
            measurements.push_back(Point2D(imx, imy));
            nproj ++;
        }
        point_data[i].SetPoint(pt);
        ptc.insert(ptc.end(), cc, cc + 3);
    }
    ///////////////////////////////////////////////////////////////////////////////
    std::cout << ncam << " cameras; " << npoint << " 3D points; " << nproj << " projections\n";

    return true;
}

bool loadImage(std::string &filePath, Mat &img){

    img = imread(filePath);
    if (!img.data) {
        std::cout << "Could not open Image or the image does not exist :: " << filePath << std::endl;
        return false;
    }
    return true;
}

bool saveImage(std::string &filePath, Mat &img){
    bool wrote = imwrite(filePath, img);
    if(!wrote){
        std::cout << "Could not write Image :: " << filePath << std::endl;
    }
}

void compute_world_coordinates(CameraT camera){
    float data[4] = {0.0, 0.0, 0.0, 1.0};
    cv::Mat point(4,1, CV_32FC1, &data);
    cv::Mat tempT = cv::Mat::eye(4,4,CV_32FC1);
    cv::Mat tempR = cv::Mat::eye(4,4,CV_32FC1);

    tempR.at<float>(0,0) = camera.m[0][0];
    tempR.at<float>(0,1) = camera.m[0][1];
    tempR.at<float>(0,2) = camera.m[0][2];

    tempR.at<float>(1,0) = camera.m[1][0];
    tempR.at<float>(1,1) = camera.m[1][1];
    tempR.at<float>(1,2) = camera.m[1][2];

    tempR.at<float>(2,0) = camera.m[2][0];
    tempR.at<float>(2,1) = camera.m[2][1];
    tempR.at<float>(2,2) = camera.m[2][2];

    tempT.at<float>(0,3) = -camera.t[0];
    tempT.at<float>(1,3) = -camera.t[1];
    tempT.at<float>(2,3) = -camera.t[2];

    Mat world_coordinates = (tempR.t() * tempT) * point;
    std::cout << world_coordinates << std::endl;
    return;
}

bool cameraTToOpenCV(CameraT cam, std::string camName, int camID){

    Imaging_Device imdc(camID, camName, cv::Size(1920, 1200));

    imdc.rotation.at<float>(0,0) = cam.m[0][0];
    imdc.rotation.at<float>(0,1) = cam.m[0][1];
    imdc.rotation.at<float>(0,2) = cam.m[0][2];
    imdc.rotation.at<float>(1,0) = cam.m[1][0];
    imdc.rotation.at<float>(1,1) = cam.m[1][1];
    imdc.rotation.at<float>(1,2) = cam.m[1][2];
    imdc.rotation.at<float>(2,0) = cam.m[2][0];
    imdc.rotation.at<float>(2,1) = cam.m[2][1];
    imdc.rotation.at<float>(2,2) = cam.m[2][2];

    imdc.translation.at<float>(0,0) = cam.t[0];
    imdc.translation.at<float>(1,0) = cam.t[1];
    imdc.translation.at<float>(2,0) = cam.t[2];

    imdc.distortion_coeffs.at<float>(0,0) = cam.radial;
    imdc.distortion_coeffs.at<float>(0,1) = -1;
    imdc.distortion_coeffs.at<float>(0,2) = -1;
    imdc.distortion_coeffs.at<float>(0,3) = -1;
    imdc.distortion_coeffs.at<float>(0,4) = -1;

    imdc.intrinsics.at<float>(0,0) = cam.GetFocalLength();
    imdc.intrinsics.at<float>(1,1) = cam.GetFocalLength();
    imdc.intrinsics.at<float>(0,2) = imdc.getWidth()/2;
    imdc.intrinsics.at<float>(1,2) = imdc.getHeight()/2;

    imdc.saveConfig(camName + ".xml");
}

int main(int argc, char *argv[])
{
    std::string filePath = "/home/behnam/MyDrive/Workstation/Qt/build-swip-Desktop_Qt_5_6_0_GCC_64bit-Debug/scene.nvm";
    std::ifstream in(filePath);
    std::vector<CameraT> camera_data;
    std::vector<Point3D> point_data;
    std::vector<Point2D> measurements;
    std::vector<int> ptidx;
    std::vector<int> camidx;
    std::vector<string> names;
    std::vector<int> ptc;

    LoadNVM(in, camera_data, point_data, measurements, ptidx, camidx, names, ptc);

    for (int it = 0; it < camera_data.size(); ++it) {
        //        std::cout << camera_data[it].t[0] << "\t" << camera_data[it].t[1] << "\t"  << camera_data[it].t[2] << "\t" << std::endl;

        //        std::cout << names[it] << std::endl;
        cameraTToOpenCV(camera_data[it],names[it], it);
        if(names[it] == "1.jpg" || names[it] == "2.jpg" || names[it] == "3.jpg"){
            std::cout << names[it] << std::endl;
            CameraT t;
            float trans[3];
            float cent[3];
            float r[9];

            camera_data[it].GetMatrixRotation(r);
            camera_data[it].GetTranslation(trans);
            camera_data[it].GetCameraCenter(cent);

            std::printf("rotation %f16 %f %f %f %f %f %f %f %f\n", r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]);
            std::printf("world coordinates %f %f %f\n", cent[0], cent[1], cent[2]);
            std::printf("translation %f %f %f\n", trans[0], trans[1], trans[2]);
            std::printf("distortion type %i \n", camera_data[it].distortion_type);
            std::cout << "distortion number " << camera_data[it].GetNormalizedMeasurementDistortion() << std::endl;
            compute_world_coordinates(camera_data[it]);

        }
    }

    std::cout << "Program End Successfully!\n";
    //    return a.exec();
}
