
#include <QCoreApplication>
#include <string>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/video/video.hpp>




using namespace cv;

//void PBRTFile(){

//    std::string data = "Sampler \"halton\" \"integer pixelsamples\" [128]\n"
//                       "PixelFilter \"mitchell\" \"float xwidth\" [2] \"float ywidth\" [2]\n\n"
//                       "# Camera setting\n"
//                       "LookAt 0.4450886  0.47929537  -8.5175648    0.745347 -0.41774 7.84536   0.021984229 0.99702835 -0.073831968 \n"
//                       "Film \"image\" \"string filename\" [\"/home/behnam/MyDrive/Workstation/Qt/build-Compensation-Desktop_Qt_5_6_0_GCC_64bit-Debug/rendered_img.png\"]\n"
//                       "\"integer xresolution\" [1920] \"integer yresolution\" [1200]\n"
//                       "WorldBegin\n"
//                       "\t#projector setting\n"
//                       "\tAttributeBegin\n"
//                       "\t\tCoordSysTransform \"camera\"\n"
//                       "\t\tLightSource \"projection\" \"rgb I\" [1500.0 1500.0 1500.0] \"float fov\" [10.5] \"string mapname\" [\"/home/behnam/MyDrive/Workstation/Qt/build-Compensation-Desktop_Qt_5_6_0_GCC_64bit-Debug/imgSeq/" + std::to_string(i) + ".png\"]\n"
//                       "\tAttributeEnd\n"
//                       "\t#scene setting\n"
//                       "\tAttributeBegin\n"
//                       "\t\tTexture \"mydiffuse\" \"spectrum\" \"imagemap\" \"string filename\" \"diffuse.png\"\n"
//                       "\t\tMaterial \"matte\" \"texture Kd\" \"mydiffuse\"\n"
//                       "\t\tShape \"plymesh\" \"string filename\" [\"mesh_bilateral.ply\"]\n"
//                       "\tAttributeEnd\n"
//                       "WorldEnd\n";
//    std::cout << data;

//    std::ofstream myfile;
//    std::string path = "render.pbrt";
//    std::cout << "writing into :: " << path << std:: endl;
//    myfile.open(path);
//    myfile << data;
//    myfile.close();
//    return;
//}

void saveInOneFolder(std::string src_path, std::string dst_path, std::string file_name,  std::string folder_name, int start_frame, int end_frame){

    Mat mask = imread("./mask1080.png")/255;
    system(("mkdir ./Videos/" + folder_name).c_str());
    Mat image;
    for (int iter = start_frame; iter < end_frame; ++iter) {
        std::string path = src_path + std::to_string(iter) + "/" + file_name;
        image = imread(path);
        image = image.mul(mask);
//        imwrite(dst_path + folder_name + "/" + std::to_string(iter-start_frame+1) + ".jpg", image);
        imwrite(dst_path + folder_name + "/" + std::to_string(iter-start_frame+283) + ".jpg", image);
    }
    return;
}

void videoToImage(const char* filename, const char* fileDestination){
    CvCapture* capture=0;
    IplImage* frame=0;

    capture = cvCaptureFromFile(filename); // read AVI video
    if( !capture )
        throw "Error when reading steam_avi";

    int i =0;
    for( ; ; )
    {
        frame = cvQueryFrame( capture );
        if(!frame)
            break;
        string path = fileDestination + std::to_string(i) + ".jpg";
        cvSaveImage(path.c_str(), frame);
        Mat img = imread(path);
        Mat cropped = img(Range(0, img.rows), Range((img.cols - 1728)/2 , 1728 + (img.cols - 1728)/2) );
//        Mat cropped = img.clone();
        imwrite(path, cropped);
        std::cout << "Saved to path :: " << path << std::endl;
        i++;
    }

    cvReleaseImage(&frame);
    return;
}

void compensation(int start_frame, int end_frame){
    std::string renderingPath = "Rendering/";

    int system_output;

    for (int frameId = start_frame; frameId < end_frame; ++frameId) {

        //read expected Image
        std::string expectedPath = "imgSeq/" + std::to_string(frameId) + ".jpg";
        Mat Expected_img = imread(expectedPath);
        Expected_img.convertTo(Expected_img, CV_32F);
        Expected_img /= 255.0;

        //read Projection Image
        std::string projectionPath = "imgSeq/" + std::to_string(frameId) + ".jpg";
        Mat Projected_img = imread(projectionPath);
        Projected_img.convertTo(Projected_img, CV_32F);
        Projected_img /= 255.0;

        Mat Update_img = Mat::zeros(Expected_img.size(), CV_32FC3);
        Mat Rendered_img = Mat::zeros(Expected_img.size(), CV_32FC3);
        Mat mask(Projected_img.size(),CV_32FC3, Scalar(1.0, 1.0, 1.0));
        Mat mask_counter(Projected_img.size(),CV_32FC1, Scalar(0.0));

        //create the folder

        system_output = system(("rm -r /home/behnam/MyDrive/Workstation/Qt/build-Compensation-Desktop_Qt_5_6_0_GCC_64bit-Release/Outputs/" + std::to_string(frameId)).c_str());
        system_output = system(("mkdir /home/behnam/MyDrive/Workstation/Qt/build-Compensation-Desktop_Qt_5_6_0_GCC_64bit-Release/Outputs/" + std::to_string(frameId)).c_str());

        //copy Projection file to right location
        imwrite(renderingPath + "projection.png", Projected_img * 255);

        //Iterate to compensate
        for (int Iterations = 0; Iterations < 30; ++Iterations) {
            std::cout << "Rendering :: \nframe :: " << frameId << "\nIteration :: " << Iterations << std::endl;
            //Run PBRT ( RENDER )
            string scene_path ="/home/behnam/MyDrive/Workstation/Qt/build-Compensation-Desktop_Qt_5_6_0_GCC_64bit-Release/Rendering/Render.pbrt";
            std::cout << "Rendering started!!\n";
            system_output = system(("pbrt " + scene_path).c_str());
            std::cout << "Rendering is Done!!\n";

            //check the difference
            Rendered_img = imread(renderingPath + "rendered_img.png");
            Rendered_img.convertTo(Rendered_img, CV_32F);
            Rendered_img /= 255.0;

            flip(Rendered_img, Rendered_img, 0);

            Update_img = Expected_img - Rendered_img;

            std::string rootPath = "Outputs/" + std::to_string(frameId) + "/";
            imwrite(rootPath + "Rendered"+std::to_string(Iterations)+".jpg", Rendered_img * 255.0);
            imwrite(rootPath + "Update_img"+std::to_string(Iterations)+".jpg", (Update_img /*+Scalar(0.5,0.5,0.5)*/ )* 255.0);
            imwrite(rootPath + "MinusUpdate_img"+std::to_string(Iterations)+".jpg", (-Update_img /*+Scalar(0.5,0.5,0.5)*/ )* 255.0);
            imwrite(rootPath + "Expected_img"+std::to_string(Iterations)+".jpg", Expected_img * 255.0);
            imwrite(rootPath + "Projected_img"+std::to_string(Iterations)+".jpg", Projected_img * 255.0);


            std::cout << "Rendering " << Iterations << std::endl;
            std::cout << "Expected_img \n" << Expected_img.at<Vec3f>(600,870) << std::endl;
            std::cout << "Rendered_img \n" << Rendered_img.at<Vec3f>(600,870) << std::endl;
            std::cout << "Update_img \n" << Update_img.at<Vec3f>(600,870) << std::endl;
            std::cout << "Projected_img \n" << Projected_img.at<Vec3f>(600,870)<< std::endl;

            Update_img /= 25.0;
            Update_img = Update_img.mul(mask);
            Projected_img += Update_img;

            bool triged = false;
            float DIF = 0;
            for (int rowIt = 0; rowIt < Projected_img.rows; ++rowIt) {
                for (int colIt = 0; colIt < Projected_img.cols; ++colIt) {
                    DIF = 0;
                    triged = false;
                    Vec3f temp = Projected_img.at<Vec3f>(rowIt, colIt);

                    if (temp[0] > 1.0 ) {
                        DIF = temp[0] - 1.0;
                        temp -= Vec3f(DIF ,DIF, DIF);
                        triged = true;
//                        std::cout << "1\n";
                    } else if(temp[1] > 1.0){
                        DIF = temp[1] - 1.0;
                        temp -= Vec3f(DIF ,DIF, DIF);
//                        std::cout << "2\n";
                        triged = true;
                    } else if(temp[2] > 1.0){
                        DIF = temp[2] - 1.0;
                        temp -= Vec3f(DIF ,DIF, DIF);
//                        std::cout << "3\n";
                        triged = true;
                    }else if(temp[0] < 0.1 ){
                        DIF = -temp[0];
                        temp += Vec3f(DIF ,DIF, DIF);
//                        std::cout << "4\n";
                        triged = true;
                    } else if(temp[1] < 0.1){
                        DIF = -temp[1];
                        temp += Vec3f(DIF ,DIF, DIF);
//                        std::cout << "5\n";
                        triged = true;
                    } else if(temp[2] < 0.1){
                        DIF = -temp[2];
                        temp += Vec3f(DIF ,DIF, DIF);
//                        std::cout << "6\n";
                        triged = true;
                    }
                    Projected_img.at<Vec3f>(rowIt, colIt) = temp;
                    if(triged){
                        mask_counter.at<float>(rowIt, colIt)++;
                    }
                    if (mask_counter.at<float>(rowIt, colIt) > 3) {
                        mask.at<Vec3f>(rowIt, colIt) = Vec3f(0.0, 0.0, 0.0);
                    }
                }
            }

            std::cout << "Next Projected_img \n" << Projected_img.at<Vec3f>(39,667)<< std::endl;

            imwrite(renderingPath + "projection.png", Projected_img*255.0);
            imwrite(rootPath + "mask"+std::to_string(Iterations)+".png", mask*255.0);
            imwrite(rootPath + "projection-next"+std::to_string(Iterations)+".png", Projected_img*255.0);

        }
    }
}

//void Create_video (std::string path)
//{
//    Mat img_sample = imread(path+"1.jpg");
//    VideoCapture in_capture(path + "%d.jpg");
//    Mat img;

//    VideoWriter out_capture("out_video.avi", CV_FOURCC('M','J','P','G'), 30, img_sample.size);

//    while (true)
//    {
//        in_capture >> img;
//        if(img.empty())
//            break;

//        out_capture.write(img);
//    }
//}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

//    Mat img1 = imread("/home/behnam/MyDrive/SPM-Database/IMportant/April 2nd- 8floor/Compensate/city/Expected_img0.jpg");
//    Mat img2 = imread("/home/behnam/MyDrive/SPM-Database/IMportant/April 2nd- 8floor/Compensate/city/projection-next14.png");

//    Mat dif = img2 - img1;
//    imshow("dif", dif);
//    imwrite("Update-Map.png", dif);
//    waitKey(0);

// 1728 * 1080


    int start_frame = 100;
    int mid_frame = 382;
    int end_frame = 493;

//    convert video to images
//    videoToImage("video.avi", "imgSeq/");



    //Compensate
//    compensation(start_frame, end_frame);

    //Create Video
//    Create_video("Videos/Videos5/");


    //    save all files in one folder
            for (int iter = 0; iter < 1; ++iter) {
                saveInOneFolder("Outputs/", "Videos/", "Projected_img" + std::to_string(iter) + ".jpg", "Videos" + std::to_string(iter), /*start_frame,*/ mid_frame, end_frame);
            }

    std::cout << "Program Ended!\n";
    return a.exec();
}
