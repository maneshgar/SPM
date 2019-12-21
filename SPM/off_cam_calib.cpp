#include "off_cam_calib.h"

//OFF_Cam_Calib::OFF_Cam_Calib(){

//}

OFF_Cam_Calib::OFF_Cam_Calib(PhaseShifting_Pattern_encoder &input_phaseShifting_pattern_encoder, Camera &input_camera):myCamera(input_camera)
{
    myCamera.distortion_coeffs = input_camera.distortion_coeffs.clone();

    phs_pattern_encoder = input_phaseShifting_pattern_encoder;
}

OFF_Cam_Calib::~OFF_Cam_Calib(){

}

bool OFF_Cam_Calib::calibrate(){
    std::cout << "Starting Calibration of Camera :: " << myCamera.name << std::endl;

    File_Manager file_manager("data/calibration/");
    std::vector<std::string> files;
    file_manager.get_dir_list(files, (myCamera.name+"/image_set/"));
    image_set_size = files.size();
    camera_decoded_poses.resize(image_set_size);

    std::thread *threads = new std::thread[image_set_size];

    //        for (int var = 0; var < image_set_size; ++var) {
    //                std::string name = myCamera.name+"/image_set/";
    //                name = name+files[var];
    //                std::cout << std::stoi(files[var]) << std::endl;
    //                proccess_ImageSet(std::ref(camera_decoded_poses[std::stoi(files[var])]), files[var], name, this->phs_pattern_encoder.vertical_PatSeq.featurePoint_indicator, this->phs_pattern_encoder.horizontal_PatSeq.featurePoint_indicator, TILE_SIZE_V, TILE_SIZE_H);
    //        }

    //        int var = 0;
    //        std::string name = myCamera.name+"/image_set/";
    //        name = name+files[var];
    //        std::cout << std::stoi(files[var]) << std::endl;
    //        proccess_ImageSet(std::ref(camera_decoded_poses[std::stoi(files[var])]), files[var], name, this->phs_pattern_encoder.vertical_PatSeq.featurePoint_indicator, this->phs_pattern_encoder.horizontal_PatSeq.featurePoint_indicator, TILE_SIZE_V, TILE_SIZE_H);

    for (int var = 0; var < image_set_size; ++var) {
        std::string name = myCamera.name+"/image_set/";
        name = name+files[var];
        threads[var] = std::thread(proccess_ImageSet, std::ref(camera_decoded_poses[std::stoi(files[var])]), files[var], name, this->phs_pattern_encoder.vertical_PatSeq.featurePoint_indicator, this->phs_pattern_encoder.horizontal_PatSeq.featurePoint_indicator, TILE_SIZE_V, TILE_SIZE_H);
    }
    for (int var = 0; var < image_set_size; ++var) {
        threads[var].join();
    }

    std::cout << "CheckerBoards Done!\n";
    myCamera.resolution = camera_decoded_poses[0].vertical_pattern_seq.pattern_size;
    apply_calibration();

    return false;
}

bool OFF_Cam_Calib::apply_calibration(){

    for (int pose_id = 0; pose_id < image_set_size; ++pose_id) {

        std::vector<cv::Point2f> image_points;
        std::vector<cv::Point3f> object_Points;
        for (uint feature_id = 0; feature_id < camera_decoded_poses[pose_id].feature_points.size(); ++feature_id) {
            cv::Point2f temp_image_corner(camera_decoded_poses[pose_id].feature_points[feature_id].pY, camera_decoded_poses[pose_id].feature_points[feature_id].pX);
            cv::Point3f temp_object_point(camera_decoded_poses[pose_id].feature_points[feature_id].rY, camera_decoded_poses[pose_id].feature_points[feature_id].rX, 0.0f);
            std::cout << temp_image_corner << " <- image , object -> " << temp_object_point << std::endl;

            image_points.push_back(temp_image_corner);
            object_Points.push_back(temp_object_point);
        }
        all_image_corners.push_back(image_points);
        all_Object_points.push_back(object_Points);

    }
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    cv::calibrateCamera(all_Object_points, all_image_corners, this->myCamera.resolution, myCamera.intrinsics, myCamera.distortion_coeffs, rvecs, tvecs);
    cv::Mat R;
    cv::Rodrigues(rvecs[0], R);

    myCamera.rotation = R.clone();
    myCamera.translation = tvecs[0];

    std::cout << "R " << myCamera.rotation << std::endl;
    std::cout << "T " << myCamera.translation << std::endl;

    for (int i = 0; i < image_set_size; ++i) {
        this->camera_decoded_poses[i].rotation = rvecs[i];
        this->camera_decoded_poses[i].translation = tvecs[i];
    }

    std::string fileR ="X,Y\n";
    for (int poseID = 1; poseID < all_Object_points.size(); ++poseID) {
        std::vector<cv::Point2f> imgp;
        cv::projectPoints(all_Object_points[poseID], rvecs[poseID], tvecs[poseID],myCamera.intrinsics, myCamera.distortion_coeffs, imgp);
        std::vector<float> reprojectionError;
        float errorX, errorY;
        for (int it = 0; it < all_Object_points[poseID].size(); ++it) {
            errorX = all_image_corners[poseID][it].x - imgp[it].x;
            errorY = all_image_corners[poseID][it].y - imgp[it].y;
            std::printf("Reprojection Error %f , %f \n", errorX, errorY);
            fileR += std::to_string(errorX) + "," + std::to_string(errorY) + "\n";
        }
    }
    std::cout << this->myCamera.name << std::endl;
    std::cout << fileR ;
    File_Manager f;
    f.write_to_file(this->myCamera.name + ".data", fileR);
    return false;
}

bool OFF_Cam_Calib::proccess_ImageSet(PhaseShifting_pattern_decoder &camera_decoded_poses, std::__cxx11::string image_set_number, std::__cxx11::string camera_name, std::vector<Feature_Point> vertical_featurePoint_indicator, std::vector<Feature_Point> horizontal_featurePoint_indicator, int v_tile_size, int h_tile_size)
{
    std::string path = "data/calibration/"+camera_name+"/";
    cv::Mat w, b, tempM;
    w = cv::imread(path + "w.tif");
    b = cv::imread(path + "b.tif");
    w.row(0) = cv::Scalar(0);
    w.row(1) = cv::Scalar(0);
    w.row(2) = cv::Scalar(0);
    w.row(3) = cv::Scalar(0);
    w.row(4) = cv::Scalar(0);

    b.row(0) = cv::Scalar(0);
    b.row(1) = cv::Scalar(0);
    b.row(2) = cv::Scalar(0);
    b.row(3) = cv::Scalar(0);
    b.row(4) = cv::Scalar(0);

    cv::cvtColor(b, b, CV_BGR2GRAY);
    cv::cvtColor(w, w, CV_BGR2GRAY);

    tempM = cv::abs(w - b);
    cv::Mat mask = cv::Mat::zeros(w.rows, w.cols, CV_32FC1);
    for (int i = 0; i < w.rows; ++i) {
        for (int j = 0; j < w.cols; ++j) {
            if(tempM.at<uchar>(i,j) > 64){
                mask.at<float>(i,j) = 1.0;
            }
        }
    }

    cv::Mat dot;
    dot = cv::imread(path + "d.tif");
    dot.row(0) = cv::Scalar(0);
    dot.row(1) = cv::Scalar(0);
    dot.row(2) = cv::Scalar(0);
    dot.row(2) = cv::Scalar(0);

    cv::cvtColor(dot, dot, CV_BGR2GRAY);
    dot.convertTo(dot,CV_32F);
    dot = dot/255.0f;
    dot = dot.mul(mask);

    double maxV;
    cv::Point maxIdx;
    cv::minMaxLoc(dot, NULL, &maxV, NULL, &maxIdx);
    for (int i = 0; i < maxIdx.x; ++i) {
        dot.at<float>(maxIdx.y, i) = 1;

    }
    //    std::cout << "maxIdx " << maxIdx << std::endl;
    //    cv::namedWindow("dotdot", cv::WINDOW_NORMAL);
    //    cv::imshow("dotdot", dot);
    //    cv::waitKey(0);
    PhaseShifting_pattern_decoder pattern_decoder = PhaseShifting_pattern_decoder(std::atoi(image_set_number.c_str()), camera_name, vertical_featurePoint_indicator, horizontal_featurePoint_indicator, v_tile_size, h_tile_size, mask);

    pattern_decoder.load_patterns(path);
    printThreadBase("Patterns loaded!", image_set_number);

    pattern_decoder.decode(maxIdx);
    printThreadBase("Patterns decoded!", image_set_number);

    cv::Mat ch_temp = cv::imread(path + "c.tif");
    //    Pattern temp = Pattern(1,std::string("feature_points"), pattern_decoder.vertical_pattern_seq.pattern_size, CV_8UC1);
    //    temp.image = cv::Scalar(0);
    std::vector<cv::KeyPoint> temp_vector;
    for (int var = 0; var < pattern_decoder.feature_points.size(); ++var) {
        temp_vector.push_back(cv::KeyPoint(pattern_decoder.feature_points[var].pY , pattern_decoder.feature_points[var].pX,10));
    }
    //    cv::drawKeypoints(temp.image, temp_vector, temp.image);
    cv::drawKeypoints(pattern_decoder.mask_image, temp_vector, ch_temp, cv::Scalar(0, 0, 255));
    cv::imwrite("data/calibration/" + std::to_string(camera_name.at(0)) + "features" + image_set_number + ".jpg", ch_temp);
    //    cv::imwrite("data/calibration/" + std::to_string(camera_name.at(0)) + "features" + image_set_number + ".jpg", temp.image);
    camera_decoded_poses = pattern_decoder;
    pattern_decoder.vertical_pattern_seq.patterns.sequence.clear();
    pattern_decoder.horizontal_pattern_seq.patterns.sequence.clear();

    return false;
}

void OFF_Cam_Calib::printThreadBase(std::string str, std::string ID){
    std::cout << "[" << ID << "] :: " << str << std::endl;
}

bool OFF_Cam_Calib::print(int mode, std::string output_file_name){
    File_Manager file_manager = File_Manager("data/calibration/");
    std::string output = "";
    output.append("-------------------------------------------------------\n");
    output.append("Camera Calibration info for " + myCamera.name + "\n");
    output.append("Camera Intrinsics: \n");
    output.append(file_manager.mat_to_string(myCamera.intrinsics));
    output.append("\n");
    output.append("Distortion coeffs: \n").append(file_manager.mat_to_string(myCamera.distortion_coeffs)).append("\n");
    for (uint i=0; i< camera_decoded_poses.size() ;i++)    {
        output.append("pose name :: ");
        output.append(camera_decoded_poses[i].name + "\n");
        output.append("\nrotations : \n").append(file_manager.mat_to_string(camera_decoded_poses[i].rotation)).append("\n");
        output.append("translations: \n").append(file_manager.mat_to_string(camera_decoded_poses[i].translation)).append("\n");
        output.append("-------------------------------------------------------\n");
    }
    if (mode == 0) {
        std::cout << output;
    } else if ( mode == 1){
        file_manager.write_to_file(output_file_name, output);
    } else if ( mode == 2){
        std::cout << output;
        file_manager.write_to_file(output_file_name, output);
    }
    return false;
}

