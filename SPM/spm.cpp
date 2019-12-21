
#include "spm.h"

SPM::SPM()
{
    camera_left  = Camera(1, "Left", cv::Size(CAMERA_WIDTH, CAMERA_HEIGHT));
    camera_right = Camera(2, "Right", cv::Size(CAMERA_WIDTH, CAMERA_HEIGHT));
    camera_top = Camera(3, "Top", cv::Size(CAMERA_WIDTH, CAMERA_HEIGHT));
    projector = Projector(1, "Projector", cv::Size(PROJECTOR_WIDTH, PROJECTOR_HEIGHT));
}

SPM::~SPM(){

}

bool SPM::display_phaseShifted_patterns_func(uint iteration){
    PhaseShifting_Pattern_encoder PhS_pattern_encoder = PhaseShifting_Pattern_encoder(PHASESHIFTING_STEP, cv::Size(LCD_WIDTH, LCD_HEIGH), CV_8UC1);
    PhS_pattern_encoder.generate_patterns();
    PhS_pattern_encoder.encode_featurePoints(FEATURE_DISTANCE, FEATURE_DISTANCE);
    PhS_pattern_encoder.display_encoded_features();
    PhS_pattern_encoder.CheckerBoard.display("Phase Shifted Patterns");
    PhS_pattern_encoder.black.display("Phase Shifted Patterns");
    PhS_pattern_encoder.white.display("Phase Shifted Patterns");
    PhS_pattern_encoder.syncronize_image.display("Phase Shifted Patterns");
    for (uint i = 0; i < iteration; ++i) {
        PhS_pattern_encoder.display_patterns("Phase Shifted Patterns");
    }
    return true;
}

bool SPM::calibrate_cameras_func(){

    PhaseShifting_Pattern_encoder encoder = PhaseShifting_Pattern_encoder(PHASESHIFTING_STEP,cv::Size(LCD_WIDTH,LCD_HEIGH),CV_8UC1);
    //    encoder.generate_patterns();
    encoder.encode_featurePoints(FEATURE_DISTANCE, FEATURE_DISTANCE);
    //    encoder.display_encoded_features();

    camera_left = Camera(0,"left");
    camera_right = Camera(1,"right");
    camera_top = Camera(2,"top");

    OFF_Cam_Calib calibrator_left = OFF_Cam_Calib(encoder, camera_left);
    calibrator_left.calibrate();
    camera_left .save_device();
    camera_left .saveConfig("Cam0.xml");

    OFF_Cam_Calib calibrator_right = OFF_Cam_Calib(encoder, camera_right);
    calibrator_right.calibrate();
    camera_right.save_device();
    camera_right.saveConfig("Cam1.xml");

    OFF_Cam_Calib calibrator_top = OFF_Cam_Calib(encoder, camera_top);
    calibrator_top.calibrate();
    camera_top.save_device();
    camera_top.saveConfig("Cam2.xml");

    cv::Mat Rot1, Trans1;
    //    cv::Mat Rot2, Trans2;

    camera_right.get_RT_with_another_device(camera_left, Rot1, Trans1);
    //    camera_right.get_RT_with_another_device(camera_top, Rot2, Trans2);

    std::cout << "Rot1 " << Rot1 << std::endl;
    //    std::cout << "Rot2 " << Rot2 << std::endl;
    std::cout << "Trans1 " << Trans1 << std::endl;
    //    std::cout << "Trans2 " << Trans2 << std::endl;
    return false;
}

bool SPM::reconstruct_func(){

    Reconstructor recontructor(camera_left, camera_right, camera_top, projector);
    //    recontructor.display_dot_pattern();
    recontructor.reconstruct();

    return false;
}

bool SPM::pointCloudToMesh_func(){
    Reconstructor recontructor(camera_left, camera_right, camera_top, projector);
    File_Manager fm;
    std::string path;
    cv::Size image_size(1920, 1200);

    ///load matches
    //    path = "Output/Data/Alexandre-Matches.txt";
    //    recontructor.matches_points = fm.import_matches(path, image_size);
    //    std::cout << "Matches have been imported successfully!\n";
    ///

    {
        /**check input**/
        //                cv::Mat left_image(cv::Mat(image_size.height, image_size.width, CV_8UC3));
        //                cv::Vec3b color (0 , 0 , 0);
        //                for (int i = 0; i < recontructor.matches_points.size(); ++i) {
        //                    color[0]++;
        //                    if(color[0] >= 255){
        //                        color[1]++;
        //                        if(color[1] >= 255){
        //                            color[2]+=20;
        //                            if(color[2] >= 236){
        //                                std::cout << "ERROR:: LACK OF COLOR!\n";
        //                            }
        //                        }
        //                    }
        //                    std::cout << color << std::endl;
        //                    for (int j = 0; j < recontructor.matches_points[i].left_points.size(); ++j) {
        //                        int col = recontructor.matches_points[i].left_points[j].x;
        //                        int row = recontructor.matches_points[i].left_points[j].y;
        //                        left_image.at<cv::Vec3b>(row, col) = color;
        //                    }
        //                }
        //                cv::namedWindow("left image", cv::WINDOW_NORMAL);
        //                cv::imshow("left image", left_image);
        //                cv::imwrite("leftcolored.jpg", left_image);

        //                //check input right**
        //                cv::Mat right_image(cv::Mat(image_size.height, image_size.width, CV_8UC3));
        //                cv::Vec3b color2 (0 , 0 , 0);
        //                for (int i = 0; i < recontructor.matches_points.size(); ++i) {

        //                    color2[0]++;
        //                    if(color2[0] >= 255){
        //                        color2[1]++;
        //                        if(color2[1] >= 255){

        //                            if(color2[2] + 20 > 255){
        //                                std::cout << "ERROR:: LACK OF COLOR!\n";
        //                            }else{
        //                                color2[2]+= 20;
        //                            }
        //                        }
        //                    }
        //                    for (int j = 0; j < recontructor.matches_points[i].right_points.size(); ++j) {
        //                        int col = recontructor.matches_points[i].right_points[j].x;
        //                        int row = recontructor.matches_points[i].right_points[j].y;
        //                        right_image.at<cv::Vec3b>(row, col) = color2;
        //                    }
        //                }
        //                cv::namedWindow("right image", cv::WINDOW_NORMAL);
        //                cv::imshow("right image", right_image);
        //                cv::imwrite("rightcolored.jpg", right_image);
        //                cv::waitKey(0);
    }

    ///triangulate
    {
        //        File_Manager file_m("Output/Point_Cloud/");
        //        recontructor.Triangulate();
        //        file_m.export_obj(recontructor.point_cloud);
    }
    ///

    ///import point cloud
    path = "data/model/point_cloud.obj";
    recontructor.point_cloud = fm.import_obj(path);
    std::cout << "Point Cloud has been imported!" << std::endl;
    ///


    ///Display Depth Map
    {
//        cv::Mat temp(projector.resolution, CV_32FC3);
//        cv::Mat Depth_Map(PROJECTOR_HEIGHT, PROJECTOR_WIDTH, CV_32FC1, cv::Scalar(0.5));
//        for (int itr = 0; itr < recontructor.point_cloud.size(); ++itr) {
//            int i = (itr % PROJECTOR_HEIGHT);
//            int j = (itr/ PROJECTOR_HEIGHT);

//            //            Depth_Map.at<float>(i, j) = 1;
//            Depth_Map.at<float>(i, j) = recontructor.point_cloud[itr].at<float>(2, 0);
//            temp.at<cv::Vec3f>(i,j) = cv::Vec3f(recontructor.point_cloud[itr].at<float>(0, 0), recontructor.point_cloud[itr].at<float>(1, 0), recontructor.point_cloud[itr].at<float>(2, 0));
//        }
//        recontructor.xyz_Map = temp.clone();

//        double min = 1000000;
//        double max = -1000000;

//        //find min and max

//        for (int var1 = 0; var1 < Depth_Map.rows; ++var1) {
//            for (int var2 = 0; var2 < Depth_Map.cols; ++var2) {
//                if (Depth_Map.at<float>(var1,var2) > max && Depth_Map.at<float>(var1,var2) != 0.0) {
//                    max = Depth_Map.at<float>(var1,var2);
//                }
//                if (Depth_Map.at<float>(var1,var2) < min && Depth_Map.at<float>(var1,var2) != 0.0) {
//                    min = Depth_Map.at<float>(var1,var2);
//                }
//            }
//        }

//        Depth_Map -= min;
//        max -= min;

//        Depth_Map /= max;
//        cv::imshow("depth_map", (1-Depth_Map));
//        cv::waitKey(0);
    }
    ///Create mesh
    recontructor.PCloudToMesh(true, false);
    this->Geometric_Obj = *recontructor.geometric_Obj;
    std::cout << "3D mesh has been reconstructed!" << std::endl;
    //    recontructor.get_projector_location();

    return false;
}

void SPM::light_proccessing_func(){

    ////////////Do The Job!
    ///
    load_system_func(false);
    lightProccessing LProcessor(camera_left, camera_right, camera_top, projector);

//    LProcessor.load_cameras(false);

    LProcessor.load_scene(false);

    LProcessor.extract_samples(false);

    LProcessor.compute_ASD_parameters(true);

    //    LProcessor.compute_compensate_image(true);
{

    //Test_the_job
    //    load_system_func(false);
    //    lightProccessing LProcessor(camera_left, camera_right, camera_top, projector);

    //    File_Manager fm("data/");
    //    cv::Mat temp1, temp2, tempMask;
    //    fm.import_Mat32F("data/normals.data", temp1);
    //    LProcessor.set_normals(temp1, true);

    //    fm.import_Mat32F("data/xyz_map.data", temp2);
    //    LProcessor.set_xyzMap(temp2, true);

    //    LProcessor.compute_CosNL(false);

    //    LProcessor.compute_Mask_Image(false);

    //    LProcessor.compute_CosRV(false);

    ////    LProcessor.load_matches(false);

    ////    std::string path = "data/Color_Oservation/";

    ////    LProcessor.initialize_colors(path, true);

    //    //    LProcessor.create_grayScale_test(true);

    //    cv::Mat inpimg = cv::imread("data/maps/diffuse.jpg");
    //    inpimg.convertTo(inpimg, CV_32FC3);
    //    inpimg /= 255.0;
    //    cv::split(inpimg, LProcessor.diffuse_map);

    //    inpimg = cv::imread("data/maps/specular.png");
    //    inpimg.convertTo(inpimg, CV_32FC3);
    //    inpimg /= 255.0;
    //    cv::split(inpimg, LProcessor.specular_map);

    //    LProcessor.create_testCase(true);

    //    //    LProcessor.create_MacbetChart_testCase(true);

    ////    LProcessor.fill_type_table(true);

    ////    LProcessor.compute_mask_type(false);

    ////    LProcessor.comput_ASD_parameters(true);

    //    LProcessor.compute_compensate_image(true);
















    //    LProcessor.create_grayScale_test(false);

    //    LProcessor.create_MacbetChart_testCase(false);


    //    LProcessor.compute_Diffuse_map(1, true);

    //    LProcessor.compute_Diffuse_map(2, true);

    //    LProcessor.Interpolate_Diffuse_map(1, true);

    //    LProcessor.compute_Diffuse_map_grayScale(3, true);

    //    cv::Mat Diffuse_Part = LProcessor.render_image_grayScale(0, 0, false);

    //    cv::Mat Diffuse_Part;
    //    LProcessor.render_image(0, 0, true);

    //    float a = LProcessor.compute_Alpha(Diffuse_Part);

    //    std::cout << "Alpha = " << a << std::endl;

    //    LProcessor.set_alpha(a);

    //    cv::Mat Diffuse_Part_left = LProcessor.render_image(0, 1, false);

    //    cv::Mat Diffuse_Part_right = LProcessor.render_image(0, 2, false);

    //    LProcessor.compute_Specular_map_grayScale(1, Diffuse_Part, 1, true);

    //    LProcessor.compute_Specular_map_grayScale(2, Diffuse_Part, 2, true);

    //    cv::Mat Specular_Part_left = LProcessor.render_image(1, 1, false);

    //    cv::Mat Specular_Part_right = LProcessor.render_image(1, 2, false);

    //    LProcessor.test_Levenberg_Marquardt(true);

    //    LProcessor.get_kd_ks_Alpha(true);

    //    LProcessor.apply_Levenberg_Marquardt_grayScale(true);

    //    cv::Mat Diffuse_Part2 = LProcessor.render_image_grayScale(0, 0, false);

    //    a = LProcessor.compute_Alpha(Diffuse_Part2);
    //    std::cout << "Alpha = " << a << std::endl;
    //    cv::waitKey(0);

}
    /*  cv::Mat LM_diffuse, LM_specular;
    C2PProcessing.apply_LM(Color(1.0f, 1.0f, 1.0f, 1), Diffuse_Map, nDOTl, Specular_Map, rDOTv, alpha, LM_diffuse, LM_specular);

    {
        //    cv::Mat rendered_Image, U, updated_Diffuse_Map, difference;
        //    bool cont = false;
        //    while(!cont){
        //        rendered_Image = C2PProcessing.render_image(Light, left_diffuse_map, cos);
        //        U = rendered_Image/(Projector_view);
        //        updated_Diffuse_Map = left_diffuse_map.mul(U);
        //        difference = updated_Diffuse_Map - left_diffuse_map;
        //        left_diffuse_map = updated_Diffuse_Map.clone();
        //        float tresh = 0.01f;
        //        for (int i = 0; i < difference.rows && !cont; ++i) {
        //            for (int j = 0; j < difference.cols && !cont; ++j) {
        //                cv::Vec3f x = difference.at<cv::Vec3f>(i,j);
        //                if (x[0] > tresh || x[1] > tresh || x[2] > tresh) {
        //                    cont = true;
        //                }
        //            }
        //        }
        //    }
        //    cv::namedWindow("rendered", cv::WINDOW_NORMAL);
        //    cv::imshow("rendered", rendered_Image);

        //    cv::namedWindow("U", cv::WINDOW_NORMAL);
        //    cv::imshow("U", U);
        //    cv::waitKey(0);

    }

    cv::Mat One(cv::Mat(left_diffuse_map.size(), CV_32FC3, cv::Scalar(1.0f,1.0f,1.0f)));
    cv::Mat Specular_saliency = cv::abs(left_diffuse_map-right_diffuse_map);
    cv::Mat Diffuse_saliency = One - Specular_saliency;



    left_diffuse_map *= 255;
    left_diffuse_map.convertTo(left_diffuse_map, CV_8UC3);

    right_diffuse_map *= 255;
    right_diffuse_map.convertTo(right_diffuse_map, CV_8UC3);

    Specular_saliency *= 255;
    Specular_saliency.convertTo(Specular_saliency, CV_8UC3);

    Diffuse_saliency *= 255;
    Diffuse_saliency.convertTo(Diffuse_saliency, CV_8UC3);

    Diffuse_Map *= 255;
    Diffuse_Map.convertTo(Diffuse_Map, CV_8UC3);

    //    cv::namedWindow("Diffuse_Map", cv::WINDOW_NORMAL);
    //    cv::imshow("Diffuse_Map", Diffuse_Map);

    //    cv::namedWindow("left", cv::WINDOW_NORMAL);
    //    cv::imshow("left", left_diffuse_map);

    //    cv::namedWindow("right", cv::WINDOW_NORMAL);
    //    cv::imshow("right", right_diffuse_map);

    //    cv::namedWindow("Spec-sal", cv::WINDOW_NORMAL);
    //    cv::imshow("Spec-sal", Specular_saliency);

    //    cv::namedWindow("Diff-sal", cv::WINDOW_NORMAL);
    //    cv::imshow("Diff-sal", Diffuse_saliency);

    cv::imwrite("Spec-sal.jpg", Specular_saliency);
    cv::imwrite("Diff-sal.jpg", Diffuse_saliency);
    cv::imwrite("left.jpg", left_diffuse_map);
    cv::imwrite("right.jpg", right_diffuse_map);

    //    cv::waitKey(0);

*/

    return;
}

bool SPM::load_system_func(bool check){

    //    camera_left. loadConfig("data/calibration/Cam0.xml");
    //    camera_right.loadConfig("data/calibration/Cam1.xml");
    //    camera_top.  loadConfig("data/calibration/Cam2.xml");
    projector.   loadConfig("data/calibration/Projector.xml");

    //    camera_left .compute_world_coordinates();
    //    camera_right.compute_world_coordinates();
    //    camera_top  .compute_world_coordinates();
    projector   .compute_world_coordinates();

    if(check){
        //        std::cout << "Camera Left:\nworld coordiantes :: \n" << camera_left.get_world_coordinates() << std::endl;
        //        std::cout << "Camera Right:\nworld coordiantes :: \n" << camera_right.get_world_coordinates() << std::endl;
        //        std::cout << "Camera top:\nworld coordiantes :: \n" << camera_top.get_world_coordinates() << std::endl;
        std::cout << "projector:\nworld coordiantes :: \n" << projector.get_world_coordinates() << std::endl;
    }
}

bool SPM::smooth_geometry(){

//    bool check = true;
//    cv::Mat input, output;
//    File_Manager fm("data/data/");

//    Image img(input);
//    Image xyz_temp;
//    bilateralFilter(img, xyz_temp,16.0, 0.5, 0.5);

//    Image *normalp = GeometryProcessing::computeNormalMap(xyz_temp, true);
//    geometric_Obj = GeometryProcessing::triangulate(xyz_temp, normalp);


//    File_Manager fm("data/data/");
//    fm.export_Mat32FC3(normal_Map, "normals_bilateral");
//    fm.export_Mat32FC3(xyz_Map, "xyz_map_bilateral");
//    GeometryExporter *GeoExporter = new GeometryExporter();
//    GeoExporter->exportToOBJ("data/model/mesh_bilateral", this->geometric_Obj);

//    delete GeoExporter;
//    delete xyz;
//    delete normalp;


    /*
    std::string source_path = "data/model/mesh-lab.obj";
    std::string mask_path = "data/model/geom-mask.obj";
    std::string destination_path = "data/model/point_cloud-geom.obj";

    std::vector<cv::Mat> point_cloud_source;
    std::vector<cv::Mat> point_cloud_destination;

    File_Manager fm;
    point_cloud_source = fm.import_obj(source_path); // mesh-lab.obj

    int flag;
    long double iterator = 0;

    cv::Mat point(3, 1, CV_32FC1);
    std::string line;
    std::ifstream myfile (mask_path); //mask_geom.obj

    if (myfile.is_open()){
        while(std::getline(myfile, line)){

            std::stringstream ssin(line);
            std::string charac;
            ssin >> charac;
            flag = std::stoi(charac);
            if(flag == 1){
//                std::cout << point_cloud_source.size() << "   " << iterator<< std::endl;

                point.at<float>(0,0) = point_cloud_source[iterator].at<float>(0,0);
                point.at<float>(1,0) = point_cloud_source[iterator].at<float>(1,0);
                point.at<float>(2,0) = point_cloud_source[iterator].at<float>(2,0);
                if (iterator < point_cloud_source.size() - 1) {
                    iterator++;
                }

                point_cloud_destination.push_back(point.clone());
            }else if (flag == 0) {
                point.at<float>(0,0) = 0;
                point.at<float>(1,0) = 0;
                point.at<float>(2,0) = 0;

                point_cloud_destination.push_back(point.clone());
            } else {
                std::cout << "SPM::smooth_geometry => Error! The flag is not right!\n";
            }
        }
        myfile.close();
    }

    fm.export_obj(point_cloud_destination, destination_path);

    Reconstructor recontructor(camera_left, camera_right, camera_top, projector);
    recontructor.point_cloud = point_cloud_destination;
    ///Create mesh
    recontructor.PCloudToMesh(false, false);
    std::cout << "3D mesh has been reconstructed!" << std::endl;
    */
    return true;
}
