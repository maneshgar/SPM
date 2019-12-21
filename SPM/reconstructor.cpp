
#include "Image/ImageProcessing.h"
#include "reconstructor.h"

using namespace cv;
Reconstructor::Reconstructor(Camera &input_camera_left, Camera &input_camera_right, Camera &input_camera_top, Projector &Proj): geometric_Obj(nullptr)
{
    this->camera_left = input_camera_left;
    this->camera_right = input_camera_right;
    this->camera_top = input_camera_top;
    this->projector = Proj;
    this->projector_resolution = this->projector.resolution;
    this->matches_points.resize(projector.resolution.width * projector.resolution.height);
    normal_Map = Mat(projector_resolution.height, projector_resolution.width, CV_32FC3);
    xyz_Map = Mat(projector_resolution.height, projector_resolution.width, CV_32FC3);
}

Reconstructor::~Reconstructor(){

    this->geometric_Obj = nullptr;
}

bool Reconstructor::reconstruct(){

    Dot_sequence left_dot_sequence;
    left_dot_sequence.ID = 1;
    left_dot_sequence.name = "Left";

    Dot_sequence right_dot_sequence;
    right_dot_sequence.ID = 2;
    right_dot_sequence.name = "Right";

    load_dot_sequence(left_dot_sequence, "Captured/dot_sequence/");
    load_dot_sequence(right_dot_sequence, "Captured/dot_sequence/");
    this->camera_resolution = left_dot_sequence.black.image.size();
    find_matches_points(left_dot_sequence, right_dot_sequence, 30);
    {
        //    for (int var = 0; var < matches_points.size(); ++var) {
        //        if (matches_points[var].should_count) {
        //            std::cout << matches_points[var].right_points.size() <<"\n";
        //            for (int i = 0; i < matches_points[var].right_points.size(); ++i) {
        //                std::cout << "------------------\n\t" << matches_points[var].right_points[i] <<"\n";
        //            }
        //        }
        //    }
    }
    Triangulate();

    {
        //    float sumx = 0;
        //    float sumy = 0;
        //    float sumz = 0;

        //    for (int var = 0; var < point_cloud.size(); ++var) {
        //        sumx += point_cloud[var].at<float>(0,0);
        //        sumy += point_cloud[var].at<float>(1,0);
        //        sumz += point_cloud[var].at<float>(2,0);
        //    }

        //    sumx = sumx / point_cloud.size();
        //    sumy = sumy / point_cloud.size();
        //    sumz = sumz / point_cloud.size();
        //    for (int var = 0; var < point_cloud.size(); ++var) {
        //        point_cloud[var].at<float>(0,0) -= sumx;
        //        point_cloud[var].at<float>(1,0) -= sumy;
        //        point_cloud[var].at<float>(2,0) -= sumz;
        //    }
    }
    File_Manager file_m("data/model/");
    file_m.export_obj(this->point_cloud, "point_cloud.obj");
    return false;
}

bool Reconstructor::find_matches_points(Dot_sequence left_captured_images, Dot_sequence right_captured_images, int threshold){
    //    for (uint image_ID = 0; image_ID < left_captured_images.dot_sequence_names.size(); ++image_ID) {
    //        Mat rgb_left_Mat = imread("Captured/dot_sequence/Left/Images/" + left_captured_images.dot_sequence_names[image_ID]);
    //        Mat rgb_right_Mat = imread("Captured/dot_sequence/Right/Images/" + right_captured_images.dot_sequence_names[image_ID]);
    //        Mat left_Mat[3];
    //        split(rgb_left_Mat, left_Mat);
    //        Mat right_Mat[3];
    //        split(rgb_right_Mat, right_Mat);
    //        int count_r = get_points(left_Mat[2], left_captured_images.black, left_captured_images.mask, matches_points[image_ID].left_points, matches_points[image_ID].left_mean_matches_point);
    //        int count_l = get_points(right_Mat[2], right_captured_images.black, right_captured_images.mask, matches_points[image_ID].right_points, matches_points[image_ID].right_mean_matches_point);

    //        if(count_l > 0 && count_r > 0){
    //            this->matches_points[image_ID].should_count = true;
    //            {
    //                //            for (int var = 0; var < 1; ++var) {
    //                //                circle(left_Mat[2],matches_points[image_ID].left_points[var],10 , Scalar(255,2552,55));
    //                //                imshow("testl", left_Mat[2]);
    //                //                circle(right_Mat[2],matches_points[image_ID].right_points[var],10 , Scalar(255,2552,55));
    //                //                imshow("testr", right_Mat[2]);
    //                //                waitKey(50);
    //                //            }
    //            }
    //        }
    //    }
    //    std::cout << "Matches Found!\n";
}

bool Reconstructor::find_mask_image(Dot_sequence &dot_seq){
    Mat mask(dot_seq.mask.image.size(), CV_32FC1);
    mask = dot_seq.white.image - dot_seq.black.image;
    threshold(mask, mask, 32, 1, THRESH_BINARY);
    dot_seq.mask = Pattern(dot_seq.ID, "mask", mask.clone());
    return false;
}

int Reconstructor::get_points(Mat input_image, Pattern black_image, Pattern mask_image, std::vector<Point2f> &matches, Point2f &mean_matches){
    int window_size = 5;
    int threshold = 5;
    int count = 0;
    int mean_i = 0;
    int mean_j = 0;
    Point max_point;
    Mat temp_image = input_image - black_image.image;
    temp_image = temp_image.mul(mask_image.image);
    minMaxLoc(temp_image, NULL, NULL, NULL, &max_point);
    for (int i = max_point.y - window_size ; i < max_point.y + window_size; ++i) {
        for (int j = max_point.x - window_size ; j < max_point.x + window_size; ++j) {
            if( i > 0 && j > 0 && i < this->camera_resolution.width && j < this->camera_resolution.height){
                if((int)(temp_image.at<uchar>(i,j)) >  threshold){
                    matches.push_back(Point2f(j,i));
                    mean_i += i;
                    mean_j += j;
                    count++;
                }
            }
        }
    }if(count += 1){
        mean_i /= count;
        mean_j /= count;
    }
    mean_matches = Point2f(mean_j, mean_i);
    return count;
}


bool Reconstructor::load_dot_sequence(Dot_sequence &dot_sequence, std::__cxx11::string directory){

    File_Manager file_manager(directory);
    file_manager.load_pattern(dot_sequence.black, dot_sequence.name + std::string("/Mask/black.jpg"));
    file_manager.load_pattern(dot_sequence.white, dot_sequence.name + std::string("/Mask/white.jpg"));
    file_manager.get_dir_list(dot_sequence.dot_sequence_names, dot_sequence.name + std::string("/Images/"));
    find_mask_image(dot_sequence);
}

bool Reconstructor::display_dot_pattern(){
    Mat red(projector.getHeight(), projector.getWidth(), CV_8UC3, Scalar(0, 0, 255));
    Mat black(projector.getHeight(), projector.getWidth(), CV_8UC1, Scalar(0));

    Pattern black_pattern(2, "black", black.clone());
    Pattern red_pattern(1, "white", red.clone());

    black_pattern.display("Patterns", CV_WINDOW_KEEPRATIO);
    red_pattern.display("Patterns", CV_WINDOW_KEEPRATIO);
    imwrite("DotImages/black.jpg", black_pattern.image);
    imwrite("DotImages/white.jpg", red_pattern.image);

    int id;
    std::vector<Mat> Images;
    Mat dotImage(projector.getResolution(), CV_8UC3, Scalar(0));
    for (int i = 0; i < projector.getHeight(); ++i) {
        for (int j = 0; j < projector.getWidth(); ++j) {
            id = i *projector.getWidth() + j;
            dotImage.at<Vec3b>(i,j) = Vec3f(0,0,255);
            Images.push_back(dotImage.clone());
            namedWindow("RED DOT", WINDOW_NORMAL);
            imshow("RED DOT", dotImage);
            waitKey(1);
            dotImage.at<Vec3b>(i,j) = Vec3f(0,0,0);
            //            imwrite("data/red_dot/"+std::to_string(id)+".bmp", Images[id]);
        }
    }
}

bool Reconstructor::Triangulate(){
    for ( uint i=0; i< matches_points.size(); i++)
    {
        //        std::cout << "Triangulation :: " << i << std::endl;
        Match matched = matches_points[i];

        //Progress
        if (matched.should_count)
        {
            Point2f nearest_pixel_left;
            Point2f nearest_pixel_right;

            float min_distant = std::numeric_limits<float>::max();
            float ptCount = 0.0;

            Mat min_mid_point(3, 1, CV_32FC1, Scalar(0.0f));
            Mat reconstructed_point(3, 1, CV_32FC1, Scalar(0.0f));

            for (const auto& left_match: matched.left_points){
                for (const auto& right_match: matched.right_points){
                    float dist=-1.0f;

                    auto midP = find_mid_point_Bkp(this->camera_left.get_ray(left_match), this->camera_right.get_ray(right_match), dist);
                    if (dist > 0.0) // if dist is valid
                    {
                        ptCount += 1.0;
                        reconstructed_point += midP;
                        if (dist < min_distant)
                        {
                            min_distant = dist;
                            min_mid_point = midP.clone();
                            nearest_pixel_left = left_match;
                            nearest_pixel_right = right_match;
                        }
                    }
                }
            }
            // Output min ray
            reconstructed_point = reconstructed_point/ptCount;

            if (min_distant < 90) //Setting threshold
            {
                point_cloud.push_back(reconstructed_point);
            }
            else{
                std::cout << "Warning :: Distance is too high! wrong matches...  " << min_distant << "\n";
                point_cloud.push_back(Mat(3, 1, CV_32FC1, Scalar(0.0f)).clone());
            }
        }else{
            point_cloud.push_back(Mat(3, 1, CV_32FC1, Scalar(0.0f)).clone());
            std::cout << "ERROR :: you need to assign sth here!!! should be a bug!\n";
        }
    }
}

Mat Reconstructor::find_mid_point_Bkp(Ray r1, Ray r2, float &dist){

    Mat v1(3,1,CV_32FC1);
    v1.at<float>(0,0) = r1.dir.at<float>(0,0);
    v1.at<float>(1,0) = r1.dir.at<float>(1,0);
    v1.at<float>(2,0) = r1.dir.at<float>(2,0);

    Mat v2(3,1,CV_32FC1);
    v2.at<float>(0,0) = r2.dir.at<float>(0,0);
    v2.at<float>(1,0) = r2.dir.at<float>(1,0);
    v2.at<float>(2,0) = r2.dir.at<float>(2,0);

    Mat p1(3,1,CV_32FC1);
    p1.at<float>(0,0) = r1.origin.at<float>(0,0);
    p1.at<float>(1,0) = r1.origin.at<float>(1,0);
    p1.at<float>(2,0) = r1.origin.at<float>(2,0);

    Mat p2(3,1,CV_32FC1);
    p2.at<float>(0,0) = r2.origin.at<float>(0,0);
    p2.at<float>(1,0) = r2.origin.at<float>(1,0);
    p2.at<float>(2,0) = r2.origin.at<float>(2,0);

    Mat v12 = p1 - p2;
    float v1_dot_v1 = v1.dot(v1);
    float v2_dot_v2 = v2.dot(v2);
    float v1_dot_v2 = v1.dot(v2);
    float v12_dot_v1 = v12.dot(v1);
    float v12_dot_v2 = v12.dot(v2);
    float denom = v1_dot_v1 * v2_dot_v2 - v1_dot_v2 * v1_dot_v2;

    if (fabs(denom) < 0.1)
    {
        dist = -1.0;
        std::cout << "Warning :: Parallel Rays!" <<std::endl;
        //                return Mat(4,1,CV_32FC1, Scalar(0.0));
    }

    float s =  (v1_dot_v2/denom) * v12_dot_v2 - (v2_dot_v2/denom) * v12_dot_v1;
    float t = -(v1_dot_v2/denom) * v12_dot_v1 + (v1_dot_v1/denom) * v12_dot_v2;
    dist = norm(p1 + s*v1 - p2 - t*v2);
    Mat temp_res= (p1 + s*v1 + p2 + t*v2)/2.0f;
    return temp_res.clone();
}

bool Reconstructor::PCloudToMesh(bool holeFilling, bool check){

    int image_height = projector_resolution.height;
    int image_width = projector_resolution.width;

    Image *xyz = new Image(image_width,image_height);
    int x;
    int y;
    for (int i = 0; i < point_cloud.size(); ++i) {
        Vec3f temp_Vec(point_cloud[i].at<float>(0,0), point_cloud[i].at<float>(1,0), point_cloud[i].at<float>(2,0));
        Color temp_color(temp_Vec[0], temp_Vec[1], temp_Vec[2]);
        x = i / image_height;
        y = i % image_height;
        xyz->setPixel(x, y, temp_color);
    }

    xyz->getcvMat_RGB(this->xyz_Map);
    get_projector_location();

    if (holeFilling) {
        std::cout << "Hole Filling started!\n";
        ImageProcessing::holeFill(xyz);
        std::cout << "Hole Filling Ended\n";
    }

    Mat Mask_geom(image_height, image_width, CV_32FC3, Scalar(1.0, 1.0, 1.0))    ;
    xyz->getcvMat_RGB(this->xyz_Map);
    Mat bilateral_output, result, bilateral_mask;
    for (int i = 0; i < xyz_Map.rows; ++i) {
        for (int j = 0; j < xyz_Map.cols; ++j) {
            Vec3f temp_Vec = xyz_Map.at<Vec3f>(i,j);
            if (temp_Vec[0] == 0 && temp_Vec[1] == 0.0 && temp_Vec[2] == 0.0) {
                Mask_geom.at<Vec3f>(i,j) = Vec3f(0.0, 0.0, 0.0);
            }
        }
    }


    bilateralFilter(xyz_Map, bilateral_output, 15, 50, 50);
    bilateralFilter(Mask_geom, bilateral_mask, 15, 50, 50);

    divide(bilateral_output, bilateral_mask, result);

    Image t(result);
    xyz->copy(&t);


    ///get output file for PBRT

    ImageProcessing::ExportToPBRT(t);

    /////////

    Image *normalp = GeometryProcessing::computeNormalMap(xyz, false);

    normalp->getcvMat_RGB(this->normal_Map); ///////////////////////Decide for RGB/BGR/GRB :)

    geometric_Obj = GeometryProcessing::triangulate(xyz, normalp);

    xyz->getcvMat_RGB(this->xyz_Map);

    File_Manager fm("data/data/");
    fm.export_Mat32FC3(normal_Map, "normals");
    fm.export_Mat32FC3(xyz_Map, "xyz_map");

    {
        ///**Display normal
                double min;
                double max;
                minMaxIdx(normal_Map, &min, &max);
                Mat adjMap;
                convertScaleAbs(normal_Map, adjMap, 255 / max);
                namedWindow("normal", WINDOW_NORMAL);
                imshow("normal" , abs(adjMap));
                waitKey(0);

        ///**Display xyz_map
        //        double minD;
        //        double maxD;
        //        std::vector<Mat> xyz_vec;
        //        split(xyz_Map,xyz_vec);
        //        minMaxIdx(xyz_vec[2], &minD, &maxD);
        //        xyz_vec[2] -= minD;
        //        Mat adjMapD;
        //        convertScaleAbs(xyz_vec[2], adjMapD, 255 / (maxD-minD));
        //        namedWindow("xyz", WINDOW_NORMAL);
        //        imshow("xyz" , adjMapD);
        //        waitKey(0);
    }


    GeometryExporter *GeoExporter = new GeometryExporter();
    GeoExporter->exportToOBJ("data/model/mesh_bilateral", this->geometric_Obj);

    delete GeoExporter;
    delete xyz;
    delete normalp;

    return false;
}

bool Reconstructor::get_projector_location(){

    std::vector<std::vector<Point3f>> all_Obj_points;
    std::vector<std::vector<Point2f>> all_Img_points;

    std::vector<Point3f> Obj_points;
    std::vector<Point2f> Img_points;

    std::vector<Mat> rvec, tvec;
    Mat intr = Mat::eye(3,3,CV_32FC1);
    std::cout << "start point" << intr << std::endl;
    intr.at<float>(0,0) = 1000;
    intr.at<float>(1,1) = 1000;
    intr.at<float>(0,2) = projector.getWidth()/2;
    intr.at<float>(1,2) = projector.getHeight()/2;
    std::cout << "end point" << intr << std::endl;

    Mat dis;
    for (int i = 0; i < xyz_Map.cols; i++) {
        for (int j = 0; j < xyz_Map.rows; j++) {
            Vec3f temp = xyz_Map.at<Vec3f>(j,i);
            if (temp[0] == 0 && temp[1] == 0 && temp[2] == 0) continue; // not a valid point
            Point3f temppoint(temp[0], temp[1], temp[2]); //X , Y , Z
            Obj_points.push_back(temppoint);
            Img_points.push_back(Point2f(i,j));
        }
    }

    all_Img_points.push_back(Img_points);
    all_Obj_points.push_back(Obj_points);
    calibrateCamera(all_Obj_points, all_Img_points, projector.getResolution(), intr, dis, rvec, tvec, CV_CALIB_USE_INTRINSIC_GUESS);

    //save projectors data
    Mat R;
    Rodrigues(rvec[0], R);

    projector.intrinsics = intr.clone();
    projector.distortion_coeffs = dis.clone();
    projector.rotation = R.clone();
    projector.translation = tvec[0];

    std::cout << projector.intrinsics << std::endl;
    std::cout << projector.rotation << std::endl;
    std::cout << projector.translation << std::endl;
    std::cout << projector.distortion_coeffs << std::endl;

    std::string fileR ="X,Y\n";
    std::vector<Point2f> imgp;

    //Compute the reprojection error

    std::cout << rvec[0] << std::endl;
    std::cout << tvec[0] << std::endl;
    std::cout << projector.intrinsics << std::endl;
    std::cout << projector.distortion_coeffs << std::endl;


    projectPoints(all_Obj_points[0], rvec[0], tvec[0],projector.intrinsics, projector.distortion_coeffs, imgp);
    std::vector<float> reprojectionError;
    float errorX, errorY;
    for (int it = 0; it < all_Img_points[0].size(); ++it) {
        errorX = all_Img_points[0][it].x - imgp[it].x;
        errorY = all_Img_points[0][it].y - imgp[it].y;
        std::printf("Reprojection Error %f , %f \n", errorX, errorY);
        if(it%150 == 0)
            fileR += std::to_string(errorX) + "," + std::to_string(errorY) + "\n";
    }

    std::cout << this->projector.name << std::endl;
    std::cout << fileR ;
    File_Manager f;
    f.write_to_file(projector.name + ".data", fileR);


    std::cout << "Rotation :: " << projector.rotation << std::endl;
    std::cout << "Translation :: " << projector.translation << std::endl;
    std::cout << "distortion :: " << projector.distortion_coeffs << std::endl;

    projector.saveConfig("Projector.xml");
    return false;

}

bool Reconstructor::store_data(){
    store_matches();
}

bool Reconstructor::store_matches(){
    File_Manager fm("Output/Data/");
    std::string str = std::to_string(this->matches_points.size());
    str += "\n";
    for (int i = 0; i < this->matches_points.size(); ++i) {
        str += match_to_store_string(this->matches_points[i]);
    }
    fm.write_to_file("mathces.data", str);
}
