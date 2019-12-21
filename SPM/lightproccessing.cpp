#include "lightproccessing.h"
#include <thread>
using namespace cv;


std::vector<float> parameters;
std::vector<float> Macbeth_R = {115 ,194 ,98 ,87 ,133 ,103 ,214 ,80 ,193 ,94 ,157 ,224 ,56 ,70 ,175 ,231 ,187 ,8 ,243 ,200 ,160 ,122 ,85 ,52};
std::vector<float> Macbeth_G = {82 ,150 ,122 ,108 ,128 ,189 ,126 ,91 ,90 ,60 ,188 ,163 ,61 ,148 ,54 ,199 ,86 ,133 ,243 ,200 ,160 ,122 ,85 ,52};
std::vector<float> Macbeth_B = {68 ,130 ,157 ,67 ,177 ,170 ,44 ,166 ,99 ,108 ,64 ,46 ,150 ,73 ,60 ,31 ,149 ,161 ,242 ,200 ,160 ,121 ,85 ,52};

std::vector< std::vector<float>> Macbeth = {Macbeth_B, Macbeth_G, Macbeth_R};

std::vector<Mat> global_Observe;
std::vector<Mat> global_cosRV;

Mat global_cosNL;
Mat global_mask;
Mat global_light;
Mat global_updated_kd;
Mat global_updated_ks;
Mat global_updated_alpha;

Mat global_diffue;
Mat global_specular;
Mat global_alpha;

lightProccessing::lightProccessing(Camera &left, Camera &right, Camera &top, Projector &p)
{

    projector = p;
    //    Size pRes = projector.resolution;
    this->light_source.resize(numberOfChannels);
    for (int i = 0; i < numberOfChannels; ++i) {
        light_source[i] = Mat(get_resolution(), CV_32FC1, Scalar(1.0f));
    }

    this->cosNL_map = Mat(get_resolution(), CV_32FC1, Scalar(0.0f));

    diffuse_map.resize(numberOfChannels);
    specular_map.resize(numberOfChannels);

    diffuse_map_computed.resize(numberOfChannels);
    specular_map_computed.resize(numberOfChannels);
}

lightProccessing::~lightProccessing(){

}

///SETS
void lightProccessing::set_xyzMap(Mat data, bool check){
    this->xyz_map = data.clone();

    std::vector<Mat> vecMerge;
    Mat xyzMP;

    if(check){
        for (int var = 0; var < 3; ++var) {


            std::vector<Mat> temp;
            split(xyz_map, temp);
            Mat Depth_Map = temp[var].clone();

            double min = 1000000;
            double max = -1000000;

            //find min and max

            for (int var1 = 0; var1 < Depth_Map.rows; ++var1) {
                for (int var2 = 0; var2 < Depth_Map.cols; ++var2) {
                    if (Depth_Map.at<float>(var1,var2) > max && Depth_Map.at<float>(var1,var2) != 0.0) {
                        max = Depth_Map.at<float>(var1,var2);
                    }
                    if (Depth_Map.at<float>(var1,var2) < min && Depth_Map.at<float>(var1,var2) != 0.0) {
                        min = Depth_Map.at<float>(var1,var2);
                    }
                }
            }

            Depth_Map -= min;
            max -= min;

            Depth_Map /= max;
            vecMerge.push_back(Depth_Map.clone());
        }
        merge(vecMerge, xyzMP);
        imshow("depth_map", xyzMP);
        imwrite("Output/Maps/xyzMap.jpg", (xyzMP * 255));
        waitKey(0);
    }
    cv::Vec3f pointMid = xyz_map.at<cv::Vec3f>(xyz_map.rows/2, xyz_map.cols/2);
    cv::Vec3f pointTop = xyz_map.at<cv::Vec3f>(0, xyz_map.cols/2);
    //    cv::Vec3f pointMid = xyz_map.at<cv::Vec3f>(xyz_map.rows/2, xyz_map.cols/2);
    //    cv::Vec3f projP(-0.40537274, -1.8238552, 1.3371408);
    //    std::cout << "point " << cv::normalize(point-projP) << std::endl;
    std::cout << "point mid " << pointMid << std::endl;
    std::cout << "point top" << pointTop << std::endl;
    //    std::cout << "point " << pointMid << std::endl;

    return;
}

void lightProccessing::set_normal(float data, int i, int j){
    this->normals.at<float>(i,j) = data;
    return;
}

void lightProccessing::set_normals(Mat data, bool check){
    this->normals = data.clone();
    if(check){
        namedWindow("normals", WINDOW_NORMAL);
        imshow("normals", cv::abs(normals));
        //change to RGB
        std::vector<Mat> vec, vec2;
        vec2.resize(3);
        cv::split(normals, vec);
        Mat res;
        vec2[0] = -vec[2].clone();
        vec2[1] = -vec[1].clone();
        vec2[2] = -vec[0].clone();
        merge(vec2,res);

        imwrite("Output/Maps/NormalMap.jpg", (res * 255));
        waitKey(0);
    }
    return;
}

void lightProccessing::set_alpha(float data){
    this->alpha = data;
    return;
}

void lightProccessing::load_matches(bool check){
    std::cout << "Loading matches... \n";
    this->matches_points.resize(numberOfCameras);
    for (int camID = 0; camID < numberOfCameras ; ++camID) {
        std::string path = "data/data/matches" +  std::to_string(camID) + ".data";
        File_Manager fm("Output/");
        std::vector<Match> temp;
        fm.import_matches(path, temp);
        matches_points[camID] = temp;
    }
    std::cout << "Done!\n";
    /**check input**/
    if(check){
        for (int camId = 0; camId < numberOfCameras; ++camId) {

            //Mask Image
            Mat mask_test(get_resolution(), CV_8UC1, Scalar(0));
            for (int i = 0; i < matches_points[camId].size(); ++i) {
                if (matches_points[camId][i].points.size() > 0) {
                    //                    mask_test.at<uchar>((get_height()-1) - (i/get_width()), i%get_width()) = 255;////////////////////////////////alex
                    mask_test.at<uchar>(i%get_height(), i/get_height()) = 255;

                }else{
                    mask_test.at<uchar>(i%get_height(), i/get_height()) = 0;
                }
            }
            namedWindow("Mask", WINDOW_NORMAL);
            imshow("Mask", mask_test);
            waitKey(0);

            //Corespondances
            Mat Image(cameras[0].resolution, CV_8UC3, Scalar(0,0,0));
            Vec3b color (0 , 0 , 0);
            for (int pixelID = 0; pixelID < matches_points[camId].size(); ++pixelID) {
                int size = matches_points[camId][pixelID].points.size();
                color[0]++;
                if(color[0] >= 255){
                    color[1]++;
                    if(color[1] >= 255){
                        color[2]+=10;
                        if(color[2] >= 236){
                            std::cout << "ERROR:: LACK OF COLOR!\n";
                        }
                    }
                }
                for (int j = 0; j < size; ++j) {
                    int col = matches_points[camId][pixelID].points[j].x;
                    int row = matches_points[camId][pixelID].points[j].y;
                    Image.at<Vec3b>(row, col) = color;
                }
            }
            namedWindow("image", WINDOW_NORMAL);
            imshow("image", Image);
            imwrite("Output/Matches/Matches" +  std::to_string(camId) + ".jpg", Image);
            waitKey(0);
        }
    }
    return;
}

///GETS
Size lightProccessing::get_resolution(){
    return projector.resolution;
}

float lightProccessing::get_alpha(){
    return this->alpha;
}

int lightProccessing::get_height(){
    return projector.resolution.height;
}

int lightProccessing::get_width(){
    return projector.resolution.width;
}

float lightProccessing::get_cosNL(int i, int j){
    return cosNL_map.at<float>(i,j);
}

float lightProccessing::get_cosRV(int i, int j, int camera_number){
    return cosRV_map[camera_number].at<float>(i,j);
}

Vec3f lightProccessing::get_normal(int i, int j){
    return normals.at<Vec3f>(i,j);
}

Vec3f lightProccessing::get_point(int i, int j){
    return xyz_map.at<Vec3f>(i,j);
}

//Vec3f lightProccessing::get_lightSource_Vec3f(int i, int j){
//    return light_source.at<Vec3f>(i,j);
//}

float lightProccessing::get_grayKd(int i, int j){
    return diffuse_map_grayScale.at<float>(i,j);
}

float lightProccessing::get_grayKs(int i, int j){
    return specular_map_grayScale.at<float>(i,j);
}

float lightProccessing::get_lightSource_grayScale(int i, int j){
    return light_source_grayScale.at<float>(i,j);
}

//Color lightProccessing::get_lightSource(int i, int j){
//    Vec3f temp_vec = light_source.at<Vec3f>(i,j);
//    Color temp(temp_vec[2], temp_vec[1], temp_vec[0]);
//    return temp;
//}

float lightProccessing::get_type(int i, int j){
    return type_table.at<float>(i,j);
}

bool lightProccessing::is_Node(int i, int j){
    if (Mask.at<float>(i,j) == 0.0f) {
        return false;
    }
    return true;
}

bool lightProccessing::is_typeX(int i, int j, int type){
    if(Mask_Type[type-1].at<float>(i,j) == 1){
        return true;
    }
    return false;
}

///COMPUTATIONS
void lightProccessing::initialize_colors( std::string path, bool check){
    Mat projector_obsL(get_resolution(), CV_8UC3, cv::Scalar(0));
    std::cout << "Initializing Colors...\n";
    Mat tempX(get_resolution(), CV_8UC1, Scalar(0));
    for (int camID = 0; camID < numberOfCameras; ++camID) {
        Mat input_Mat = imread(path +  std::to_string(camID) + ".png");
        Image input_Image(input_Mat);
        std::cout << "\tColor has been loaded!" <<  std::endl;
        for (int i = 0; i < matches_points[camID].size(); ++i) {
            int u = i/get_height();
            int v = i%get_height();
            Color meanC(0.0, 0.0, 0.0);
            Match m  = matches_points[camID][i];
            if(m.should_count){
                tempX.at<uchar>(v,u) = 255;
                for (int clr = 0; clr < m.points.size(); ++clr) {
                    meanC += input_Image.getPixel(m.points[clr].x , m.points[clr].y);
                    matches_points[camID][i].Colors.push_back(input_Image.getPixel(m.points[clr].x , m.points[clr].y));
                }
                meanC = meanC / m.points.size();
                matches_points[camID][i].mean_Color = meanC;
            }else if(xyz_map.at<Vec3f>(v, u) != Vec3f(0.0,0.0,0.0)){
                tempX.at<uchar>(v,u) = 125;

                std::vector<Point3f> Objpoint;
                std::vector<Point2f> Imgpoint;
                Mat R;
                Vec3f point_P = xyz_map.at<Vec3f>(v, u);
                Point3f point_projector(point_P[0], point_P[1], point_P[2]);
                Objpoint.push_back(point_projector);
                Rodrigues(cameras[camID].rotation, R);
                projectPoints(Objpoint, R, cameras[camID].translation, cameras[camID].intrinsics, cameras[camID].distortion_coeffs, Imgpoint);
                matches_points[camID][i].mean_Color = input_Image.getPixel(int(Imgpoint[0].x), int(Imgpoint[0].y));
            }else{
                matches_points[camID][i].mean_Color = meanC;
            }
            //            projector_obsL.at<vec3
        }
        for (int i = 0; i < get_width(); ++i) {
            for (int j = 0; j < get_height(); ++j) {
                int id = i * get_height() + j;
                if(matches_points[camID][id].should_count || xyz_map.at<Vec3f>(j, i) != Vec3f(0.0,0.0,0.0)){
                    Color temp =  matches_points[camID][id].mean_Color;
                    Observed_map[camID * numberOfChannels + 0].at<float>(j, i) = temp.r()/255.0;
                    Observed_map[camID * numberOfChannels + 1].at<float>(j, i) = temp.g()/255.0;
                    Observed_map[camID * numberOfChannels + 2].at<float>(j, i) = temp.b()/255.0;
                }
            }
        }
        std::cout << "\tColors has been assigned!" <<  std::endl;
    }
    Mat tempDiffuse(get_resolution(),CV_8UC3, Scalar(0, 0, 0));
    for (int i = 0; i < get_height(); ++i) {
        for (int j = 0; j < get_width(); ++j) {
            float colorR = 0, colorG=0, colorB=0;
            float counter =0;
            for (int camID = 0; camID < numberOfCameras; ++camID) {
                if (!(Observed_map[camID * numberOfChannels + 0].at<float>(i,j) == 0 &&  Observed_map[camID * numberOfChannels + 1].at<float>(i,j) == 0 && Observed_map[camID * numberOfChannels + 2].at<float>(i,j) == 0)) {
                    colorR += Observed_map[camID * numberOfChannels + 0].at<float>(i,j);
                    colorG += Observed_map[camID * numberOfChannels + 1].at<float>(i,j);
                    colorB += Observed_map[camID * numberOfChannels + 2].at<float>(i,j);
                    counter++;
                }
            }
            tempDiffuse.at<Vec3b>(i,j) = Vec3b(int(255*colorB/counter), int(255*colorG/counter), int(255*colorR/counter));
        }
    }
    if (check) {
        for (int it = 0; it < numberOfCameras*numberOfChannels; ++it) {
            std::string name = "Camera " +  std::to_string(it/numberOfCameras) + " Cahnnel " +  std::to_string(it%numberOfCameras);
            namedWindow(name , WINDOW_NORMAL);
            imshow(name, Observed_map[it]);
            //            waitKey(0);
        }
        Mat RGB;
        cv::vector<Mat> x;
        x.push_back(Observed_map[5].clone());
        x.push_back(Observed_map[4].clone());
        x.push_back(Observed_map[3].clone());
        merge(x,RGB);
        imshow("rgb", RGB);
        imwrite("rgb.jpg", RGB*255);
        waitKey(0);
    }
    std::cout << "Done!\n";
    return;
}

void lightProccessing::compute_Mask_Image(bool check){
    int counter = 0;
    Mat temp(get_resolution(), CV_32FC1, Scalar(0.0f));
    for (int i = 0; i < get_height(); ++i) {
        for (int j = 0; j < get_width(); ++j) {
            if (get_normal(i,j) != Vec3f(0.0f, 0.0f, 0.0f) && get_cosNL(i,j) > 0) {
                temp.at<float>(i,j) = 1.0f;
                counter++;
            }
        }
    }
    Mask = temp.clone();
    nodes = counter;
    imwrite("Output/Masks/Main_Mask.jpg", Mask * 255);
    if(check){
        std::cout << "Number of Nodes = " << nodes <<  std::endl;
        namedWindow("Mask Image", WINDOW_NORMAL);
        imshow("Mask Image", Mask);
        waitKey(0);
    }
    return;
}

void lightProccessing::compute_CosNL(bool check){
    std::cout << "Computing CosNL ...\t";
    if(!this->normals.empty() && !this->xyz_map.empty()){

        Ray L_vector;
        L_vector.origin = Mat(3,1,CV_32FC1);
        L_vector.dir = Mat(3,1,CV_32FC1);
        L_vector.origin = projector.get_world_coordinates().clone();

        for (int i = 0; i < get_height(); ++i) {
            for (int j = 0; j < get_width(); ++j) {

                Vec3f normal = get_normal(i,j);
                if(normal != Vec3f(0.0f, 0.0f, 0.0f)){
                    //by the xyzmap find destination => direction,
                    Vec3f xyz_temp = get_point(i,j);
                    L_vector.dir.at<float>(0,0) = L_vector.origin.at<float>(0,0) - xyz_temp[0];
                    L_vector.dir.at<float>(1,0) = L_vector.origin.at<float>(1,0) - xyz_temp[1];
                    L_vector.dir.at<float>(2,0) = L_vector.origin.at<float>(2,0) - xyz_temp[2];

                    normalize(L_vector.dir, L_vector.dir);
                    //by geo object get the normal.

                    Mat normal_Mat(3,1,CV_32FC1);
                    normal_Mat.at<float>(0,0) = normal[0];
                    normal_Mat.at<float>(1,0) = normal[1];
                    normal_Mat.at<float>(2,0) = normal[2];

                    float nDOTl = normal_Mat.dot(L_vector.dir);

                    if(nDOTl>0){
                        cosNL_map.at<float>(i,j) = nDOTl;
                    } else if(nDOTl > 1){
                        std::cout << "Error:: Cos T > 1!\n";
                    }else{
                        cosNL_map.at<float>(i,j) = 0;
                    }
                }
            }
        }
        std::cout << "Done!\n";
    }else{
        std::cout << "compute_CosNL :: Error ~ normals or xyz_map is empty!\n;";
    }

    if(check){
        namedWindow("cosNL", WINDOW_NORMAL);
        imshow("cosNL", cosNL_map);
        imwrite("Output/Maps/cosNL_map.jpg", cosNL_map * 255);

        waitKey(0);
    }
    return;
}

void lightProccessing::compute_CosRVs(bool check){

    std::cout << "Computing cosRV...\t";
    Mat rdv(get_resolution(), CV_32FC1, Scalar(0.0f));

    Ray L_vector, V_vector;

    L_vector.origin = Mat(3,1,CV_32FC1);
    L_vector.dir = Mat(3,1,CV_32FC1);

    V_vector.dir = Mat(3,1,CV_32FC1);

    // set origin:: location of the projector
    L_vector.origin = this->projector.get_world_coordinates();
    //    std::cout << projector.get_world_coordinates() << std:: endl;
    //    std::cout << this->cameras[1].name << this->cameras[1].get_world_coordinates() << std::endl;

    std::cout << "compute CosRV :: Warning:: Check for the bug here. the camera number is not right!\n";
    Ux.resize(numberOfCameras);
    Uy.resize(numberOfCameras);
    Uz.resize(numberOfCameras);

    Vx.resize(numberOfCameras);
    Vy.resize(numberOfCameras);
    Vz.resize(numberOfCameras);

    for (int camID = 0; camID < numberOfCameras; ++camID) {
        Ux[camID] = Mat(get_resolution(), CV_32FC1, Scalar(0.0)).clone();
        Uy[camID] = Mat(get_resolution(), CV_32FC1, Scalar(0.0)).clone();
        Uz[camID] = Mat(get_resolution(), CV_32FC1, Scalar(0.0)).clone();

        Vx[camID] = Mat(get_resolution(), CV_32FC1, Scalar(0.0)).clone();
        Vy[camID] = Mat(get_resolution(), CV_32FC1, Scalar(0.0)).clone();
        Vz[camID] = Mat(get_resolution(), CV_32FC1, Scalar(0.0)).clone();

        V_vector.origin = this->cameras[camID].get_world_coordinates();

        for (int i = 0; i < get_height(); ++i) {
            for (int j = 0; j < get_width(); ++j) {

                if (is_Node(i,j)) {
                    Vec3f normal = get_normal(i,j);
                    if(normal != Vec3f(0.0f, 0.0f, 0.0f)){

                        //by the xyzmap find destination => direction,
                        Vec3f xyz_temp = get_point(i,j);

                        L_vector.dir.at<float>(0,0) = xyz_temp[0] - L_vector.origin.at<float>(0,0);
                        L_vector.dir.at<float>(1,0) = xyz_temp[1] - L_vector.origin.at<float>(1,0);
                        L_vector.dir.at<float>(2,0) = xyz_temp[2] - L_vector.origin.at<float>(2,0);

                        V_vector.dir.at<float>(0,0) = V_vector.origin.at<float>(0,0) - xyz_temp[0];
                        V_vector.dir.at<float>(1,0) = V_vector.origin.at<float>(1,0) - xyz_temp[1];
                        V_vector.dir.at<float>(2,0) = V_vector.origin.at<float>(2,0) - xyz_temp[2];

                        normalize(L_vector.dir, L_vector.dir);
                        normalize(V_vector.dir, V_vector.dir);

                        Ux[camID].at<float>(i,j) = -L_vector.dir.at<float>(0,0);
                        Uy[camID].at<float>(i,j) = -L_vector.dir.at<float>(1,0);
                        Ux[camID].at<float>(i,j) = -L_vector.dir.at<float>(2,0);

                        Vx[camID].at<float>(i,j) = V_vector.dir.at<float>(0,0);
                        Vy[camID].at<float>(i,j) = V_vector.dir.at<float>(1,0);
                        Vx[camID].at<float>(i,j) = V_vector.dir.at<float>(2,0);

                        //by geo object get the normal.
                        Mat normal_Mat(3,1,CV_32FC1);
                        normal_Mat.at<float>(0,0) = normal[0];
                        normal_Mat.at<float>(1,0) = normal[1];
                        normal_Mat.at<float>(2,0) = normal[2];

                        //compute n.l
                        float nDOTl = normal_Mat.dot(L_vector.dir);

                        Mat reflection = (L_vector.dir) - ((2 * nDOTl) * normal_Mat);

                        float rDOTv = reflection.dot(V_vector.dir);

                        if(rDOTv > 0){
                            rdv.at<float>(i,j) = rDOTv;
                        } else if(rDOTv > 1){
                            std::cout << "Error:: CosRV > 1!\n";
                        }else{
                            rdv.at<float>(i,j) = 0;
                        }
                    }
                }else{
                    rdv.at<float>(i,j) = 0;
                }
            }
        }

        cosRV_map[camID] = rdv.clone();
        imwrite("Output/Maps/cosRV_map[" +  std::to_string(camID) + "].jpg", cosRV_map[camID] * 255);

        if(check){
            namedWindow("CosRV", WINDOW_NORMAL);
            imshow("CosRV", cosRV_map[camID]);
            std::printf("showing CosRV [%i] \n", camID);
            waitKey(1);
        }

    }
    std::cout << "Done!\n";
    return;
}

void fcn_Alpha(int m, int n, double *x, double *fvec, int *iflag, double params[]){

    float D;
    float CphiL;
    float CphiR;
    float Il;
    float Ir;
    float ap = x[0];
    int j;
    for (int i = 0; i < m; ++i) {
        j = i * 5;
        D = parameters[j];
        CphiL = parameters[j+1];
        CphiR = parameters[j+2];
        Il = parameters[j+3];
        Ir = parameters[j+4];
        float x1 = pow(CphiL, ap) / pow(CphiR, ap);
        float x2 = (Il - D)/(Ir - D);
        float res =  pow(((pow(CphiL, ap) / pow(CphiR, ap)) - ( std::abs((Il - D)/(Ir - D)))) ,1);
        fvec[i] = pow(((pow(CphiL, ap) / pow(CphiR, ap)) - ( std::abs((Il - D)/(Ir - D)))) ,1);
        if( std::isnan(fvec[i]) ||  std::isinf(fvec[i])){
            std::cout << "WAIT!!!\n";
        }
    }
    return;
}

void lightProccessing::compute_kd(Mat Intensity, Mat LightSource, Mat &Kd_output, bool check){

    Mat Kd_Mat(get_resolution(), CV_32FC1);
    float I, L;

    for (int i = 0; i < get_height(); ++i) {
        for (int j = 0; j < get_width(); ++j) {

            if(is_Node(i,j)){
                I = Intensity.at<float>(i,j);
                L = LightSource.at<float>(i,j);

                float kd = I / (get_cosNL(i,j) * L);
                if (kd < 0) {
                    kd = 0;
                    std::cout << "Kd is less than 0!\n";
                }
                Kd_Mat.at<float>(i,j) = kd;

                if( std::isnan(kd) ||  std::isinf(kd)){
                    std::cout << " i , j " << i << "  " << j <<  std::endl;
                    std::cout << " Intensity " <<  I <<  std::endl;
                    std::cout << " get_cosNL(i,j) " <<  get_cosNL(i,j) <<  std::endl;
                    std::cout << " L " << L <<  std::endl;
                    std::cout << " -------------------------------- "  <<  std::endl;
                }
            }
        }
    }
    if(check){
        namedWindow("compute_kd", WINDOW_NORMAL);
        imshow("compute_kd", Kd_Mat);
        waitKey(0);
    }
    Kd_output = Kd_Mat.clone();
    return;
}


//Phong Model
/*
void fcn_DS(int m, int n, double *x, double *fvec, int *iflag, double params[])
{
    double light_weight = 1;
    double L, cosT, I, cosPhi, Ks, Kd, alpha, cosPhi_Alpha;

    //    double neighbour_Error;
    //    double neighbour_weight = 1;
    //    int window_size = 5;

    int i, j;
    i = (int) params[0];
    j = (int) params[1];

    Kd = x[0];
    Ks = x[1];
    alpha = x[2];

    L       = global_light.at<float>(i,j);
    cosT    = global_cosNL.at<float>(i,j);

    //    std::printf(" Kd \t :: %f \n", Kd);
    //    std::printf(" Ks \t :: %f \n", Ks);
    //    std::printf(" L  \t :: %f \n", L );
    //    std::printf(" alpha \t :: %f \n", alpha);

    // Apply the limitations
    if(Kd > 1 || Kd < 0 || Ks > 1 || Ks < 0 || alpha <= 0.1 || alpha > 50){
        for (int var = 0; var < m; ++var) {
            fvec[var] =  pow(2,32);
        }
    }
    // Check data validity
    else if(std::isnan(Kd) ||  std::isnan(Ks) ||  std::isnan(alpha) ||  std::isinf(Kd) ||  std::isinf(Ks) ||  std::isinf(alpha)){
        for (int var = 0; var < m; ++var) {
            fvec[var] =  -pow(2,32);
        }
    }else{
        for (int camID = 0; camID < m; ++camID) {
            I = global_Observe[camID].at<float>(i,j);
            cosPhi = global_cosRV[camID].at<float>(i,j);
            cosPhi_Alpha = pow(cosPhi,alpha);
            fvec[camID] = pow(((( L * Kd * cosT + L * Ks * cosPhi_Alpha )) - (I)),2) * light_weight;

            //            neighbour_Error = 0;
            //            for (int p = -window_size+1; p < window_size; ++p) {
            //                for (int q = -window_size+1; q < window_size; ++q) {
            //                    if(global_mask.at<float>(i+p, j+q) == 1 && (p != 0 && q != 0)){
            //                        neighbour_Error +=  std::abs(Kd - global_updated_kd.at<float>(i+p,j+q)) * 20;
            //                        neighbour_Error +=  std::abs(Ks - global_updated_ks.at<float>(i+p,j+q)) * 20;
            //                        neighbour_Error +=  std::abs(alpha - global_updated_alpha.at<float>(i+p,j+q));
            //                    }
            //                }
            //            }
            //            fvec[camID] = pow(neighbour_Error, 2) * neighbour_weight;
        }
    }
    return;
}
*/

/*
//Lafortune Model
void fcn_DS(int m, int n, double *x, double *fvec, int *iflag, double params[])
{
    double L = 1, I, CosT, alpha;
    double Ks, Kd;
    double Cx, Cy, Cz;
    double Ux, Uy, Uz;
    double Vx, Vy, Vz;
    double Lambertian;
    double Lafortune;

    int i, j;
    i = (int) params[0];
    j = (int) params[1];

    Kd = x[0];
    Ks = x[1];
    Cx = x[2];
    Cy = x[3];
    Cz = x[4];
    alpha = x[5];

    L       = global_light.at<float>(i,j);
    CosT    = global_cosNL.at<float>(i,j);

    // Apply the limitations
    if(Kd > 1 || Kd < 0 || Ks > 1 || Ks < 0 || alpha <= 0.1 || alpha > 50){
        for (int var = 0; var < m; ++var) {
            fvec[var] =  pow(2,32);
        }
    }

    // Check data validity
    else if(std::isnan(Kd) ||  std::isnan(Ks) ||  std::isnan(alpha) ||  std::isinf(Kd) ||  std::isinf(Ks) ||  std::isinf(alpha)){
        for (int var = 0; var < m; ++var) {
            fvec[var] =  -pow(2,32);
        }
    }

    // does the job
    else{
        for (int camID = 0; camID < m; ++camID) {
            Ux = global_Ux[camID].at<float>(i,j);
            Uy = global_Uy[camID].at<float>(i,j);
            Uz = global_Uz[camID].at<float>(i,j);

            Vx = global_Vx[camID].at<float>(i,j);
            Vy = global_Vy[camID].at<float>(i,j);
            Vz = global_Vz[camID].at<float>(i,j);

            I = global_Observe[camID].at<float>(i,j);
            Lambertian = L * Kd * CosT;
            Lafortune = L * Ks * pow(( Cx*Ux*Vx + Cy*Uy*Vy + Cz*Uz*Vz ), alpha);
            fvec[camID] = pow((Lambertian + Lafortune)-I, 2);
        }
    }
    return;
}
*/

// Phong Model
void fcn_DS(int m, int n, double *x, double *fvec, int *iflag, double params[])
{
    float light_weight = 1;
    float L, cosT, I, cosPhi, Ks, Kd, alpha, cosPhi_Alpha;
    float Error = 0;
    int i, j;
    i = (int) params[0];
    j = (int) params[1];

    Kd = x[0];
    Ks = x[1];
    alpha = x[2];

    L       = global_light.at<float>(i,j);
    cosT    = global_cosNL.at<float>(i,j);

    // Apply the limitations
    if(Kd > 1 || Kd < 0 || Ks > 1 || Ks < 0 || alpha <= 1 || alpha > 50){
        for (int var = 0; var < m; ++var) {
            fvec[var] =  pow(2,32);
        }
    }
    // Check data validity
    else if(std::isnan(Kd) ||  std::isnan(Ks) ||  std::isnan(alpha) ||  std::isinf(Kd) ||  std::isinf(Ks) ||  std::isinf(alpha)){
        for (int var = 0; var < m; ++var) {
            fvec[var] =  -pow(2,32);
        }
    }else{
        for (int camID = 0; camID < m; ++camID) {
            I = global_Observe[camID].at<float>(i,j);
            if (I == 0) {
                fvec[camID] = 0;
            } else {
                cosPhi = global_cosRV[camID].at<float>(i,j);
                cosPhi_Alpha = pow(cosPhi,alpha);
                fvec[camID] = pow(((( L * Kd * cosT + L * Ks * cosPhi_Alpha )) - (I)),2);
            }

            //            std::cout << "I\t\t = " << I << std::endl;
            //            std::cout << "cosT\t\t = " << cosT << std::endl;
            //            std::cout << "cosPhi\t\t = " << cosPhi << std::endl;
            //            std::cout << "Alpha\t\t = " << alpha << std::endl;
            //            std::cout << "Kd\t\t = " << Kd << std::endl;
            //            std::cout << "Ks\t\t = " << Ks << std::endl;
            //            std::cout << "term error\t\t = " << fvec[camID] << std::endl;
            //            Error += fvec[camID];
        }
    }

    //    std::cout << "Error :: " << Error <<std::endl;
    //    std::cout << "fcn_DS is Done!.\n\n";
    return;
}



void static Levenberg_Marquardt_Driver(int rangeL, int rangeH){

    Size resolution = global_cosNL.size();
    int numberOfCameras = global_Observe.size();
    global_updated_kd = Mat(resolution, CV_32FC1, Scalar(0.5));
    global_updated_ks = Mat(resolution, CV_32FC1, Scalar(0.5));
    global_updated_alpha = Mat(resolution, CV_32FC1, Scalar(14));

    //    int rangewL = resolution.height/2;
    //    int rangewH = rangewL + 100;

    //    int rangehL = get_height()/2;
    //    int rangehH = get_height()/2 + 100;


    //        int rangewL = 400;
    //        int rangewH = 500;

    //        int rangehL = 200;
    //        int rangehH = 300;


    for (int pixel_i = rangeL; pixel_i < rangeH; ++pixel_i) {
        std::cout << "|" << std::flush;
        for (int pixel_j = 0; pixel_j < resolution.width; ++pixel_j) {

            //    for (int pixel_i = rangeL; pixel_i < rangeH; ++pixel_i) {
            //        std::cout << "|" << std::flush;
            //        for (int pixel_j = rangewL; pixel_j < rangewH; ++pixel_j) {


            if(global_mask.at<float>(pixel_i,pixel_j) == 1.0){ // /* && get_type(pixel_i, pixel_j) == 8*/

                int m = numberOfCameras; // number of observes
                int n = 3; // number of variables
                int info;
                int nfev;
                int total;

                double Sum = 0.0;
                double newError = 0.0;
                double oldError = 0.0;

                double * fvec = new double[m];
                double * x = new double[n];
                double tol = sqrt ( dpmpar[0] );
                double params[2];
                int k=0;

                x[0] = global_updated_kd.at<float>(pixel_i,pixel_j);
                x[1] = global_updated_ks.at<float>(pixel_i,pixel_j);
                x[2] = global_updated_alpha.at<float>(pixel_i,pixel_j);

                params[0] = pixel_i; // pixel row
                params[1] = pixel_j; // pixel col

                //not sure what this is
                int *msk = new int[n];
                for ( int i=0;i<n;i++ ) msk[i] = 1;

                //Call the energy function once
                fcn_DS ( m,n,x,fvec,0,params );

                Sum = 0.0;

                total = 0 ;
                for ( int j=0;j<m;j++ )
                {
                    Sum +=fvec[j]*fvec[j];
                }

                oldError = sqrt ( Sum/ ( ( double ) m ) );


                //call the optimizer
                for ( int i=0;i<20;i++ )
                {
                    //printf ( "LM...\n" );
                    lmdif0 ( fcn_DS,m,n,x,msk,fvec,tol,&info,&nfev,params );

                    //printf ( "GD...\n" );
                    gd0 ( fcn_DS,m,n,x,msk,fvec,tol,&info,&nfev,params );

                    //printf ( "DS...\n" );
                    ds0 ( fcn_DS,m,n,x,msk,fvec,tol,&info,&nfev,params );
                }

                Sum = 0.0;
                total =0 ;
                for ( int j=0;j<m;j++ )
                {
                    Sum +=fvec[j]*fvec[j];
                }
                newError = sqrt ( Sum/ ( ( double ) m ) );

                //set the final matrix
                k=0;

                if(std::isnan(x[0]) || std::isnan(x[1]) || std::isnan(x[2])){
                    std::cout << "NAN Found!\n";
                }
                if(std::isinf(x[0]) || std::isinf(x[1]) || std::isinf(x[2])){
                    std::cout << "INF Found!\n";
                }

                //                std::printf("APP: diffuse :: %f\n", x[0]);
                //                std::printf("APP: specular :: %f\n", x[1]);

                global_diffue.at<float>(pixel_i,pixel_j) = x[0];
                global_specular.at<float>(pixel_i,pixel_j) = x[1];
                global_alpha.at<float>(pixel_i,pixel_j) = x[2];

                delete [] x;
                delete [] fvec;
                delete [] msk;

            }
        }
    }
    return;
}

void lightProccessing::render_channel(Mat Diffuse, Mat Specular, Mat light, int camera_number, Mat &output, bool check){
    Mat rendered_Channel = Diffuse.mul(cosNL_map.mul(light));
    Mat powered_cos;
    pow(cosRV_map[camera_number], alpha, powered_cos);
    rendered_Channel += Specular.mul(light.mul(powered_cos));
    if(check){
        namedWindow("Rendered Image", WINDOW_NORMAL);
        imshow("Rendered Image", rendered_Channel);
        waitKey(0);
    }
    output = rendered_Channel.clone();
    return;
}

Mat lightProccessing::render_image_grayScale(bool check){

    Mat rendered_Image = diffuse_map_grayScale.mul(cosNL_map.mul(light_source_grayScale));

    if(check){
        namedWindow("Rendered Image", WINDOW_NORMAL);
        imshow("Rendered Image", rendered_Image);
        waitKey(0);
    }
    return rendered_Image.clone();
}

Mat lightProccessing::render_image_grayScale(Mat Diffuse, Mat Specular, int camera_number, bool check){

    Mat rendered_Image = Diffuse.mul(cosNL_map.mul(light_source_grayScale));
    Mat powered_cos;
    power_MatbyMat(cosRV_map[camera_number], Alpha_grayScale, powered_cos);
    rendered_Image += Specular.mul(light_source_grayScale.mul(powered_cos));
    if(check){
        namedWindow("Rendered Image", WINDOW_NORMAL);
        imshow("Rendered Image", rendered_Image);
        waitKey(0);
    }
    return rendered_Image.clone();
}

Mat lightProccessing::render_image_grayScale(int mode, int camera_number, bool check){
    Mat rendered_Image;
    switch(mode){
    case 0:
        //Diffuse
        rendered_Image = diffuse_map_grayScale.mul(cosNL_map.mul(light_source_grayScale));
        break;
    case 1:
    {
        //specular
        Mat powered_cos;
        pow(cosRV_map[camera_number], alpha, powered_cos);
        rendered_Image = specular_map_grayScale.mul(light_source_grayScale.mul(powered_cos));
        break;
    }
    case 2:
    {
        //specular + Diffuse
        rendered_Image = diffuse_map_grayScale.mul(cosNL_map.mul(light_source_grayScale));
        Mat powered_cos;
        pow(cosRV_map[camera_number], alpha, powered_cos);
        rendered_Image += specular_map_grayScale.mul(light_source_grayScale.mul(powered_cos));
        break;
    }
    }
    if(check){
        namedWindow("Rendered Image", WINDOW_NORMAL);
        imshow("Rendered Image", rendered_Image);
        waitKey(0);
    }
    return rendered_Image.clone();
}

void lightProccessing::create_testCase(bool check){

    Mat Intensity(get_resolution(), CV_32FC1, Scalar(0.0));
    for (int camId = 0; camId < numberOfCameras; ++camId) {
        vector<Mat> ChannelIntensity;
        ChannelIntensity.resize(numberOfChannels);
        for (int channelID = 0; channelID < numberOfChannels; ++channelID) {
            Mat powerd;
            pow(cosRV_map[camId], (double)get_alpha(), powerd);
            Mat d(get_resolution(), CV_32FC1, Scalar(0.4));
            Intensity = diffuse_map[channelID].mul(cosNL_map) + specular_map[channelID].mul(powerd);
            //            Intensity = kd[channelID] * cosNL_map + ks[channelID] * powerd;

            this->Observed_map[(camId*numberOfChannels)+channelID] = Intensity.clone();
            ChannelIntensity[channelID] = Intensity.clone();
        }
        if(check && numberOfChannels > 1){
            Mat channel;
            merge(ChannelIntensity, channel);
            namedWindow("Test Case", WINDOW_NORMAL);
            imshow("Test Case", channel);
            imwrite("Output/Maps/Object[" +  std::to_string(camId) + "].jpg", (channel * 255));
            waitKey(0);
        }
    }
    return;
}

void lightProccessing::create_MacbetChart_testCase(bool check){

    Mat Intensity(get_resolution(), CV_32FC1, Scalar(0.0));
    int id_Kd = 0;
    int id_Ks = 0;
    int row, col;
    for (int camId = 0; camId < numberOfCameras; ++camId) {
        vector<Mat> ChannelIntensity;
        ChannelIntensity.resize(numberOfChannels);
        for (int channelID = 0; channelID < numberOfChannels; ++channelID) {
            for (float i = 0; i < get_height(); ++i) {
                for (float j = 0; j < get_width(); ++j) {
                    row = (i/get_height() * 24);
                    col = (j/get_width() * 24);
                    id_Kd = (col + row * 6)%24;
                    id_Ks = 23 - id_Kd;
                    if(is_Node(i,j)){
                        Intensity.at<float>(i,j) = (1 * Macbeth[channelID][id_Kd] * get_cosNL(i,j) + 1 * Macbeth[channelID][id_Ks] * pow((get_cosRV(i,j,camId)), get_alpha()))/255.0;
                    }
                }
            }
            this->Observed_map[(camId*numberOfChannels)+channelID] = Intensity.clone();
            ChannelIntensity[channelID] = Intensity.clone();
        }
        if(check && numberOfChannels > 1){
            Mat channel;
            merge(ChannelIntensity, channel);
            namedWindow("MacBeth Chart", WINDOW_NORMAL);
            imshow("MacBeth Chart", channel);
            imwrite("Output/Maps/MacBethChart[" +  std::to_string(camId) + ".jpg", channel * 255);
            waitKey(0);
        }
    }

    if (check) {
        for (int Id = 0; Id < numberOfCameras*numberOfChannels; ++Id) {
            namedWindow("Camera Observe macberth chart", WINDOW_NORMAL);
            imshow("Camera Observe macberth chart", Observed_map[Id]);
            waitKey(0);
        }
    }
    return;
}

void lightProccessing::project_image_on_mesh(vector<Mat> Inp_Image, vector<Mat> &Out_Image, bool check){

    int chNumber = Inp_Image.size();
    Out_Image.resize(chNumber);
    Mat Intensity(get_resolution(), CV_32FC1, Scalar(0.0));
    Out_Image.resize(chNumber);
    for (int channelID = 0; channelID < chNumber; ++channelID) {
        Intensity = Inp_Image[channelID].mul(cosNL_map);
        Out_Image[channelID] = Intensity.clone();
    }
    if(check){
        Mat channel;
        if (chNumber > 1) {
            merge(Out_Image, channel);
        } else {
            channel = Out_Image[0].clone();
        }
        namedWindow("Expected", WINDOW_NORMAL);
        imshow("Expected", channel);
        imwrite("Output/Expected.jpg", channel * 255);
        waitKey(0);
    }
    return;
}

void lightProccessing::create_grayScale_test(bool check){

    Mat image(get_resolution(), CV_32FC1);

    float L;
    float NL;

    for (int camId = 0; camId < numberOfCameras; ++camId) {
        for (float i = 0; i < get_height(); ++i) {
            for (float j = 0; j < get_width(); ++j) {

                if(is_Node(i,j)){
                    L = get_lightSource_grayScale(i,j);
                    NL = get_cosNL(i,j);

                    image.at<float>(i,j) =   L * kd[0] * NL + L * ks[0] * pow(get_cosRV(i,j,camId), alpha);
                }else{
                    image.at<float>(i,j) = 0;
                }
            }
        }
        Observed_map[camId] = image.clone();
    }

    if (check) {
        for (int camID = 0; camID < numberOfCameras; ++camID) {
            namedWindow("GrayScale test", WINDOW_NORMAL);
            imshow("GrayScale test", Observed_map[camID]);
            waitKey(0);
        }

    }
    return;
}

void lightProccessing::fill_type_table(bool check){

    Mat table(get_resolution(), CV_32FC1, Scalar(1.0f));
    for (int i = 0; i < get_height(); ++i) {
        for (int j = 0; j < get_width(); ++j) {
            if(is_Node(i,j)){
                if (get_cosRV(i,j,0) > 0.05){
                    table.at<float>(i,j) += 4;
                }
                if (get_cosRV(i,j,1) > 0.05) {
                    table.at<float>(i,j) += 2;
                }
                if (get_cosRV(i,j,2) > 0.05) {
                    table.at<float>(i,j) += 1;
                };
            }
        }
    }
    this->type_table = table.clone();
    return;
}

void lightProccessing::compute_mask_type(bool check){

    int tp = 0;
    for (int i = 0; i < get_height(); ++i) {
        for (int j = 0; j < get_width(); ++j) {
            if(is_Node(i,j)){
                tp = get_type(i,j);
                Mask_Type[tp-1].at<float>(i,j) = 1;
            }
        }
    }

    if(check){
        for (int i = 0; i < pow(2, numberOfCameras); ++i) {
            namedWindow("MaskType"+ std::to_string(i+1), CV_WINDOW_NORMAL);
            imshow("MaskType"+ std::to_string(i+1), Mask_Type[i]);
            imwrite("Output/Masks/MaskType"+ std::to_string(i+1)+".jpg", Mask_Type[i] * 255);
            waitKey(0);
        }
    }
    return;
}

void lightProccessing::compute_ASD_parameters(bool check){
    std::cout << "Optimization started!\n";

    global_cosNL = cosNL_map.clone();
    global_cosRV = cosRV_map;
    global_mask = Mask.clone();
    global_diffue = Mat(get_resolution(), CV_32FC1, Scalar(0));
    global_specular = Mat(get_resolution(), CV_32FC1, Scalar(0));
    global_alpha = Mat(get_resolution(), CV_32FC1, Scalar(0));
    global_light = Mat(get_resolution(), CV_32FC1, Scalar(1.0));
    global_Observe.resize(numberOfCameras);

    for (int channelID = 0; channelID < numberOfChannels; ++channelID) {

        for (int i = 0; i < numberOfCameras; ++i) {
            std::cout << "i am here ..." << i << std::endl;
            global_Observe[i] = Observed_map[(i * numberOfChannels) + channelID].clone();
        }

        int numberOfThreads = 8;
        std::thread *threads = new std::thread[numberOfThreads];
        int rangeHigh = 0, rangeLow = 0;
        ////////////////
        int difference = get_height()/numberOfThreads;
        rangeHigh = difference;
        for (int TrdIT = 0; TrdIT < numberOfThreads; ++TrdIT) {
            if (TrdIT == numberOfThreads - 1) {
                rangeHigh == get_height();
            }
            //            Levenberg_Marquardt_Driver(rangeLow, rangeHigh);
            threads[TrdIT] = std::thread(Levenberg_Marquardt_Driver, rangeLow, rangeHigh);
            rangeLow = rangeHigh;
            rangeHigh += difference;
        }

        for (int TrdIT = 0; TrdIT < numberOfThreads; ++TrdIT) {
            threads[TrdIT].join();
        }

        imwrite("diffuse"+ std::to_string(channelID) + ".jpg", global_diffue * 255.0);
        imwrite("specular"+ std::to_string(channelID) +".jpg", global_specular * 255.0);
        imwrite("alpha"+ std::to_string(channelID) +".jpg", global_alpha/50.0);

        diffuse_map_computed[channelID] = global_diffue.clone();
        specular_map_computed[channelID] = global_specular.clone();
        printf("\tOptimization done for channel %i .\n", channelID);

    }
    ///////////////////////////////////
    if (check) {
        //        for (int channelID = 0; channelID < numberOfChannels; ++channelID) {
        //            namedWindow("Diffuse Map Gray", WINDOW_NORMAL);
        //            namedWindow("Specular Map Gray", WINDOW_NORMAL);

        //            imshow("Diffuse Map", diffuse_map_computed[channelID]);
        //            imshow("Specular Map",  specular_map_computed[channelID]);
        //            imwrite("Output/LightProcessing/Diffuse" +  std::to_string(channelID) + ".jpg", diffuse_map_computed[channelID]*255.0);
        //            imwrite("Output/LightProcessing/Specular" +  std::to_string(channelID) + ".jpg", specular_map_computed[channelID]*255.0);

        //        }
        Mat merged_diffuse;
        Mat merged_specular;

        if (numberOfChannels > 1) {
            merge(diffuse_map_computed, merged_diffuse);
            merge(specular_map_computed, merged_specular);

        }else{
            merged_diffuse = diffuse_map_computed[0].clone();
            merged_specular = specular_map_computed[0].clone();
        }

        //        namedWindow("Specular", WINDOW_NORMAL);
        //        namedWindow("Diffuse", WINDOW_NORMAL);

        //        imshow("Specular", merged_specular);
        //        imshow("Diffuse", merged_diffuse);
        //        Mat rendered;
        //        render_image(diffuse_map_computed, specular_map_computed, light_source, cosNL_map, cosRV_map[1], Alpha_grayScale, rendered, true);

        imwrite("Output/LightProcessing/diffuse.png", merged_diffuse * 255.0);
        imwrite("Output/LightProcessing/specular.png", merged_specular * 255.0);
        //        imwrite("Output/LightProcessing/alpha.png", Alpha_grayScale);
        //        imwrite("Output/LightProcessing/rendered.png", rendered*255);

        //        waitKey(0);
    }
    std::cout << "ASD varibales computed! \n";
    return;
}

void lightProccessing::compute_compensate_image(bool check){

    //Expected Image
    Mat input_image = imread("data/maps/expected.jpg");
    input_image.convertTo(input_image,CV_32FC3);
    input_image /= 255.0;
    vector<Mat> expected_image_vec;
    split(input_image, expected_image_vec);
    vector<Mat> expectedMesh;
    project_image_on_mesh(expected_image_vec, expectedMesh, true);

    //set up
    vector<Mat> compensated_Image_vec;
    compensated_Image_vec.resize(numberOfChannels);
    for (int channelID = 0; channelID < numberOfChannels; ++channelID) {
        compensated_Image_vec[channelID] = Mat(get_resolution(), CV_32FC1, Scalar(0.0)).clone();
    }

    Mat grayMat(get_resolution(), CV_32FC1, Scalar(0.0));
    float grayish = 0;
    float difference = 0;
    float diffuse = 0;
    float specular = 0;
    float expected = 0;
    float maxExpectation = 0;

    for (int i = 0; i < get_height(); ++i) {
        for (int j = 0; j < get_width(); ++j) {
            if(is_Node(i,j)){
                grayish = 0;
                difference = 0;

                for (int channelID = 0; channelID < numberOfChannels; ++channelID) {
                    expected = expected_image_vec[channelID].at<float>(i,j);
                    diffuse = diffuse_map[channelID].at<float>(i,j);
                    specular = specular_map[channelID].at<float>(i,j);
                    maxExpectation = (diffuse * get_cosNL(i,j)) + (specular *  pow(get_cosRV(i,j,1), get_alpha()));

                    if (maxExpectation < expected) {
                        difference = expected - maxExpectation;
                        if (difference > grayish) {
                            grayish = difference;
                        }
                    }
                }

                for (int channelID = 0; channelID < numberOfChannels; ++channelID) {
                    expected_image_vec[channelID].at<float>(i,j) =  std::max(expected_image_vec[channelID].at<float>(i,j)-grayish , 0.0f);
                }
                grayMat.at<float>(i,j) += grayish;
            }
        }
    }


    //EXPECTED IMAGE
    Mat expectedGrayish;
    merge(expected_image_vec, expectedGrayish);
    namedWindow("expectedGrayish", WINDOW_NORMAL);
    imshow("expectedGrayish", expectedGrayish);
    waitKey(0);

    //PROJECTING GRAYISH
    vector<Mat> grayish3Mat;
    grayish3Mat.resize(3);
    grayish3Mat[0] = grayMat;
    grayish3Mat[1] = grayMat;
    grayish3Mat[2] = grayMat;
    Mat result;
    project(diffuse_map, specular_map, grayish3Mat, cosNL_map, cosRV_map[1], get_alpha(), result);

    //GRAYISH
    namedWindow("grayish", WINDOW_NORMAL);
    imshow("grayish", grayMat);
    waitKey(0);

    for (int channelID = 0; channelID < numberOfChannels; ++channelID) {
        Mat powered;
        pow(cosRV_map[1], get_alpha(), powered);
        compensated_Image_vec[channelID] = expected_image_vec[channelID] / ((diffuse_map[channelID].mul(cosNL_map)) + (specular_map[channelID].mul(powered)));
    }

    vector<Mat> Observeation;
    Observeation.resize(numberOfChannels);
    Mat img(get_resolution(), CV_32FC1, Scalar(0.0));
    Mat powered;
    pow(cosRV_map[1], get_alpha(), powered);
    for (int channelID = 0; channelID < numberOfChannels; ++channelID) {
        img = compensated_Image_vec[channelID].mul((diffuse_map[channelID]).mul(cosNL_map) + (compensated_Image_vec[channelID].mul(specular_map[channelID].mul(powered))));
        Observeation[channelID] = img.clone();
    }

    if(check){

        Mat compensated_Image;
        if (numberOfChannels > 1) {
            merge(compensated_Image_vec, compensated_Image);
        } else {
            compensated_Image = compensated_Image_vec[0].clone();
        }
        namedWindow("compensated!", WINDOW_NORMAL);
        imshow("compensated!", compensated_Image);
        imwrite("Output/compensated.jpg", compensated_Image*255.0);
        waitKey(0);

        Mat result;
        if (numberOfChannels > 1) {
            merge(Observeation, result);
        } else {
            result = img.clone();
        }
        namedWindow("result!", WINDOW_NORMAL);
        imshow("result!", result);
        imwrite("Output/result.jpg", result*255.0);
        waitKey(0);

    }



    return;
}

void lightProccessing::project( std::vector<Mat> kd,  std::vector<Mat> ks,  std::vector<Mat> light, Mat cosTeta, Mat cosPhi, float alpha, Mat &result){
    Mat projectedImage;
    int channels = kd.size();
    if(channels > 1){
        std::vector<Mat> projected;
        projected.resize(channels);
        Mat powered;
        pow(cosPhi, alpha, powered);
        for (int it = 0; it < channels; ++it) {
            projected[it] = light[it].mul((kd[it]).mul(cosTeta)) + light[it].mul(ks[it].mul(powered));
        }
        merge(projected, projectedImage);
    }
    result = projectedImage.clone();
    namedWindow("project!", WINDOW_NORMAL);\
    imshow("project!", projectedImage);
    waitKey(0);
}

void lightProccessing::render_image(std::vector<Mat> Diffuse, std::vector<Mat> Specular, std::vector<Mat> Light, Mat CosNL, Mat CosRV, Mat alpha, Mat &rendered, bool check){
    std::vector<Mat> rendered_vec;
    rendered_vec.resize(numberOfChannels);
    for (int channelId = 0; channelId < Diffuse.size(); ++channelId) {
        rendered_vec[channelId] = Diffuse[channelId].mul(CosNL.mul(Light[channelId]));
        Mat powered_cos;
        power_MatbyMat(CosRV, alpha, powered_cos);
        rendered_vec[channelId] += Specular[channelId].mul(Light[channelId].mul(powered_cos));
    }
    merge(rendered_vec, rendered);
    imwrite("rendered_image.png", rendered);

    if(check){
        namedWindow("Rendered Image", WINDOW_NORMAL);
        imshow("Rendered Image", rendered);
        waitKey(0);
    }
    return;
}

void lightProccessing::load_scene(bool check){
    cv::Mat temp1, temp2;
    File_Manager fm("data/data/");

    fm.import_Mat32F("normals.data", temp1);
    set_normals(-temp1, true);

    fm.import_Mat32F("xyz_map.data", temp2);
    set_xyzMap(temp2, true);

    compute_CosNL(check);

    compute_Mask_Image(check);

    compute_CosRVs(check);

    std::cout << "Scene has been loaded! \n";
}

int lightProccessing::load_cameras(bool check){

    std::vector<std::string> file_list;
    DIR *dp;
    struct dirent *dirp;
    std::string pathCamera = "data/cameras/";
    std::string pathSample = "data/samples/";
    std::string name;
    if((dp  = opendir(pathCamera.c_str())) == NULL) {
        std::cout << "Error(" << errno << ") opening " << pathCamera << std::endl;
        return errno;
    }

    while ((dirp = readdir(dp)) != NULL) {
        if (std::string(dirp->d_name) != "." && std::string(dirp->d_name) != "..") {
            file_list.push_back(std::string(dirp->d_name));
        }
    }
    closedir(dp);
    std::sort( file_list.begin(), file_list.end() );

    numberOfCameras = file_list.size();
    cameras.resize(numberOfCameras);

    //Save Images
    for (int it = 0; it < numberOfCameras; ++it) {

        name = file_list[it];
        name = name.substr(0, name.size()-4);

        Camera temp(it, name);
        temp.loadConfig(pathCamera+file_list[it]);
        std::cout << temp.name << std::endl << temp.focal_lenght << std::endl;
        Mat tempMat = imread(pathSample+name);
        tempMat.convertTo(tempMat, CV_32F);
        tempMat /= 255.0;
        temp.set_image(tempMat);
        temp.compute_world_coordinates();
        cameras[it] = temp;
    }

    if (check) {
        for (int it = 0; it < numberOfCameras; ++it) {
            std::cout << cameras[it].name << " :: " << cameras[it].intrinsics << std::endl;
            namedWindow(cameras[it].name, WINDOW_NORMAL);
            imshow(cameras[it].name, cameras[it].get_image());
            waitKey(0);
        }
    }

    //Initialize
    Observed_map.resize(numberOfCameras * numberOfChannels);
    for (int it = 0; it < Observed_map.size(); ++it) {
        Observed_map[it] = Mat(get_resolution(), CV_32FC1, Scalar(0.0)).clone();
    }

    cosRV_map.resize(numberOfCameras);

    std::cout << "Cameras has been loaded! \n";
    return 0;
}

void lightProccessing::extract_samples(bool check){

    int size = xyz_map.cols * xyz_map.rows;

    Vec3f point;
    Point3f pt3F;
    std::vector<Point3f> points3D;
    Point2f point2D;

    // convert the points to point3f format and put them into a vector
    for (int i = 0; i < xyz_map.rows; ++i) {
        for (int j = 0; j < xyz_map.cols; ++j) {
            if (is_Node(i,j)) {
                point = xyz_map.at<Vec3f>(i,j);
                pt3F = {point[0], point[1], point[2]};
            }else{
                pt3F = {0.0,0.0,0.0};
            }
            points3D.push_back(pt3F);
        }
    }

    std::vector<std::vector<Point2f>> imagePoints;
    std::vector<Point2f> imgpnts;

    // project the points to the images
    cv::Mat test_distortion(1,5, CV_32FC1, Scalar(0.0));
    for (int it = 0; it < numberOfCameras; ++it) {
        Camera &camIterator = cameras[it];
        Mat rvec;
        Rodrigues(camIterator.rotation, rvec);
        projectPoints(points3D, rvec, camIterator.translation, camIterator.intrinsics, test_distortion, imgpnts);
        imagePoints.push_back(imgpnts);
    }

    int i=0, j=0;

    for (int camID = 0; camID < numberOfCameras; ++camID) { // goes for all cameras
        Camera &camIterator = cameras[camID];

        std::cout << "Camera " << camID << " started...\n";
        Mat image = camIterator.get_image();
        Mat R,G,B;
        B = Observed_map[camID * numberOfChannels + 0].clone();
        G = Observed_map[camID * numberOfChannels + 1].clone();
        R = Observed_map[camID * numberOfChannels + 2].clone();

        Mat undist = image.clone();

        for (int i = 1; i < image.rows; ++i) {
            for (int j = 1; j < image.cols; ++j) {
                cv::Point2f unD = camIterator.undistort_SFM(Point2f(j,i));
                if(!(unD.y < 0 || unD.x<0 || unD.x>image.cols || unD.y>image.rows))
                    undist.at<Vec3f>(unD.y,unD.x) = image.at<Vec3f>(i,j);
            }
        }

        for (int pointID = 0; pointID < imagePoints[0].size(); ++pointID) { // goes for all points

            i = pointID / get_width();
            j = pointID % get_width();

            if (is_Node(i,j)) {


                Vec3f c = getMidColor(undist, imagePoints[camID][pointID], false);

                B.at<float>(i,j) = c[0];
                G.at<float>(i,j) = c[1];
                R.at<float>(i,j) = c[2];

            }

        }

        Observed_map[camID * numberOfChannels + 0] = B.clone();
        Observed_map[camID * numberOfChannels + 1] = G.clone();
        Observed_map[camID * numberOfChannels + 2] = R.clone();

        Mat merged;
        std::vector<Mat> arrayofimages;
        arrayofimages.push_back(B.clone());
        arrayofimages.push_back(G.clone());
        arrayofimages.push_back(R.clone());
        merge(arrayofimages,merged);

        imwrite("Output/Maps/Reprojection[" + std::to_string(camID) +"].jpg", merged*255);
    }
    std::cout << "Samples has been extracted! \n";
    return;
}

Vec3f lightProccessing::getMidColor(Mat &image, Point2f &point, bool check){

    Vec3f color = {0.0,0.0,0.0};
    int window_size = 5;
    float count = 0;
    for (int i = -window_size; i < window_size; ++i) {
        for (int j = -window_size; j < window_size; ++j) {
            if(point.y+j > 0 && point.x+i > 0 && point.y+j < image.rows && point.x+i < image.cols ){
                color += image.at<Vec3f>(point.y+j, point.x+i);
                count++;
            }
        }
    }

    if (count > 0) {
        color /= count;

        //        if (check) {
        //            circle(image, point, 10, cv::Scalar(255,0,0) );
        //            namedWindow("image", WINDOW_NORMAL);
        //            imshow("image", image);

        //            Mat test(100,100, CV_32FC3, Scalar(color));
        //            imshow("extracte color", test);
        //            waitKey(0);
        //        }
    }
    return color;
}
