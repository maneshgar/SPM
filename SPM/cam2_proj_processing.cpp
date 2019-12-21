//#include "cam2_proj_processing.h"

//Cam2_Proj_Processing::Cam2_Proj_Processing(Camera &cam_left, Camera &cam_right, Projector &proj, GeometricObject &Geometric_Obj, std::vector<Match> &matches, cv::Mat xyz, cv::Mat normal_map)
//{
//    this->normal_Map = normal_map.clone();
//    this->xyz_Map = xyz;
//    this->camera_left = cam_left;
//    this->camera_right = cam_right;
//    this->projector = proj;
//    this->Geo_Object = Geometric_Obj;
//    this->Matches_Points = matches;

//}

//void Cam2_Proj_Processing::compute_diffuse_map_grayScale(Color input_color, cv::Mat &cos, cv::Mat &left_output, cv::Mat &right_output){

//    ///Calculates each camera seperatly
//    cv::Mat left(cv::Mat(projector.resolution.height, projector.resolution.width, CV_32FC3, cv::Scalar(0.0f)));
//    cv::Mat right(cv::Mat(projector.resolution.height, projector.resolution.width, CV_32FC3, cv::Scalar(0.0f)));

//    Ray L_vector;
//    L_vector.origin = cv::Mat(3,1,CV_32FC1);
//    L_vector.dir = cv::Mat(3,1,CV_32FC1);

//    // set origin:: location of the projector
//    L_vector.origin.at<float>(0,0) = -600;
//    L_vector.origin.at<float>(1,0) = -0;
//    L_vector.origin.at<float>(2,0) = -0;
//    int counter = 0;
//    //

//    cv::Mat temp_cos(cv::Mat(projector.resolution, CV_32FC1, cv::Scalar(0)));

//    for (int i = 0; i < this->xyz_Map.rows; ++i) {
//        for (int j = 0; j < this->xyz_Map.cols; ++j) {

//            int id = i * this->normal_Map.cols + j;
//            cv::Vec3f normal = this->normal_Map.at<cv::Vec3f>(i,j);
//            if(normal != cv::Vec3f(0.0f, 0.0f, 0.0f)){
//                //by the xyzmap find destination => direction,
//                cv::Vec3f xyz_temp = this->xyz_Map.at<cv::Vec3f>(i,j);
//                L_vector.dir.at<float>(0,0) = xyz_temp[0] - L_vector.origin.at<float>(0,0);
//                L_vector.dir.at<float>(1,0) = xyz_temp[1] - L_vector.origin.at<float>(1,0);
//                L_vector.dir.at<float>(2,0) = xyz_temp[2] - L_vector.origin.at<float>(2,0);

//                cv::normalize(L_vector.dir, L_vector.dir);
//                //by geo object get the normal.

//                cv::Mat normal_Mat(cv::Mat(3,1,CV_32FC1));
//                normal_Mat.at<float>(0,0) = normal[0];
//                normal_Mat.at<float>(1,0) = normal[1];
//                normal_Mat.at<float>(2,0) = normal[2];

//                //compute n.l
//                float nDOTl = normal_Mat.dot(L_vector.dir);
//                cv::Vec3f kd;
//                if(nDOTl>0){
//                    temp_cos.at<float>(i,j) = nDOTl;
//                    counter++;
//                    ///read reflected color

//                    Color Intensity = Matches_Points[id].left_mean_color;

//                    //calculate Kd
//                    kd[0] = Intensity.b() / (nDOTl * input_color.r());
//                    kd[1] = Intensity.g() / (nDOTl * input_color.g());
//                    kd[2] = Intensity.r() / (nDOTl * input_color.b());
//                    left.at<cv::Vec3f>(i,j) = kd;

//                    Intensity = Matches_Points[id].right_mean_color;

//                    //calculate Kd
//                    kd[0] = Intensity.b() / (nDOTl * input_color.r());
//                    kd[1] = Intensity.g() / (nDOTl * input_color.g());
//                    kd[2] = Intensity.r() / (nDOTl * input_color.b());
//                    right.at<cv::Vec3f>(i,j) = kd;

//                } else if(nDOTl>1){
//                    std::cout << "Error:: Cos T > 1!\n";
//                }
//                else{
//                    kd[0] = 0;
//                    kd[1] = 0;
//                    kd[2] = 0;
//                    left.at<cv::Vec3f>(i,j) = kd;
//                    right.at<cv::Vec3f>(i,j) = kd;
//                }
//            }
//        }
//    }
//    cv::imwrite("cosine_map.jpg", temp_cos*255);
//    cv::imwrite("diffuse_map.jpg", left*255);

//    cos = temp_cos.clone();
//    left_output = left.clone();
//    right_output = right.clone();
//    return;
//}

//void Cam2_Proj_Processing::compute_specular_map_grayScale(Color input_color, cv::Mat Diffuse_Part, cv::Mat &Specular_Map_output, cv::Mat &rdotv, int mode ){
//    ///Calculates each camera seperatly
//    cv::Mat temp_out(cv::Mat(projector.resolution.height, projector.resolution.width, CV_32FC1, cv::Scalar(0.0f)));
//    cv::Mat rdv(cv::Mat(projector.resolution.height, projector.resolution.width, CV_32FC1, cv::Scalar(0.0f)));
//    float Phi = 0.5;
//    Ray L_vector, V_vector;

//    L_vector.origin = cv::Mat(3,1,CV_32FC1);
//    L_vector.dir = cv::Mat(3,1,CV_32FC1);

//    V_vector.dir = cv::Mat(3,1,CV_32FC1);

//    float Intensity;
//    ///read reflected color

//    // set origin:: location of the projector
//    L_vector.origin.at<float>(0,0) = -600;
//    L_vector.origin.at<float>(1,0) = -0;
//    L_vector.origin.at<float>(2,0) = -0;
////    int counter = 0;
//    //

//    cv::Mat temp_cosRV(cv::Mat(projector.resolution, CV_32FC1, cv::Scalar(0)));

//    for (int i = 0; i < this->xyz_Map.rows; ++i) {
//        for (int j = 0; j < this->xyz_Map.cols; ++j) {

//            int id = i * this->normal_Map.cols + j;

//            switch (mode){
//            case 1:
//                Intensity = Matches_Points[id].left_mean_color.getGrayScale();
//                V_vector.origin = this->camera_left.get_world_coordinates();
//                break;

//            case 2:
//                Intensity = Matches_Points[id].right_mean_color.getGrayScale();
//                V_vector.origin = this->camera_right.get_world_coordinates();
//                break;
//            }

//            cv::Vec3f normal = this->normal_Map.at<cv::Vec3f>(i,j);

//            if(normal != cv::Vec3f(0.0f, 0.0f, 0.0f)){
//                //by the xyzmap find destination => direction,
//                cv::Vec3f xyz_temp = this->xyz_Map.at<cv::Vec3f>(i,j);
//                L_vector.dir.at<float>(0,0) = xyz_temp[0] - L_vector.origin.at<float>(0,0);
//                L_vector.dir.at<float>(1,0) = xyz_temp[1] - L_vector.origin.at<float>(1,0);
//                L_vector.dir.at<float>(2,0) = xyz_temp[2] - L_vector.origin.at<float>(2,0);

//                V_vector.dir.at<float>(0,0) = xyz_temp[0] - V_vector.origin.at<float>(0,0);
//                V_vector.dir.at<float>(1,0) = xyz_temp[1] - V_vector.origin.at<float>(1,0);
//                V_vector.dir.at<float>(2,0) = xyz_temp[2] - V_vector.origin.at<float>(2,0);

//                cv::normalize(L_vector.dir, L_vector.dir);
//                cv::normalize(V_vector.dir, V_vector.dir);
//                //by geo object get the normal.

//                cv::Mat normal_Mat(cv::Mat(3,1,CV_32FC1));
//                normal_Mat.at<float>(0,0) = normal[0];
//                normal_Mat.at<float>(1,0) = normal[1];
//                normal_Mat.at<float>(2,0) = normal[2];

//                //compute n.l
//                float nDOTl = normal_Mat.dot(L_vector.dir);

//                ///////////////////////check the sign!
//                cv::Mat reflection = L_vector.dir - (2 * nDOTl * normal_Mat);

//                float rDOTv = reflection.dot(V_vector.dir);
//                rdv.at<float>(i,j) = rDOTv;
//                float ks;

//                if(rDOTv > 0){
//                    temp_cosRV.at<float>(i,j) = rDOTv;
//                    //calculate Kd
//                    ks =  std::abs((Intensity - Diffuse_Part.at<float>(i,j))) / (input_color.getGrayScale() * pow(rDOTv, Phi));
//                    temp_out.at<float>(i,j) = ks;

//                } else if(rDOTv > 1){
//                    std::cout << "Error:: Cos Phi > 1!\n";
//                }
//                else{
//                    ks = 0;
//                    temp_out.at<float>(i,j) = ks;
//                }
//            }
//        }
//    }
//    cv::imwrite("cosine_Phi_map.jpg", temp_cosRV*255);
//    cv::imwrite("Specular_map.jpg", temp_out*255);

//    rdotv = rdv.clone();
//    Specular_Map_output = temp_out.clone();
//    return;
//}

//void Cam2_Proj_Processing::compute_diffuse_map_grayScale(Color input_color, cv::Mat &cos, cv::Mat &output, int mode){

//    ///Calculates each camera seperatly
//    /// modes :: 1- Mean 2-Max 3-Min
//    cv::Mat temp_out(cv::Mat(projector.resolution.height, projector.resolution.width, CV_32FC1, cv::Scalar(0.0f)));

//    Ray L_vector;
//    L_vector.origin = cv::Mat(3,1,CV_32FC1);
//    L_vector.dir = cv::Mat(3,1,CV_32FC1);

//    // set origin:: location of the projector
//    L_vector.origin.at<float>(0,0) = -600;
//    L_vector.origin.at<float>(1,0) = -0;
//    L_vector.origin.at<float>(2,0) = -0;
//    //

//    cv::Mat temp_cos(cv::Mat(projector.resolution, CV_32FC1, cv::Scalar(0)));

//    for (int i = 0; i < this->xyz_Map.rows; ++i) {
//        for (int j = 0; j < this->xyz_Map.cols; ++j) {

//            int id = i * this->normal_Map.cols + j;
//            cv::Vec3f normal = this->normal_Map.at<cv::Vec3f>(i,j);
//            if(normal != cv::Vec3f(0.0f, 0.0f, 0.0f)){
//                //by the xyzmap find destination => direction,
//                cv::Vec3f xyz_temp = this->xyz_Map.at<cv::Vec3f>(i,j);
//                L_vector.dir.at<float>(0,0) = xyz_temp[0] - L_vector.origin.at<float>(0,0);
//                L_vector.dir.at<float>(1,0) = xyz_temp[1] - L_vector.origin.at<float>(1,0);
//                L_vector.dir.at<float>(2,0) = xyz_temp[2] - L_vector.origin.at<float>(2,0);

//                cv::normalize(L_vector.dir, L_vector.dir);
//                //by geo object get the normal.

//                cv::Mat normal_Mat(cv::Mat(3,1,CV_32FC1));
//                normal_Mat.at<float>(0,0) = normal[0];
//                normal_Mat.at<float>(1,0) = normal[1];
//                normal_Mat.at<float>(2,0) = normal[2];

//                //compute n.l
//                float nDOTl = normal_Mat.dot(L_vector.dir);
//                float kd;
//                if(nDOTl>0){
//                    temp_cos.at<float>(i,j) = nDOTl;
//                    counter++;
//                    Color Intensity;
//                    ///read reflected color
//                    switch (mode){
//                    case 1:
//                        Intensity = Matches_Points[id].mean_color;
//                        break;

//                    case 2:
//                        if(Matches_Points[id].left_mean_color.getGrayScale() > Matches_Points[id].right_mean_color.getGrayScale())
//                            Intensity = Matches_Points[id].left_mean_color;
//                        else
//                            Intensity = Matches_Points[id].right_mean_color;
//                        break;

//                    case 3:
//                        if(Matches_Points[id].left_mean_color.getGrayScale() < Matches_Points[id].right_mean_color.getGrayScale())
//                            Intensity = Matches_Points[id].left_mean_color;
//                        else
//                            Intensity = Matches_Points[id].right_mean_color;
//                        break;
//                    }

//                    //calculate Kd
//                    kd = Intensity.getGrayScale() / (nDOTl * input_color.getGrayScale());

//                    temp_out.at<float>(i,j) = kd;

//                } else if(nDOTl>1){
//                    std::cout << "Error:: Cos T > 1!\n";
//                }
//                else{
//                    kd = 0;
//                    temp_out.at<float>(i,j) = kd;
//                }
//            }
//        }
//    }
//    cv::imwrite("cosine_map.jpg", temp_cos*255);
//    cv::imwrite("diffuse_map.jpg", temp_out*255);

//    cos = temp_cos.clone();
//    output = temp_out.clone();
//    return;
//}

//cv::Mat Cam2_Proj_Processing::render_image(cv::Mat Light, cv::Mat kd, cv::Mat cos){

//    cv::Mat I = Light.clone();
//    I = I.mul(kd);
//    for (int i = 0; i < kd.rows; ++i) {
//        for (int j = 0; j < kd.cols; ++j) {
//            I.at<cv::Vec3f>(i,j)[0] *= cos.at<float>(i,j);
//            I.at<cv::Vec3f>(i,j)[1] *= cos.at<float>(i,j);
//            I.at<cv::Vec3f>(i,j)[2] *= cos.at<float>(i,j);
//        }
//    }
//    return I.clone();

//}

//cv::Mat Cam2_Proj_Processing::render_image(Color I, cv::Mat kd, cv::Mat cos){

//    cv::Mat output = kd.mul(cos);
//    output *= I.getGrayScale();
//    return output.clone();
//}

//void fcn(int m, int n, double *x, double *fvec, int *iflag, double params[])
//{
//    float I = params[0];
//    float alpha = params[1];
//    float L, Kd, cosT, Ks, cosPhi;
//    for (int i = 0; i < m; ++i) {
//        L = x[i*5+0];
//        Kd = x[i*5+1];
//        cosT = x[i*5+2];
//        Ks = x[i*5+3];
//        cosPhi = x[i*5+4];

//        fvec[i] = I - ( L*Kd*cosT + L * Ks * (pow(cosPhi,alpha)));
//        std::cout << "fcn :: " << fvec[i] << std::endl;

//    }
//        /* calculate the functions at x and return
//           the values in fvec[0] through fvec[m-1] */
//}

//void Cam2_Proj_Processing::apply_LM(Color Intensity, cv::Mat Diffuse_Map, cv::Mat nDOTl, cv::Mat Specular_Map, cv::Mat rDOTv, float alpha, cv::Mat &LM_diffuse_output, cv::Mat &LM_specular_output){
////    cv::Mat diffuse (cv::Mat(Diffuse_Map.size(), CV_32FC1, cv::Scalar(0.0f)));
////    cv::Mat specular(cv::Mat(Diffuse_Map.size(), CV_32FC1, cv::Scalar(0.0f)));
//    int number_of_unknowns = 2 * Diffuse_Map.rows * Diffuse_Map.cols;
//    int number_of_equations = Diffuse_Map.rows * Diffuse_Map.cols;
//    int number_of_knowns = Diffuse_Map.rows * Diffuse_Map.cols * 5;
//    double unKnowns[number_of_unknowns];
//    int msk[1];
//    double fvec[number_of_equations];
//    double tol;
//    int *info = new int;
//    int *nfev = new int;
//    double params[number_of_knowns];

//    for (int i = 0; i < Diffuse_Map.rows * Diffuse_Map.cols; ++i) {
//        unKnowns[i*2]   = Diffuse_Map.at<float>(i/Diffuse_Map.cols,i%Diffuse_Map.cols) = unKnowns[i*2];
//        unKnowns[i*2 + 1] = Specular_Map.at<float>(i/Diffuse_Map.cols,i%Diffuse_Map.cols);
//    }

//    for (int i = 0; i < Diffuse_Map.rows * Diffuse_Map.cols; ++i) {
//        params[i*5] = Intensity.getGrayScale();
//        Match m;
//        params[i*5 + 1] = Matches_Points[i].left_mean_color.getGrayScale();
//        params[i*5 + 2] = nDOTl.at<float>(i/Diffuse_Map.cols,i%Diffuse_Map.cols);
//        params[i*5 + 3] = rDOTv.at<float>(i/Diffuse_Map.cols,i%Diffuse_Map.cols);
//        params[i*5 + 4] = alpha;
//    }

//    lmdif0(fcn,number_of_equations, number_of_unknowns, unKnowns, msk, fvec, tol, info, nfev, params);

////    for (int i = 0; i < number_of_unknowns; ++i) {
////        diffuse.at<float>(i/Diffuse_Map.cols,i%Diffuse_Map.cols) = unKnowns[i*2];
////        specular.at<float>(i/Diffuse_Map.cols,i%Diffuse_Map.cols) = unKnowns[i*2 + 1];
////    }
////    LM_diffuse_output = diffuse.clone();
////    LM_specular_output = specular.clone();
//    return;
//}
