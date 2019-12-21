#include "file_manager.h"

File_Manager::File_Manager()
{

}

File_Manager::File_Manager(std::__cxx11::string input_root=""){
    this->root = input_root;
}

int File_Manager::get_dir_list(std::vector<std::__cxx11::string> &output_flist, std::__cxx11::string sub_directory){
    DIR *dp;
    struct dirent *dirp;
    std::string path = this->root;
    path = path.append(sub_directory);
    if((dp  = opendir(path.c_str())) == NULL) {
        std::cout << "Error(" << errno << ") opening " << path << std::endl;
        return errno;
    }

    while ((dirp = readdir(dp)) != NULL) {
        if (std::string(dirp->d_name) != "." && std::string(dirp->d_name) != "..") {
            output_flist.push_back(std::string(dirp->d_name));
        }
    }
    closedir(dp);
    std::sort( output_flist.begin(), output_flist.end() );
    return 0;
}
bool File_Manager::write_to_file(std::__cxx11::string path, std::__cxx11::string str){
    std::ofstream myfile;
    std::cout << "writing into :: " << (this->root+path)  << std:: endl;
    myfile.open ((this->root+path));
    myfile << str;
    myfile.close();
    return false;
}

bool File_Manager::set_root(std::__cxx11::string input_root){
    this->root = input_root;
    return false;
}

std::string File_Manager::get_root(){
    return this->root.c_str();
}

bool File_Manager::load_pattern(Pattern &out_pattern, std::__cxx11::string sub_directory){
    cv::Mat read_image = cv::imread(this->root+sub_directory);
    cv::cvtColor(read_image,read_image,CV_BGR2GRAY);
    out_pattern.type = read_image.type();
    out_pattern.size = read_image.size();
    out_pattern.image = read_image.clone();
    return false;
}

bool File_Manager::load_pattern_seq(Pattern_Sequence &out_pattern_seq, std::__cxx11::string sub_directory){
    std::vector<std::string> files = std::vector<std::string>();
    this->get_dir_list(files, sub_directory);
    for (uint i = 0;i < files.size();i++) {
        Pattern temp_pattern(i, sub_directory);
        this->load_pattern(temp_pattern, sub_directory+files[i]);
        out_pattern_seq.push_Element(temp_pattern);
        if (!out_pattern_seq.initialized) {
            out_pattern_seq.initialize(temp_pattern.size, temp_pattern.type);
        }
    }
    return false;
}

std::string File_Manager::mat_to_string(cv::Mat input_matrix){
    cv::Mat temp_mat;
    input_matrix.convertTo(temp_mat, CV_32F);

    std::string temp_str = "[";
    for (int i = 0; i < temp_mat.rows; ++i) {
        for (int j = 0; j < temp_mat.cols; ++j) {
            temp_str.append(std::to_string(temp_mat.at<float>(i,j)));
            temp_str.append(" , " );
        }
        if(i != temp_mat.rows-1){
            temp_str.append("\n");
        }
    }
    temp_str.append("]\n");
    return temp_str;
}

std::string File_Manager::mat_to_string_file_format(cv::Mat input_matrix){
    cv::Mat temp_mat;
    input_matrix.convertTo(temp_mat, CV_32F);
    std::string temp_str = std::to_string(input_matrix.rows) + std::string("\n") + std::to_string(input_matrix.cols) + std::string("\n");
    for (int i = 0; i < temp_mat.rows; ++i) {
        for (int j = 0; j < temp_mat.cols; ++j) {
            temp_str.append(std::to_string(temp_mat.at<float>(i,j)));
            temp_str.append("\n");
        }
    }
    return temp_str;
}

bool File_Manager::read_matrix_file(std::__cxx11::string sub_directory, cv::Mat &output_mat){
    std::ifstream infile(this->root + sub_directory);
    float r,c;
    infile >> r >> c;
    cv::Mat temp(r,c,CV_32FC1);
    for (int i = 0; i < r; ++i) {
        for (int j = 0; j < c; ++j) {
            infile >> temp.at<float>(i,j);
        }
    }
    output_mat = temp.clone();
    return false;
}

std::string File_Manager::vec3f_to_string(cv::Vec3f input_vector){
    std::string temp_str = " [";
    temp_str.append(std::to_string(input_vector[0]));
    temp_str.append(", ");
    temp_str.append(std::to_string(input_vector[1]));
    temp_str.append(", ");
    temp_str.append(std::to_string(input_vector[2]));
    temp_str.append("] ");
    return temp_str;
}

void File_Manager::export_obj(std::vector<cv::Mat> point_cloud){
    std::cout << "Exporting to Obj file...\n";
    std::cout << "Number of points :: " << point_cloud.size() << std::endl;
    std::ofstream of(this->root + "point_cloud.obj", std::ofstream::out);
    for (uint i=0; i<point_cloud.size(); i++){
        //        std::cout << "v " << i << " :: " << point_cloud[i].at<float>(0,0) <<" "<<point_cloud[i].at<float>(1,0)<<" "<<point_cloud[i].at<float>(2,0)<<std::endl;
        of << "v " << point_cloud[i].at<float>(0,0) <<"\t"<<point_cloud[i].at<float>(1,0)<<"\t"<<point_cloud[i].at<float>(2,0)<<std::endl;
    }
    //    of.close();
}

void File_Manager::export_Mat32FC3(cv::Mat data, std::string name){
    std::cout << "Exporting " + name + "...\n";
    std::ofstream of(this->root + name + ".data", std::ofstream::out);
    of << data.rows << std::endl;
    of << data.cols << std::endl;
    for (uint i = 0; i < data.rows; i++){
        for (int j = 0; j < data.cols; ++j) {
            of << data.at<cv::Vec3f>(i,j)[0] << "\t" << data.at<cv::Vec3f>(i,j)[1] <<" \t "<< data.at<cv::Vec3f>(i,j)[2] <<std::endl;
        }
    }
}

std::vector<cv::Mat> File_Manager::import_obj(std::__cxx11::string path){
    std::string sinput;
    float finput;
    std::cout << "Importing Obj file : " << path << std::endl;
    std::vector<cv::Mat> PC_input;
    cv::Mat point(3, 1, CV_32FC1);
    std::string line;
    std::ifstream myfile (path);
    std::string vector_sign = "v";
    if (myfile.is_open()){
        while(std::getline(myfile, line)){
            std::stringstream ssin(line);
            std::string charac;
            ssin >> charac;
            if(charac.compare(vector_sign) == 0){
                ssin >> sinput;
                finput = std::stof(sinput);
                point.at<float>(0,0) = finput;

                ssin >> sinput;
                finput = std::stof(sinput);
                point.at<float>(1,0) = finput;

                ssin >> sinput;
                finput = std::stof(sinput);
                point.at<float>(2,0) = finput;

                PC_input.push_back(point.clone());
            }
        }
        myfile.close();
    }
    else std::cout << "Unable to open file";
    return PC_input;
}

void File_Manager::import_Mat32F(std::__cxx11::string path, cv::Mat &imported_mat){

    std::fstream myfile(this->root + path, std::ios_base::in);
    int rows, cols;
    myfile >> rows;
    myfile >> cols;

    cv::Mat input(cv::Mat(rows, cols, CV_32FC3));
    for (int i = 0; i < input.rows; ++i) {
        for (int j = 0; j < input.cols; ++j) {
            cv::Vec3f temp;
            myfile >> temp[0];
            myfile >> temp[1];
            myfile >> temp[2];
            input.at<cv::Vec3f>(i,j) = temp;
        }
    }
    imported_mat = input.clone();
    return;
}

/*
//bool is_comment_line(std::string line){
//    for (int i = 0; i < line.length(); ++i) {
//        if(line.at(i) == '#'){
//            return true;
//        } else if(!line.at(i) == ' '){
//            return false;
//        }
//    }
//    std::cout << "\nis_comment_line DOES NOT WORK CORRECTLY!!!\n";
//    return false;
//}
*/

void File_Manager::import_matches(std::string path, std::vector<Match> &match_out){

    std::vector<Match> matches_p;
    int x, y, index;
    std::cout << "Load matches : " << path << std::endl;

    std::string line;
    std::string word;
    std::ifstream myfile(path);

    if (myfile.is_open()){
        std::getline(myfile, line);
        std::stringstream ssin(line);
        ssin >> word;
        x = std::stoi(word);
        ssin >> word;
        y = std::stoi(word);
        while(std::getline(myfile, line)){
            Match temp_match;
            std::stringstream ssin(line);
            ssin >> word;
            int number_of_elements = std::stoi(word);
            if( number_of_elements > 0){
                temp_match.should_count = true;
                for (int it = 0; it < number_of_elements; ++it) {
                    cv::Point2f temp_point;
                    ssin >> word;
                    index = std::stoi(word);
                    temp_point.x = index / y;
                    temp_point.y = index % y;
                    temp_match.points.push_back(temp_point);
                }
            }else{
                temp_match.should_count = false;
            }
            matches_p.push_back(temp_match);
        }
    }else{
        std::cout << "Unable to open file" << std::endl;
    }
    myfile.close();
    match_out = matches_p;
    return;
}
