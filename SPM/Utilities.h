////////////////////////////////////////////////////////////////////////////////////
// Copyright © Charalambos "Charis" Poullis, charalambos@poullis.org    	  //
// This work can only be used under an exclusive license of the author.           //
////////////////////////////////////////////////////////////////////////////////////

#ifndef __UTILITIES_H__
#define __UTILITIES_H__

#include <iostream>
#include <stdarg.h>
#include <malloc.h>
#include <string.h>
#include <string>
#include <vector>
#include <iterator>
#include <algorithm>
#include <time.h>
#include <sys/types.h>
#include <sys/timeb.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include "structures.h"

using namespace Eigen;


///To convert from radians to degrees and back
#define RADIANS_TO_DEGREES	57.295779513082320876798
#define DEGREES_TO_RADIANS	0.0174532925199432957692

#define INVALID_VALUE	666

#define EPSILON   1e-06

#define _MAX_PATH	256

typedef enum {
    NONE=0,
    FAILURE,
    SUCCESS,
    WARNING,
    INFO,
    SCRIPT
} STATUS;

///Helper function for _format(...)
static std::string vformat(const char *fmt, va_list argPtr) {
    /// We draw the line at a 1MB string.
    const int maxSize = 1000000;

    /// If the string is less than 161 characters,
    /// allocate it on the stack because this saves
    /// the malloc/free time.
    const int bufSize = 512;
    char stackBuffer[bufSize];

    int attemptedSize = bufSize - 1;

    int numChars = vsnprintf(stackBuffer, attemptedSize, fmt, argPtr);

    if (numChars >= 0) {
        /// Got it on the first try.
        //printf("%s\n",stackBuffer);
        return std::string(stackBuffer);
    }

    /// Now use the heap.
    char* heapBuffer = NULL;

    while ((numChars == -1 || numChars >= attemptedSize) && (attemptedSize < maxSize)) {
        /// Try a bigger size
        attemptedSize *= 2;
        heapBuffer = (char*)realloc(heapBuffer, attemptedSize + 1);
        numChars = vsnprintf(heapBuffer, attemptedSize, fmt, argPtr);
    }

    //printf("%s\n",heapBuffer);
    std::string result = std::string(heapBuffer);

    free(heapBuffer);

    return result;
}

///Prints out a string given a set of parameters. Like printf but for strings.
static std::string _format(const char* fmt ...) {
    va_list argList;
    va_start(argList,fmt);
    std::string result = vformat(fmt, argList);
    va_end(argList);
    return result;
}

///Function for timestamp. Returns the time in the format YMDHMS
static std::string timestamp()	{
    ///TIMESTAMP prints the current YMDHMS date as a time stamp.
#define TIME_SIZE	40

    static char time_buffer[TIME_SIZE];
    time_t now = time ( NULL );
    const struct tm *tm = localtime ( &now );

    size_t len = strftime ( time_buffer, TIME_SIZE, "%d %B %Y %I:%M:%S %p", tm );

#undef TIME_SIZE

    return _format("%s",time_buffer);
}

///Clamps a value between the range [a,b]
inline float clamp(float x, float a, float b)
{
    return x < a ? a : (x > b ? b : x);
}

///Copies the string and returns a pointer to the copied string
inline void copy_string(char *input, char *&output)	{
    output = new char[strlen(input)+1];
    for (unsigned int i=0;i<strlen(input)+1;i++)	{
        output[i] = input[i];
    }
    return;
}

inline void copy_strings(char **input, int number_of_strings, char **&output)	{
    output = new char*[number_of_strings];
    for (unsigned int i=0;i<number_of_strings;i++)	{
        copy_string(input[i],output[i]);
    }
    return;
}

/// Function to return the current working directory
/// this is generally the application path
static void getCurrentPath(char* buffer)	{
    getcwd(buffer, _MAX_PATH);
    return;
}

static std::string fullPath(const char *relative_path)	{
    // _MAX_PATH is the maximum length allowed for a path
    char working_dir[_MAX_PATH];
    // use the function to get the path
    getCurrentPath(working_dir);

    //std::cout << "Working directory: " << working_dir << std::endl;

    std::string full_path = std::string(working_dir) + "/" + std::string(relative_path);

    return full_path;
}

inline static unsigned short int float2short(float f)	{
    unsigned int bits = * (unsigned int *) &f;
    return (bits >> 15);
}

inline static unsigned short int signedfloat2short(float f)	{
    unsigned int bits = * (unsigned int *) &f;
    return (bits >> 16);
}

inline static float short2float(unsigned short int s)	{
    unsigned int bits = s << 15;
    return * (float *) &bits;
}

inline static float short2signedfloat(unsigned short int s)	{
    unsigned int bits = s << 16;
    return * (float *) &bits;
}

template<typename T>
inline static int _round(T num)	{
    if (ceil(num)-num > T(0.5))	{
        return int(floor(num));
    }
    else	{
        return int(ceil(num));
    }
}

///Comparison function for two ints using the std::sort
template<class T>
static bool compare_func(const T &v1, const T &v2)	{
    return (v1 < v2);
}

template<class T>
static void remove(std::vector<T *> &vec, int args, T *v1, ...)	{
    va_list a_list;
    //Initialize the va_list i.e T variable 'list' by the address
    //of first unknown variable argument by a call of va_start() macro
    va_start(a_list, v1);
    //In loop, retrieve each argument
    //second argument to va_arg is datatype
    //of expected argument
    typename std::vector<T *>::iterator it = vec.begin();
    for (int i=0;i<vec.size();i++,it++)	{
        if (vec[i] == (T *) v1)	{
            delete vec[i];
            vec.erase(it);
            break;
        }
    }
    for (int a=1;a<args;a++)	{
        T *v = va_arg(a_list, T *);
        typename std::vector<T *>::iterator it = vec.begin();
        for (int i=0;i<vec.size();i++,it++)	{
            if (vec[i] == v)	{
                delete vec[i];
                vec.erase(it);
                break;
            }
        }
    }
    va_end(a_list);
    return;
}

template<class T>
static void remove(std::vector<T *> &vec, std::vector<T *> const &to_be_removed)	{

    for (int i=0;i<to_be_removed.size();i++)	{
        typename std::vector<T *>::iterator it = vec.begin();
        bool not_found = false;
        for (int j=0;it!=vec.end();j++,it++)	{
            if (to_be_removed[i] == vec[j])	{
                delete vec[j];
                vec.erase(it);
                not_found = true;
                break;
            }
        }
        if (!not_found)	{
            printf("here 4\n");
            exit(0);
        }
    }
    return;
}

template<class T>
static void remove(std::vector<T> &vec, std::vector<T> const &to_be_removed)	{

    for (int i=0;i<to_be_removed.size();i++)	{
        typename std::vector<T>::iterator it = vec.begin();
        bool not_found = false;
        for (int j=0;it!=vec.end();j++,it++)	{
            if (to_be_removed[i] == vec[j])	{
                vec.erase(it);
                not_found = true;
                break;
            }
        }
        if (!not_found)	{
            printf("here 5\n");
            exit(0);
        }
    }
    return;
}

///to_be_removed MUST be in ascending order because the function reads it in reverse
template<class T>
static void remove(std::vector<T *> &vec, std::vector<int> const &to_be_removed)	{

    for (int i=to_be_removed.size()-1;i>=0;i--)	{
        typename std::vector<T *>::iterator it = vec.begin() + to_be_removed[i];
        delete vec[to_be_removed[i]];
        vec.erase(it);
    }
    return;
}

///to_be_removed MUST be in ascending order because the function reads it in reverse
template<class T>
static void remove(std::vector<T> &vec, std::vector<int> const &to_be_removed)	{

    for (int i=to_be_removed.size()-1;i>=0;i--)	{
        typename std::vector<T>::iterator it = vec.begin() + to_be_removed[i];
        vec.erase(it);
    }
    return;
}

static cv::Mat Vector3f_To_Mat(Vector3f Eigen_Input){
    cv::Mat cv_vector(3,1,CV_32FC1);
    cv_vector.at<float>(0,0) = Eigen_Input[0];
    cv_vector.at<float>(1,0) = Eigen_Input[1];
    cv_vector.at<float>(2,0) = Eigen_Input[2];
    return cv_vector.clone();
}

static Vector3f Mat_To_Vector3f(cv::Mat Mat_Input){
    Vector3f Eigen_vector;
    Eigen_vector[0] = Mat_Input.at<float>(0,0);
    Eigen_vector[1] = Mat_Input.at<float>(1,0);
    Eigen_vector[2] = Mat_Input.at<float>(2,0);
    return Eigen_vector;
}

static std::string match_to_store_string(Match match){
    std::string str = "";
//    str += match.should_count + "\n";
//    str += "#left_mean_matches_point\n";
//    str += std::to_string(match.left_mean_matches_point.x) + "\t"  + std::to_string(match.left_mean_matches_point.y) + "\n";
//    str += "#right_mean_matches_point\n";
//    str += std::to_string(match.right_mean_matches_point.x) + "\t"  + std::to_string(match.right_mean_matches_point.y) + "\n";
//    str += "#left_points...\n";
//    str += std::to_string(match.left_points.size()) + "\n";
//    for (int i = 0; i < match.left_points.size(); ++i) {
//        str += std::to_string(match.left_points[i].x) + "\t" + std::to_string(match.left_points[i].y) + "\n";
//    }
//    str += "#right_points...\n";
//    str += std::to_string(match.right_points.size()) + "\n";
//    for (int i = 0; i < match.right_points.size(); ++i) {
//        str += std::to_string(match.right_points[i].x) + "\t"  + std::to_string(match.right_points[i].y) + "\n";
//    }
    return str;
}

static cv::Mat normalize_Mat31FC1(cv::Mat input){

    float sum = pow(input.at<float>(0,0), 2);
    sum += pow(input.at<float>(1,0), 2);
    sum += pow(input.at<float>(2,0), 2);
    cv::Mat output(3,1,CV_32FC1);
    output.at<float>(0,0) = input.at<float>(0,0)/sum;
    output.at<float>(1,0) = input.at<float>(1,0)/sum;
    output.at<float>(2,0) = input.at<float>(2,0)/sum;
    return output.clone();
}

static void display_vector_as_Image(std::vector<Color> input, int rows, int cols, std::string window_name){
    cv::Mat image(cv::Mat(rows, cols, CV_32FC3));
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            int id = i * cols + j;
            cv::Vec3f temp;
            temp[0] = input[id].b();
            temp[1] = input[id].g();
            temp[2] = input[id].r();
            std::cout << temp << std::endl;
            image.at<cv::Vec3f>(i,j) = temp;
        }
    }
    cv::imwrite(window_name+".jpg", image);
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::imshow(window_name, image);
    cv::waitKey(0);
    return;
}

static void power_MatbyMat(cv::Mat src, cv::Mat power, cv::Mat &dst){
    cv::Mat res(src.size(), CV_32FC1);
    for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
            res.at<float>(i,j) = pow(src.at<float>(i,j) , power.at<float>(i,j));
        }
    }
    dst = res.clone();
    return;
}

// Checks if a matrix is a valid rotation matrix.
static bool isRotationMatrix(cv::Mat &R)
{
    cv::Mat Rt;
    transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());

    return  cv::norm(I, shouldBeIdentity) < 1e-6;

}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
static cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R)
{

    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y, z);
}




/*
static void print(std::string const &text, STATUS flag, int error_code=-1)	{
    if (error_code != -1)	{
        std::cout << "ERROR " << error_code << ": " << text << std::endl;
    }
    else	{
        std::cout << "INFO: " << text << std::endl;
    }
    return;
}
*/

#endif
