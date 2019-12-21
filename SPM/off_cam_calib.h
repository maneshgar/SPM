#ifndef OFF_CAM_CALIB_H
#define OFF_CAM_CALIB_H

#include <string>
#include <thread>

#include "file_manager.h"
#include "phaseshifting_pattern_encoder.h"
#include "phaseshifting_pattern_decoder.h"
#include "camera.h"


#define TILE_SIZE_V 31
#define TILE_SIZE_H 31

class OFF_Cam_Calib
{
public:

    Camera &myCamera;

//    OFF_Cam_Calib();
    OFF_Cam_Calib(PhaseShifting_Pattern_encoder &input_phaseShifting_pattern, Camera &input_camera);
    ~OFF_Cam_Calib();

    std::vector<std::vector<cv::Point2f>> all_image_corners;
    std::vector<std::vector<cv::Point3f>> all_Object_points;

    bool calibrate();
    void static printThreadBase(std::string str, std::string ID);
    int image_set_size;
    bool static proccess_ImageSet(PhaseShifting_pattern_decoder &camera_decoded_poses, std::string file, std::string img_set_name, std::vector<Feature_Point> vertical_featurePoint_indicator, std::vector<Feature_Point> horizontal_featurePoint_indicator, int v_tile_size, int h_tile_size);

private:

    std::vector<PhaseShifting_pattern_decoder> camera_decoded_poses;

    std::string root_dir = std::string("Image Set/");
    PhaseShifting_Pattern_encoder phs_pattern_encoder;
    Pattern syncronize_image;
    Pattern Mask_image;

    bool apply_calibration();
    bool print(int mode, std::string output_file_name);
};

#endif // OFF_CAM_CALIB_H
