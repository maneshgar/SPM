#ifndef PHASESHIFTING_PATTERN_DECODER_H
#define PHASESHIFTING_PATTERN_DECODER_H

#include <string>
#include <opencv2/opencv.hpp>

#include "structures.h"
#include "pattern.h"
#include "pattern_sequence.h"
#include "file_manager.h"

class PhaseShifting_pattern_decoder
{
public:

    PhaseShifting_pattern_decoder(int input_id, std::string input_name, std::vector<Feature_Point> vertical_featurePoint_indicator, std::vector<Feature_Point> horizontal_featurePoint_indicator, int input_vertical_tile_size, int input_horizontal_tile_size, cv::Mat &mask);
    PhaseShifting_pattern_decoder();
    ~PhaseShifting_pattern_decoder();

    uint ID;
    std::string name;

    Phase_Pattern_Seq vertical_pattern_seq;
    Phase_Pattern_Seq horizontal_pattern_seq;

    cv::Mat rotation;
    cv::Mat translation;

    std::vector<Feature_Point> feature_points;

    bool load_patterns(std::string directory);
    bool decode(cv::Point center);
    bool display_phase_map();
    bool set_feature_indicators();
    bool is_node();
    cv::Mat mask_image;

private:
    int tile_size_h;
    int tile_size_v;

    bool Initialize_phase_pattern_seqs();
    bool extract_phaseMap(Phase_Pattern_Seq &vertical_pattern_seq, cv::Point2f circle);
    bool decode_feature_points(std::vector<std::vector<Feature_Point>> &feature_points_with_duplicate);
    bool find_nearest_point(std::vector<std::vector<Feature_Point>> feature_points_with_duplicate);
};

#endif // PHASESHIFTING_PATTERN_DECODER_H
