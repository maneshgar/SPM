#ifndef PHASESHIFTING_PATTERN_ENCODER_H
#define PHASESHIFTING_PATTERN_ENCODER_H

#include "pattern.h"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <string>
#include "pattern_sequence.h"
#include "feature_point.h"
#include "structures.h"

class PhaseShifting_Pattern_encoder
{
public:

    cv::Vec2i Unwrapping_k;
    Phase_Pattern_Seq vertical_PatSeq;
    Phase_Pattern_Seq horizontal_PatSeq;
    Pattern syncronize_image;
    Pattern black;
    Pattern white;
    Pattern CheckerBoard;

    PhaseShifting_Pattern_encoder();
    PhaseShifting_Pattern_encoder(uint input_sequence_length, cv::Size input_pattern_size, int input_pattern_type);

    bool generate_patterns();
    bool encode_featurePoints(int vertical_distance, int horizontal_distance);

    bool display_patterns(std::string window_name);
    bool display_encoded_features();

    bool is_patterns_generated();



private:

    int length;
    int pattern_type;
    float scale = 1;
    cv::Size pattern_size;
    bool pattersn_generated;

    bool generate_sequence(Phase_Pattern_Seq &pattern_seq);
    bool generate_phaseMap(Phase_Pattern_Seq &pattern_seq);
    bool generate_syncronize_image();

};

#endif // PHASESHIFTED_PATTERN_H
