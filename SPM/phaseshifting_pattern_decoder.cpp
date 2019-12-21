#include "phaseshifting_pattern_decoder.h"

PhaseShifting_pattern_decoder::PhaseShifting_pattern_decoder(){

}

PhaseShifting_pattern_decoder::PhaseShifting_pattern_decoder(int input_id, std::string input_name, std::vector<Feature_Point> vertical_featurePoint_indicator, std::vector<Feature_Point> horizontal_featurePoint_indicator, int input_vertical_tile_size, int input_horizontal_tile_size, cv::Mat &mask)
{
    this->ID = input_id;
    this->name = input_name;
    this->vertical_pattern_seq.orientation = 'V';
    this->horizontal_pattern_seq.orientation = 'H';
    this->vertical_pattern_seq.featurePoint_indicator = vertical_featurePoint_indicator;
    this->horizontal_pattern_seq.featurePoint_indicator = horizontal_featurePoint_indicator;
    this->tile_size_v = input_vertical_tile_size;
    this->tile_size_h = input_horizontal_tile_size;
    this->mask_image = mask.clone();
}

PhaseShifting_pattern_decoder::~PhaseShifting_pattern_decoder(){

}

bool PhaseShifting_pattern_decoder::display_phase_map(){
    //    cv::Mat(cv::Mat(phase_shifted.phaseMap.size,phase_shifted.phaseMap.type));
    return false;
}

bool PhaseShifting_pattern_decoder::decode(cv::Point center){    //    extract_common_point(center);
    extract_phaseMap(vertical_pattern_seq,center);
    extract_phaseMap(horizontal_pattern_seq, center);
    std::vector<std::vector<Feature_Point>> feature_points_with_duplicate;
    decode_feature_points(feature_points_with_duplicate);
    find_nearest_point(feature_points_with_duplicate);
    return false;
}

bool PhaseShifting_pattern_decoder::extract_phaseMap(Phase_Pattern_Seq &pattern_seq, cv::Point2f center){
    std::cout << "Center :: " << center <<std::endl;
    double minK, MaxK;
    std::vector<Phase_Map_Point> pixels_phases;
    int rows = pattern_seq.pattern_size.height;
    int cols = pattern_seq.pattern_size.width;

    cv::Mat K(rows, cols, CV_32FC1, cv::Scalar(0));
    cv::Mat Phase(rows, cols, CV_32FC1, cv::Scalar(0));
    int k = 40; // scale 1 => 40; scale 3 => 13
    K.at<float>(center.y, center.x) = k;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0 ; j < cols; ++j) {
            if (mask_image.at<float>(i,j) == 1) {

                float numerator = 0;
                float denominator = 0;

                for (uint ptN = 0; ptN < pattern_seq.patterns.sequence.size(); ++ptN) {
                    numerator  += pattern_seq.patterns.sequence[ptN].image.at<uchar>(i, j) * sin(2*M_PI*ptN/pattern_seq.patterns.sequence.size());
                    denominator += pattern_seq.patterns.sequence[ptN].image.at<uchar>(i, j) * cos(2*M_PI*ptN/pattern_seq.patterns.sequence.size());
                }
                float phase = -(atan(numerator/denominator));
                Phase.at<float>(i,j) = phase;
            }
        }
    }

    float phase;
    float lastPhase;
    for (int j = center.x + 1 ; j < cols; ++j) {
        if (mask_image.at<float>(center.y, j) == 1) {
            phase = Phase.at<float>(center.y,j);
            lastPhase = Phase.at<float>(center.y,j-1);
            k = K.at<float>(center.y,j-1);
            if ( (lastPhase - phase) > 1 ) {
                k++;
            }else if ( (phase - lastPhase) > 1 ) {
                k--;
            }
            K.at<float>(center.y,j) = k;
        }
    }

    for (int j = (center.x)-1 ; j >= 0; --j) {
        if (mask_image.at<float>(center.y, j) == 1) {
            phase = Phase.at<float>(center.y,j);
            lastPhase = Phase.at<float>(center.y,j+1);
            k = K.at<float>(center.y,j+1);
            if ( (phase - lastPhase) > 1) {
                k--;
            }else if ( (lastPhase - phase) > 1 ) {
                k++;
            }
            K.at<float>(center.y,j) = k;
        }
    }

    for (int j = 0; j < cols; ++j) {
        for (int i = center.y+1 ; i < rows; ++i) {
            if (mask_image.at<float>(i, j) == 1) {
                phase = Phase.at<float>(i, j);
                lastPhase = Phase.at<float>(i-1, j);
                k = K.at<float>(i-1, j);
                if ( (phase - lastPhase) > 1) {
                    k--;
                }else if ( (lastPhase - phase) > 1) {
                    k++;
                }
                K.at<float>(i, j) = k;
            }
        }
    }

    for (int j = 0; j < cols; ++j) {
        for (int i = center.y-1 ; i >= 0; --i) {
            if (mask_image.at<float>(i, j) == 1) {
                phase = Phase.at<float>(i, j);
                lastPhase = Phase.at<float>(i+1, j);
                k = K.at<float>(i+1, j);
                if ( (phase - lastPhase) > 1) {
                    k--;
                }else if ( (lastPhase - phase) > 1) {
                    k++;
                }
                K.at<float>(i, j) = k;
            }
        }
    }

    for (int i = 0 ; i < rows; ++i) {
        for (int j = center.x + 1; j < cols; ++j) {
            if (mask_image.at<float>(i, j) == 1 && mask_image.at<float>(i, j-1) == 1) {
                phase = Phase.at<float>(i, j);
                lastPhase = Phase.at<float>(i, j-1);
                k = K.at<float>(i, j-1);
                if ( (phase - lastPhase) > 1) {
                    k--;
                }else if ( (lastPhase - phase) > 1) {
                    k++;
                }
                K.at<float>(i, j) = k;
            }
        }
    }

    for (int i = 0 ; i < rows; ++i) {
        for (int j = center.x - 1; j >= 0; --j) {
            if (mask_image.at<float>(i, j) == 1 && mask_image.at<float>(i, j+1) == 1 ) {
                phase = Phase.at<float>(i, j);
                lastPhase = Phase.at<float>(i, j+1);
                k = K.at<float>(i, j+1);
                if ( (phase - lastPhase) > 1) {
                    k--;
                }else if ( (lastPhase - phase) > 1) {
                    k++;
                }
                K.at<float>(i, j) = k;
            }
        }
    }

    //    for (int i = 0; i < rows; ++i) {
    //        for (int j = 0; j < cols; ++j) {
    //            if(mask_image.at<float>(i, j) == 1){
    //                if ( K.at<float>(i,j) < 0) {
    //                    std::cout << i << "  " << j << "\n";
    //                }
    //            }
    //        }
    //    }
    cv::Point min, max;
    cv::minMaxLoc(K, &minK, &MaxK, &min, &max);
    std::printf("Min and max k == %f, %f,  ", minK, MaxK);
    std::cout << min <<  "    " << max << std::endl;
    pattern_seq.phaseMap.image = Phase + (K * M_PI);
    //    cv::imshow("extracted Phase map", pattern_seq.phaseMap.image/255.0);
    //    cv::imshow("K", K/50.0);
    //    cv::waitKey(0);
    return false;
}

bool PhaseShifting_pattern_decoder::decode_feature_points(std::vector<std::vector<Feature_Point>> &feature_points_with_duplicate){
    float verror = 0.1;
    float herror = 0.02;

    int cam_width = vertical_pattern_seq.patterns.patterns_size.width;
    int cam_height = vertical_pattern_seq.patterns.patterns_size.height;
    for (uint vf = 0; vf < vertical_pattern_seq.featurePoint_indicator.size(); ++vf) {
        cv::Mat tempV;
        tempV = cv::abs(vertical_pattern_seq.phaseMap.image - vertical_pattern_seq.featurePoint_indicator[vf].vPhase);

        for (uint hf = 0; hf < horizontal_pattern_seq.featurePoint_indicator.size(); ++hf) {

            std::vector<Feature_Point> localKeys;

            cv::Mat tempH;
            tempH = cv::abs(horizontal_pattern_seq.phaseMap.image - horizontal_pattern_seq.featurePoint_indicator[hf].hPhase);

            for (int i = 0; i < cam_height; ++i) {
                for (int j = 0; j < cam_width; ++j) {
                    if (mask_image.at<float>(i,j) == 1) {
                        if (tempV.at<float>(i,j) < verror) {
                            if (tempH.at<float>(i,j) < herror) {
                                Feature_Point key = Feature_Point(hf*tile_size_h, vf*tile_size_v, i, j, vertical_pattern_seq.phaseMap.image.at<float>(i,j), horizontal_pattern_seq.phaseMap.image.at<float>(i,j),
                                                                  (vertical_pattern_seq.phaseMap.image.at<float>(i,j) - vertical_pattern_seq.featurePoint_indicator[vf].vPhase),
                                                                  (horizontal_pattern_seq.phaseMap.image.at<float>(i,j)- horizontal_pattern_seq.featurePoint_indicator[hf].hPhase));
                                //                                std::printf("Local Key :: %f, %f, %f, %f \n", key.pX, key.pY, key.rX, key.rY);
                                localKeys.push_back(key);
                            }
                        }
                    }
                }
            }
            //checks if any feature point found
            if (localKeys.size() > 0) {
                feature_points_with_duplicate.push_back(localKeys);
            }
        }
    }
    return false;
}

bool PhaseShifting_pattern_decoder::find_nearest_point(std::vector<std::vector<Feature_Point>> all_local_features){
    std::vector<Feature_Point> corners;
    for (uint i = 0; i < all_local_features.size(); ++i) {
        float min = 1000;
        int min_ID = 0;
        for (uint k = 0; k < all_local_features[i].size(); ++k) {
            float error = sqrt(all_local_features[i][k].hError*all_local_features[i][k].hError + all_local_features[i][k].vError*all_local_features[i][k].vError);
            if (min > error) {
                min_ID = k;
                min = error;
            }
        }
        corners.push_back(all_local_features[i][min_ID]);
    }
    feature_points = corners;
    return false;
}

bool PhaseShifting_pattern_decoder::load_patterns(std::__cxx11::string directory){
    File_Manager file_manager = File_Manager(directory);
    file_manager.load_pattern_seq(vertical_pattern_seq.patterns, "Vertical/");
    file_manager.load_pattern_seq(horizontal_pattern_seq.patterns, "Horizontal/");
    Initialize_phase_pattern_seqs();
    return false;
}

bool PhaseShifting_pattern_decoder::Initialize_phase_pattern_seqs(){
    vertical_pattern_seq.pattern_size = vertical_pattern_seq.patterns.patterns_size;
    vertical_pattern_seq.phaseMap = Pattern(this->ID, "Vertical_Phase_Map", vertical_pattern_seq.pattern_size, CV_32FC1);

    horizontal_pattern_seq.pattern_size = horizontal_pattern_seq.patterns.patterns_size;
    horizontal_pattern_seq.phaseMap = Pattern(this->ID, "Horizontal_Phase_Map", horizontal_pattern_seq.pattern_size, CV_32FC1);
    return false;
}
