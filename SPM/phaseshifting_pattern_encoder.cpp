#include "phaseshifting_pattern_encoder.h"

PhaseShifting_Pattern_encoder::PhaseShifting_Pattern_encoder(){

}

PhaseShifting_Pattern_encoder::PhaseShifting_Pattern_encoder(uint input_sequence_length, cv::Size input_pattern_size, int input_pattern_type)
{

    pattersn_generated = false;

    vertical_PatSeq.orientation = 'V';
    vertical_PatSeq.pattern_size = pattern_size;
    vertical_PatSeq.phaseMap = Pattern(0, "Vertical_Phase_Map", input_pattern_size, input_pattern_type);
    vertical_PatSeq.patterns = Pattern_Sequence(0, "Vertical_PhaseShifting_Patterns", input_pattern_type, input_pattern_size);

    horizontal_PatSeq.orientation = 'H';
    horizontal_PatSeq.pattern_size = pattern_size;
    horizontal_PatSeq.phaseMap = Pattern(0, "Horizontal_Phase_Map", input_pattern_size, input_pattern_type);
    horizontal_PatSeq.patterns = Pattern_Sequence(0, "Horizontal_PhaseShifting_Patterns", input_pattern_type, input_pattern_size);

    length = input_sequence_length;
    pattern_size = input_pattern_size;
    pattern_type = input_pattern_type;
    CheckerBoard = Pattern(3, "chekerBoard", cv::Mat::zeros(pattern_size, CV_32FC1));
    this->black = Pattern(1,"black", cv::Mat(this->pattern_size, CV_8UC1, cv::Scalar(0)));
    this->white = Pattern(2, "white", cv::Mat(this->pattern_size, CV_8UC1, cv::Scalar(255)));
    for (int i = 0; i < pattern_size.height/25; ++i) {
        white.image.row(i) = cv::Scalar(0);
        white.image.row(pattern_size.height - (i+1)) = cv::Scalar(0);
    }
    for (int i = 0; i < pattern_size.width/25; ++i) {
        white.image.col(i) = cv::Scalar(0);
        white.image.col(pattern_size.width - (i+1)) = cv::Scalar(0);
    }
}

bool PhaseShifting_Pattern_encoder::generate_patterns(){
    generate_syncronize_image();
    generate_sequence(vertical_PatSeq);
    generate_sequence(horizontal_PatSeq);
    for (int i = 0; i < vertical_PatSeq.patterns.sequence.size(); ++i) {
        cv::imwrite("data/calibration/patterns/phase_shifting/v"+vertical_PatSeq.patterns.sequence[i].name+".png" , vertical_PatSeq.patterns.sequence[i].image);
    }
    for (int i = 0; i < horizontal_PatSeq.patterns.sequence.size(); ++i) {
        cv::imwrite("data/calibration/patterns/phase_shifting/h"+horizontal_PatSeq.patterns.sequence[i].name+".png" , horizontal_PatSeq.patterns.sequence[i].image);
    }
    cv::imwrite("data/calibration/patterns/phase_shifting/syncronized.tif" , syncronize_image.image);
    cv::imwrite("data/calibration/patterns/phase_shifting/black.tif" , black.image);
    cv::imwrite("data/calibration/patterns/phase_shifting/white.tif" , white.image );

    pattersn_generated = true;
    return false;
}

bool PhaseShifting_Pattern_encoder::generate_syncronize_image(){
    cv::Mat image(this->pattern_size, CV_8UC3, cv::Scalar(0));
    cv::Point2f center;
    center.x = pattern_size.width / 2;
    center.y = pattern_size.height / 2;
    int color = 0;
    int iterations = 5 * scale;
    for (int it = 0; it < iterations; ++it) {
        color += 255/iterations;
        cv::circle(image, center, (iterations-it), cv::Scalar(color,color,color), -1);
    }

    this->syncronize_image = Pattern(1,"syncronize_image", image.clone());
}

bool PhaseShifting_Pattern_encoder::display_patterns(std::string window_name){
    vertical_PatSeq.patterns.display(window_name);
    horizontal_PatSeq.patterns.display(window_name);
    return false;
}

bool PhaseShifting_Pattern_encoder::generate_sequence(Phase_Pattern_Seq &pattern_seq){
    cv::Mat vP(pattern_size, CV_32FC1);

    for (int pattern_iterator = 0; pattern_iterator < length; ++pattern_iterator) {

        Pattern temp_pattern = Pattern(pattern_iterator+1, std::to_string(pattern_iterator), cv::Size(pattern_size.width, pattern_size.height), pattern_type);

        for (float i = 0; i < temp_pattern.size.height; ++i) {
            for (float j = 0; j < temp_pattern.size.width; ++j) {
                float phase_amount = 0;
                if (pattern_seq.orientation == 'V') {
                    phase_amount = (2 * M_PI * j / pattern_size.width ) * (255/(2*M_PI));
                } else if(pattern_seq.orientation == 'H'){
                    phase_amount = (2 * M_PI * i / pattern_size.height) * (255/(2*M_PI));
                }
                phase_amount /= scale;
                vP.at<float>(i,j) = phase_amount;
                temp_pattern.image.at<uchar> (i,j) = 127.5 * (1+cos(phase_amount+(2*pattern_iterator*M_PI/length)));
            }
        }
        pattern_seq.patterns.push_Element(temp_pattern);
    }
    vP /= 255.0;
//    pattern_seq.phaseMap.image = vP.clone();
//    pattern_seq.phaseMap.display("")
    cv::imshow("forsequence", vP);
    cv::waitKey(0);
    return false;
}

bool PhaseShifting_Pattern_encoder::is_patterns_generated(){
    if(pattersn_generated)
        return true;
    return false;
}

bool PhaseShifting_Pattern_encoder::generate_phaseMap(Phase_Pattern_Seq &pattern_seq){
    pattern_seq.phaseMap = Pattern(1, "phase_map", pattern_size, CV_32FC1);
    for (int i = 0; i < pattern_size.height; ++i) {
        for (int j = 0; j < pattern_size.width; ++j) {
            float phase;
            if (pattern_seq.orientation == 'V') {
                phase = (2 * M_PI * j / pattern_size.width ) * (255/(2*M_PI));
            } else if (pattern_seq.orientation == 'H'){
                phase = (2 * M_PI * i / pattern_size.height) * (255/(2*M_PI));
            }
            pattern_seq.phaseMap.image.at<float> (i,j) = phase/scale;
        }
    }
//    cv::imshow("phaseMapGenerated", pattern_seq.phaseMap.image/255.0);
//    cv::waitKey(0);
}

bool PhaseShifting_Pattern_encoder::encode_featurePoints(int vertical_distance, int horizontal_distance){
    generate_phaseMap(vertical_PatSeq);
    generate_phaseMap(horizontal_PatSeq);
    cv::Mat vertical_checker(pattern_size, CV_32FC1);
    int count = 0;
    int color = 0;
    for (int var = 0; var < pattern_size.width; ++var) {
        if (var % horizontal_distance == 0 && var!=0) {
            count++;
            Feature_Point temp = Feature_Point();
            temp.ID = count;
            temp.pX = var;
            temp.vPhase = vertical_PatSeq.phaseMap.image.at<float>(100, var);
//            std::cout << temp.pX << "   " << temp.vPhase << std::endl;
            vertical_PatSeq.featurePoint_indicator.push_back(temp);
            color = (color+1)%2;

        }
        vertical_checker.col(var) = cv::Scalar(color);
    }

    count = 0;
    color = 0;
    cv::Mat horizontal_checker(pattern_size, CV_32FC1);
    for (int var = 0; var < pattern_size.height; ++var) {
        if (var % vertical_distance == 0 && var!=0) {
            count++;
            Feature_Point temp = Feature_Point();
            temp.ID = count;
            temp.pY = var;
            temp.hPhase = horizontal_PatSeq.phaseMap.image.at<float>(var, 100);
//            std::cout << temp.pY << "   " << temp.hPhase << std::endl;
            horizontal_PatSeq.featurePoint_indicator.push_back(temp);
            color = (color+1)%2;
        }
        horizontal_checker.row(var) = cv::Scalar(color);
    }

    CheckerBoard.image = vertical_checker ^ horizontal_checker;
    cv::imwrite("data/calibration/patterns/checker.tif" , CheckerBoard.image * 255);
//    cv::imshow("checker", CheckerBoard.image);
//    cv::waitKey(0);
}

bool PhaseShifting_Pattern_encoder::display_encoded_features(){

    std::vector<cv::KeyPoint> all_feature_keyPoints;
    for (int i = 0; i < vertical_PatSeq.featurePoint_indicator.size(); ++i) {
        for (int j = 0; j < horizontal_PatSeq.featurePoint_indicator.size(); ++j) {
            cv::KeyPoint temp(cv::KeyPoint(vertical_PatSeq.featurePoint_indicator[i].pX, horizontal_PatSeq.featurePoint_indicator[j].pY, 1));
            all_feature_keyPoints.push_back(temp);
        }
    }
    Pattern features(1,"encoded_featurePoints", pattern_size, pattern_type);
    features.image = cv::Scalar(255);
    cv::drawKeypoints(features.image,all_feature_keyPoints, features.image, cv::Scalar(0,0,255));
    features.display("Encoded_Feature_Points");
    cv::imwrite("encodedFeatures.png", features.image);
}















