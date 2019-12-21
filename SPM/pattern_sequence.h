#ifndef PATTERN_SEQUENCE_H
#define PATTERN_SEQUENCE_H

#include <string>
#include <vector>

#include "pattern.h"

class Pattern_Sequence
{
public:

    Pattern_Sequence();
    Pattern_Sequence(uint input_ID, std::string input_name, int input_type, cv::Size input_patterns_size);
    Pattern_Sequence(uint input_ID, std::string input_name);

    bool display(std::string window_name);
    bool push_Element(Pattern input_Pattern);
    bool initialize(cv::Size input_size, int input_type);
    uint ID;
    int type;
    cv::Size patterns_size;
    std::string name;
    std::vector<Pattern> sequence;

    bool initialized = false;

};

#endif // PATTERN_SEQUENCE_H
