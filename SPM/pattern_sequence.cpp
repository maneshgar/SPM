#include "pattern_sequence.h"

Pattern_Sequence::Pattern_Sequence(){

}

Pattern_Sequence::Pattern_Sequence(uint input_ID, std::string input_name, int input_type, cv::Size input_patterns_size)
{
    this->ID = input_ID;
    this->name = input_name;
    initialize(input_patterns_size, input_type);
}

Pattern_Sequence::Pattern_Sequence(uint input_ID, std::string input_name)
{
    this->ID = input_ID;
    this->name = input_name;
}

bool Pattern_Sequence::initialize(cv::Size input_size, int input_type){
    this->type = input_type;
    this->patterns_size = input_size;
    this->initialized = true;
    return false;
}

bool Pattern_Sequence::display(std::string window_name){
    for (uint i = 0; i < sequence.size(); ++i) {
        sequence[i].display(window_name);
    }
    return false;
}

bool Pattern_Sequence::push_Element(Pattern input_Pattern){
    this->sequence.push_back(input_Pattern);
    return false;
}
