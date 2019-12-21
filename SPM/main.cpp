#include <QCoreApplication>
#include "main.h"
#include <time.h>

int main(int argc, char *argv[])
{
    //    Tester test;
    //    test.test_phS_pattern_decoder();

    bool cont = true;
    while(cont){
        int func = get_function();
        cont = do_function(func);
    }
    std::cout << "Program Ended Successfuly!\n";
}

int get_function(){
    int id = 0;
    std::cout << "Please choose the function: \n1- Display phaseshifted Patterns.\n2- Start Calibration of Left and Right Cameras.\n"
                 "3- Load System (Cameras)\n4- Load System and Reconstruct\n"
                 "5- Generate Red Dots\n6- Point Cloud to Mesh\n7- Light Processing\n8- Generate GrayCode\n";
    std::cin >> id;
    return id;
}

bool do_function(uint id){
    int input = 0;
    switch (id){
    case 1:
        std::cout << "How many times?\n";
        std::cin >> input;
        spm.display_phaseShifted_patterns_func(input);
        break;
    case 2:
        spm.calibrate_cameras_func();
        break;
    case 3:
        spm.load_system_func(true);
        break;
    case 4:
        spm.load_system_func(true);
        spm.reconstruct_func();
        break;
    case 5:
        spm.display_dot_sequence_func();
        break;
    case 6:
        spm.load_system_func(true);
        spm.pointCloudToMesh_func();
        break;
    case 7:
        spm.light_proccessing_func();
        break;
    case 8:
//        spm.binaryCode();
        break;
    case 9:
        spm.smooth_geometry();
        break;
    case 0:
        return false;
    }
    return true;
}
