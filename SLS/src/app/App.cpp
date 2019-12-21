#include <core/fileReader.h>
#include <core/Reconstructor.h>
#include <core/log.hpp>
#include <glm/gtx/string_cast.hpp>
#include <core/ReconstructorCPU.h>

int main()
{
    LOG::restartLog();
    

    SLS::FileReader *rightCam=new SLS::FileReader("rightCamera");
    SLS::FileReader *leftCam= new SLS::FileReader("leftCamera");
    SLS::FileReader *topCam= new SLS::FileReader("topCamera");

    rightCam->loadImages("../../../data/alexander/rightCam/dataset1/");
    leftCam->loadImages("../../../data/alexander/leftCam/dataset1/");
    topCam->loadImages("../../../data/alexander/topCam/dataset1/");

    rightCam->loadConfig("../../../data/alexander/rightCam/calib/output/calib.xml");
    leftCam->loadConfig("../../../data/alexander/leftCam/calib/output/calib.xml");
    topCam->loadConfig("../../../data/alexander/topCam/calib/output/calib.xml");

    
    SLS::ReconstructorCPU reconstruct(1024,1024);

    reconstruct.addCamera(rightCam);
    reconstruct.addCamera(leftCam);
    reconstruct.addCamera(topCam);
    reconstruct.reconstruct();

    SLS::exportOBJ("test.obj",  reconstruct);
    SLS::exportPLYGrid("test.ply",  reconstruct);

    LOG::writeLog("DONE!\n");
    
    return 0;
}
