#include <fileReader.h>
#include <Reconstructor.h>
#include <log.hpp>
#include <glm/gtx/string_cast.hpp>
#include <ReconstructorCPU.h>

#define PROJECTOR_WIDTH 1280//1024 // 768
#define PROJECTOR_HEIGHT 800//540 //1024
int main()
{
    LOG::restartLog();
    
    SLS::FileReader *rightCam=new SLS::FileReader("rightCamera");
    SLS::FileReader *leftCam= new SLS::FileReader("leftCamera");
//    SLS::FileReader *topCam= new SLS::FileReader("topCamera");

    rightCam->loadConfig("data/rightCam/calib/CalibRes.xml");
    leftCam->loadConfig("data/leftCam/calib/CalibRes.xml");
//    topCam->loadConfig("data/topCam/calib/CalibRes.xml");

    rightCam->loadImages("data/rightCam/dataset/");
    leftCam->loadImages("data/leftCam/dataset/");
//    topCam->loadImages("data/topCam/dataset/");
    
    SLS::ReconstructorCPU reconstruct(PROJECTOR_WIDTH, PROJECTOR_HEIGHT);

    reconstruct.addCamera(rightCam);
    reconstruct.addCamera(leftCam);
//    reconstruct.addCamera(topCam);

    reconstruct.reconstruct();
//    reconstruct.calibrate_projector();

    SLS::exportOBJ("Outputs/point_cloud.obj",  reconstruct);
    SLS::exportPLYGrid("Outputs/point_cloud.ply",  reconstruct);
    SLS::exportPLY("Outputs/point_cloud&Color.ply",  reconstruct);

    std::cout << "DONE! \n" ;
    LOG::writeLog("DONE!\n");
    
    return 0;
}
