#ifndef SPM_H
#define SPM_H

#include "camera.h"
#include "phaseshifting_pattern_encoder.h"
#include "phaseshifting_pattern_decoder.h"
#include "off_cam_calib.h"
#include "reconstructor.h"
#include "Geometry/GeometricObject.h"
#include "cam2_proj_processing.h"
#include "projector.h"
#include "Utilities.h"
#include "lightproccessing.h"


#define LCD_WIDTH 1920
#define LCD_HEIGH 1080

#define CAMERA_WIDTH 1920 // 4896 alex
#define CAMERA_HEIGHT 1080 //3264 alex

#define PROJECTOR_WIDTH 1280 // 1024 alex
#define PROJECTOR_HEIGHT 800

#define PHASESHIFTING_STEP 4
#define FEATURE_DISTANCE 50

class SPM
{
public:

    SPM();
    ~SPM();

    bool display_phaseShifted_patterns_func(uint iteration);
    bool calibrate_cameras_func();
    bool reconstruct_func();
    bool pointCloudToMesh_func();
    void light_proccessing_func();
    bool smooth_geometry();

    bool display_dot_sequence_func(){
        Reconstructor recontructor(camera_left, camera_right, camera_top, projector);
        recontructor.display_dot_pattern();
    }
    bool load_system_func(bool check);

private:

    int numberOfCameras = 3;
    Camera camera_top;
    Camera camera_left;
    Camera camera_right;

    GeometricObject Geometric_Obj;
    Projector projector;

    bool save_data();
};

#endif // SPM_H
