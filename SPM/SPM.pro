QT += core
QT += xml
QT -= gui

CONFIG += c++11

TARGET = SPM
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

INCLUDEPATH += -I/usr/include/opencv
INCLUDEPATH += -I/usr/local/include/openEXR

LIBS +=  /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so -lopencv_calib3d /usr/lib/x86_64-linux-gnu/libopencv_contrib.so -lopencv_contrib /usr/lib/x86_64-linux-gnu/libopencv_core.so -lopencv_core /usr/lib/x86_64-linux-gnu/libopencv_features2d.so -lopencv_features2d /usr/lib/x86_64-linux-gnu/libopencv_flann.so -lopencv_flann /usr/lib/x86_64-linux-gnu/libopencv_gpu.so -lopencv_gpu /usr/lib/x86_64-linux-gnu/libopencv_highgui.so -lopencv_highgui /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so -lopencv_imgproc /usr/lib/x86_64-linux-gnu/libopencv_legacy.so -lopencv_legacy /usr/lib/x86_64-linux-gnu/libopencv_ml.so -lopencv_ml /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so -lopencv_objdetect /usr/lib/x86_64-linux-gnu/libopencv_ocl.so -lopencv_ocl /usr/lib/x86_64-linux-gnu/libopencv_photo.so -lopencv_photo /usr/lib/x86_64-linux-gnu/libopencv_stitching.so -lopencv_stitching /usr/lib/x86_64-linux-gnu/libopencv_superres.so -lopencv_superres /usr/lib/x86_64-linux-gnu/libopencv_ts.so -lopencv_ts /usr/lib/x86_64-linux-gnu/libopencv_video.so -lopencv_video /usr/lib/x86_64-linux-gnu/libopencv_videostab.so -lopencv_videostab
LIBS += -lfftw3

SOURCES += \
    cminpack/enorm.cpp \
    cminpack/fdjac2.cpp \
    cminpack/lmdif.cpp \
    cminpack/lmdif0.cpp \
    cminpack/lmpar.cpp \
    cminpack/qrfac.cpp \
    cminpack/qrsolv.cpp \
    Geometry/GeometricObject.cpp \
    Geometry/GeometryExporter.cpp \
    Geometry/GeometryProcessing.cpp \
    Image/bilateral_filtering/fft_3D/support_3D.cpp \
    Image/Color.cpp \
    Image/Image.cpp \
    Image/ImageProcessing.cpp \
    cam2_proj_processing.cpp \
    camera.cpp \
    color_processing.cpp \
    feature_point.cpp \
    file_manager.cpp \
    GrayCode.cpp \
    imaging_device.cpp \
    lightproccessing.cpp \
    main.cpp \
    off_cam_calib.cpp \
    pattern.cpp \
    pattern_sequence.cpp \
    phaseshifting_pattern_decoder.cpp \
    phaseshifting_pattern_encoder.cpp \
    projector.cpp \
    reconstructor.cpp \
    spm.cpp

HEADERS += \
    cminpack/cminpak.h \
    cminpack/dpmpar.h \
    Geometry/GeometricObject.h \
    Geometry/GeometryExporter.h \
    Geometry/GeometryProcessing.h \
    Image/bilateral_filtering/fft_3D/convolution_3D.h \
    Image/bilateral_filtering/fft_3D/fill_3D.h \
    Image/bilateral_filtering/fft_3D/support_3D.h \
    Image/bilateral_filtering/include/array.h \
    Image/bilateral_filtering/include/array_n.h \
    Image/bilateral_filtering/include/chrono.h \
    Image/bilateral_filtering/include/fast_color_bf.h \
    Image/bilateral_filtering/include/fft_3D.h \
    Image/bilateral_filtering/include/geom.h \
    Image/bilateral_filtering/include/linear_bf.h \
    Image/bilateral_filtering/include/math_tools.h \
    Image/bilateral_filtering/include/mixed_vector.h \
    Image/bilateral_filtering/include/msg_stream.h \
    Image/Color.h \
    Image/Image.h \
    Image/ImageProcessing.h \
    BinaryCode.h \
    cam2_proj_processing.h \
    camera.h \
    color_processing.h \
    Edge.h \
    Face.h \
    feature_point.h \
    file_manager.h \
    FileUtilities.h \
    GrayCode.hpp \
    imaging_device.h \
    lightproccessing.h \
    main.h \
    off_cam_calib.h \
    pattern.h \
    pattern_sequence.h \
    phaseshifting_pattern_decoder.h \
    phaseshifting_pattern_encoder.h \
    projector.h \
    reconstructor.h \
    spm.h \
    structures.h \
    Utilities.h \
    cminpack/alta_interface.h


DISTFILES += \

SUBDIRS += \
