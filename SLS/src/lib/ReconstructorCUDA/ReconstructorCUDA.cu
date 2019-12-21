#include <device_launch_parameters.h>
#include "ReconstructorCUDA.cuh"
#include "fileReaderCUDA.cuh"
namespace SLS
{

ReconstructorCUDA :: ReconstructorCUDA(const size_t projX, const size_t projY): 
    Reconstructor()
{
    projector_ = new Projector(projX, projY);
}
ReconstructorCUDA::~ReconstructorCUDA(){
    for (auto &cam: cameras_)
        delete cam;
    delete projector_;
}
void ReconstructorCUDA::addCamera(Camera *cam)
{
    cameras_.push_back(cam);
}
void ReconstructorCUDA::reconstruct()
{
    // For each camera, hack
    GPUBuckets buckets[2] =
    {
        GPUBuckets( 1024*768,110),
        GPUBuckets( 1024*768,110)
    };
    
    /**** Profile *****/
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    cudaEventRecord(start);
    /**/


    for(size_t camIdx = 0; camIdx < cameras_.size(); camIdx++)
    {
        FileReaderCUDA* cam = (FileReaderCUDA*)cameras_[camIdx];
        LOG::writeLog("Generating reconstruction bucket for \"%s\" ... \n", cam->getName().c_str());
        cam->computeShadowsAndThresholds();    // can it be done in GPU? Yes
        size_t x=0,y=0,xTimesY=0;
        cam->getResolution(x,y);
        xTimesY=x*y;
        //Skip first two frames
        cam->getNextFrame();
        cam->getNextFrame();
        // Load all images into GPU memory
        uchar *images_d=nullptr;
        gpuErrchk(cudaMalloc((void**)&images_d, sizeof(uchar)*xTimesY*projector_->getRequiredNumFrames()*2));
        Dynamic_Bitset_Array bitsetArray(xTimesY, projector_->getRequiredNumFrames());

        //Preparing data
        for (size_t i=0; i<projector_->getRequiredNumFrames(); i++)
        {
            auto frm = cam->getNextFrame();
            auto invFrm = cam->getNextFrame();
            assert(frm.isContinuous() && invFrm.isContinuous());
            gpuErrchk( cudaMemcpy( &images_d[xTimesY*2*i], frm.data, 
                    sizeof(uchar)*xTimesY, cudaMemcpyHostToDevice));
            gpuErrchk( cudaMemcpy( &images_d[xTimesY*(2*i+1)], invFrm.data, 
                    sizeof(uchar)*xTimesY, cudaMemcpyHostToDevice));
        }

        Kernel::genPatternArray<<<200,200>>> (
                images_d, 
                projector_->getRequiredNumFrames(),
                xTimesY,
                cam->getWhiteThreshold(0),
                cam->getMask()->getGPUOBJ(),
                bitsetArray.getGPUOBJ()
                );
        //Check for errors
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaFree(images_d)); // Release the heavy image array

        Kernel::buildBuckets<<<200, 200>>> (
             cam->getMask()->getGPUOBJ(),
             bitsetArray.getGPUOBJ(),
             xTimesY,
             buckets[camIdx].getGPUOBJ()
            );
        gpuErrchk(cudaPeekAtLastError());
    }


    // A lot of hacks down there, need to be refactored
    auto camera0 = (FileReaderCUDA*)(cameras_[0]);
    auto camera1 = (FileReaderCUDA*)(cameras_[1]);
    float* cloud = nullptr;
    size_t resX, resY;
    camera0->getResolution(resX, resY);

    gpuErrchk ( cudaMalloc((void**)&cloud, buckets[0].getNumBKTs()*sizeof(float)*4));

    // Reconstructing point cloud
    LOG::writeLog("Reconstructing point cloud ...\n");
    Kernel::getPointCloud2Cam<<<200,200>>>(
            buckets[0].getGPUOBJ(),
            camera0->getMask()->getGPUOBJ(),
            camera0->getDeviceCamMat(),
            camera0->getDeviceDistMat(),
            camera0->getDeviceCamTransMat(),

            buckets[1].getGPUOBJ(),
            camera1->getMask()->getGPUOBJ(),
            camera1->getDeviceCamMat(),
            camera1->getDeviceDistMat(),
            camera1->getDeviceCamTransMat(),
            resX,resY,
            cloud
            );
    gpuErrchk( cudaPeekAtLastError());
    pointCloud_.resize(buckets[0].getNumBKTs()*4);
    gpuErrchk( 
            cudaMemcpy(  &pointCloud_[0],cloud, buckets[0].getNumBKTs()*sizeof(float)*4, cudaMemcpyDeviceToHost));
    /**** Profile *****/
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    float milliseconds = 0;
    cudaEventElapsedTime(&milliseconds, start, stop);
    LOG::writeLog("GPU Time : %fms\n", milliseconds);
    /*****/
    gpuErrchk( cudaFree(cloud));
    LOG::writeLog("Done\n");
}

namespace Kernel{
// Kernels 

__global__ void genPatternArray(
        const uchar * imgs,
        size_t numImgs,
        size_t XtimesY,
        uchar whiteThreshold,
        Dynamic_Bitset_Array_GPU mask,
        Dynamic_Bitset_Array_GPU patterns
        )
{
    uint idx = blockIdx.x*blockDim.x + threadIdx.x;
    uint stride = blockDim.x * gridDim.x;
    while (idx < XtimesY)   // For each pixel
    {
        for (size_t i = 0; i<numImgs; i++)
        {
            if (!mask.getBit(0, idx)) 
            {
                // set the bit to black, unnecessary
                for (size_t j=0; j<numImgs; j++)
                    patterns.clearBit(j, idx);
                continue;
            }
            uchar pixel = imgs[ idx + XtimesY*(2*i)];
            uchar invPixel = imgs[ idx + XtimesY*(2*i+1)];
            if (invPixel > pixel && invPixel-pixel >= whiteThreshold)
                patterns.clearBit(numImgs-1-i, idx);
            else if (pixel > invPixel && pixel-invPixel > whiteThreshold)
                patterns.setBit(numImgs-1-i, idx);
            else
                mask.clearBit(0, idx);
        }
        idx += stride;
    }
}


__global__ void buildBuckets(
        Dynamic_Bitset_Array_GPU mask,
        Dynamic_Bitset_Array_GPU patterns,
        size_t XtimesY,

        GPUBucketsObj bkts
        )
{
    uint idx = blockIdx.x*blockDim.x + threadIdx.x;
    uint stride = blockDim.x * gridDim.x;
    while (idx < XtimesY)   // For each pixel
    {
        glm::uvec2 bkt2v = patterns.to_uint_gray(idx);
        if (bkt2v.x < 1024 && bkt2v.y < 768 && mask.getBit(0, idx))
            bkts.add2Bucket(idx, bkt2v.x+bkt2v.y*1024);
        idx += stride;
    }
}

__global__ void getPointCloud2Cam(
        GPUBucketsObj buckets0,
        Dynamic_Bitset_Array_GPU mask0,
        float *camMat0,
        float *distMat0,
        float *camTransMat0,

        GPUBucketsObj buckets1,
        Dynamic_Bitset_Array_GPU mask1,
        float *camMat1,
        float *distMat1,
        float *camTransMat1,


        uint camResX,
        uint camResY,

        float* pointCloud
        )
{
    //Each thread takes care of one projector pixel
    uint idx = blockIdx.x*blockDim.x + threadIdx.x;
    uint stride = blockDim.x * gridDim.x;
    while (idx < buckets0.NUM_BKTS_)   // For each pixel
    {
        if ( buckets0.count_[idx] == 0 || buckets1.count_[idx] == 0) 
        {
            memset( &pointCloud[4*idx], 0, sizeof(float)*4);
            pointCloud[4*idx+3] = float(0.0);
        }
        else
        {
            //Undistorted pixels
            float minDist = 99999.0;
            float minMidPoint[4];

            float avgPoint[4];
            memset(avgPoint, 0, sizeof(float)*4);
            uint ptCount = 0;

            for (uint i=0; i<buckets0.count_[idx]; i++)
                for (uint j=0; j<buckets1.count_[idx]; j++)
                {

                    float undistorted0[2];
                    float undistorted1[2];

                    //Pick the first pixel in both buckets to test
                    undistortPixel(
                            buckets0.data_[idx*buckets0.MAX_CNT_PER_BKT_+i],
                            camResX, camResY,
                            camMat0, distMat0,
                            undistorted0);
                    undistortPixel(
                            buckets1.data_[idx*buckets1.MAX_CNT_PER_BKT_+j],
                            camResX, camResY,
                            camMat1, distMat1,
                            undistorted1);

                    float origin0[4];
                    float origin1[4];
                    float dir0[4];
                    float dir1[4];

                    getRay(undistorted0, camMat0, camTransMat0, 
                            origin0, dir0);
                    getRay(undistorted1, camMat1, camTransMat1, 
                            origin1, dir1);

                    float midPoint[4];

                    auto dist = getMidPoint(
                            origin0, dir0, origin1, dir1,
                            midPoint);
                    avgPoint[0] += midPoint[0];
                    avgPoint[1] += midPoint[1];
                    avgPoint[2] += midPoint[2];
                    avgPoint[3] += midPoint[3];
                    ptCount++;
                    if (dist < minDist)
                    {
                        minDist = dist;
                        memcpy (minMidPoint, midPoint, sizeof(float)*4);
                    }
                }
            //if (minDist < 0.3)
            avgPoint[0] /= (float)ptCount;
            avgPoint[1] /= (float)ptCount;
            avgPoint[2] /= (float)ptCount;
            avgPoint[3] = 1.0;
            memcpy ( &pointCloud[4*idx], avgPoint, sizeof(float)*4);
            //else
                //memset( &pointCloud[4*idx], 0, sizeof(float)*4);
        }
        idx += stride;
    }
}

} // namespace Kernel
} // namespace SLS
