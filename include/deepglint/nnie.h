#ifndef VDEC_NNIE_H
#define VDEC_NNIE_H

#include "utils.h"
#include "thread"
#include "vector"
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include "sample_comm_nnie.h"
#include "sample_comm_svp.h"
#include <opencv2/opencv.hpp>

struct Bbox{
    float confidence;
    cv::Rect_<float> rect;
    bool deleted;
    int idx;
};

typedef struct DG_NNIE_WORKER_S
{
    SAMPLE_SVP_NNIE_PARAM_S *s_stSsdNnieParam;
    tbb::concurrent_bounded_queue<std::pair<std::pair<HI_U32, VIDEO_FRAME_INFO_S *>, IVE_IMAGE_S *>> *pipeline;
}NNIE_WORKER_S;

HI_VOID SAMPLE_COMM_VDEC_StartNnie(HI_S32 s32ChnNum, NNIE_WORKER_S *pstNnieWorker, pthread_t *pNnieThread);

#endif //VDEC_NNIE_H
