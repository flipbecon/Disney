#ifndef VDEC_IVE_H
#define VDEC_IVE_H

#include "utils.h"
#include "thread"

#define IVE_ALIGN 16

typedef struct DG_IVE_WORKER_S
{
    tbb::concurrent_bounded_queue<VIDEO_FRAME_INFO_S *> *frameQueue;
    tbb::concurrent_bounded_queue<IVE_IMAGE_S *> *imageQueue;
}IVE_WORKER_S;

HI_VOID *SAMPLE_COMM_VDEC_IVE(HI_VOID *pArgs);

HI_VOID SAMPLE_COMM_VDEC_StartIve(HI_S32 s32ChnNum, IVE_WORKER_S *pstIveWorker, pthread_t *pIveThread);

#endif //VDEC_IVE_H