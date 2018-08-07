#ifndef UTILS_H
#define UTILS_H

#include <unistd.h>

#include "hi_type.h"
#include "mpi_ive.h"
#include "mpi_sys.h"
#include "hi_comm_video.h"

HI_U16 IVE_CalcStride(HI_U32 u32Width, HI_U8 u8Align);

HI_S32 DGIveSP420ToBGR24(IVE_IMAGE_S *pSrcImage, IVE_IMAGE_S *pDstImage);

HI_S32 DGIveBGR24ToSP420(IVE_IMAGE_S *pSrcImage, IVE_IMAGE_S *pDstImage);

HI_S32 DGIveSP420ToHSV(IVE_IMAGE_S *pSrcImage, IVE_IMAGE_S *pDstImage);

HI_S32 DGSP420ToBGR24Planar(VIDEO_FRAME_S *stSrcFrame, IVE_IMAGE_S *stDstImage);

HI_S32 DGIveResize(IVE_SRC_IMAGE_S astSrc[], IVE_DST_IMAGE_S astDst[]);

#endif