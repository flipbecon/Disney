#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/time.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include <opencv2/opencv.hpp>

#include "utils.h"
#include "sample_comm.h"
#include "sample_comm_svp.h"
#include "sample_comm_nnie.h"
#include "sample_svp_nnie_software.h"


#define IVE_ALIGN 16
#define INPUT_VIDEO_WIDTH  1920
#define INPUT_VIDEO_HEIGHT 1080

struct Bbox{
    float confidence;
    cv::Rect_<float> rect;
    bool deleted;
    int idx;
};

bool mycmp(struct Bbox b1, struct Bbox b2)
{
    return b1.confidence>b2.confidence;
}

void nms(std::vector<struct Bbox>& p, float threshold) {

    sort(p.begin(), p.end(), mycmp);
    for (int i = 0; i < p.size(); i++) {
        if (p[i].deleted)
            continue;
        for (int j = i + 1; j < p.size(); j++) {

            if (!p[j].deleted) {
                cv::Rect_<float> intersect = p[i].rect & p[j].rect;
                float iou = intersect.area() * 1.0
                            / (p[i].rect.area() + p[j].rect.area()
                               - intersect.area());
                if (iou > threshold) {
                    p[j].deleted = true;
                }
            }
        }
    }
}

HI_VOID SAMPLE_VDEC_HandleSig(HI_S32 signo)
{
    if (SIGINT == signo || SIGTSTP == signo || SIGTERM == signo)
    {
        SAMPLE_COMM_SYS_Exit();
        printf("\033[0;31mprogram exit abnormally!\033[0;39m\n");
    }

    exit(0);
}

HI_VOID SAMPLE_COMM_VDEC_SaveYUVFile_Linear8Bit(FILE* pfd, VIDEO_FRAME_S* pVBuf)
{
    HI_U8* pY_map = NULL;
    HI_U8* pC_map = NULL;
    unsigned int w, h;
    HI_U8* pMemContent;
    HI_U8 *pTmpBuff=HI_NULL;
    HI_U64 phy_addr;
    HI_U32 u32Size;
    PIXEL_FORMAT_E  enPixelFormat = pVBuf->enPixelFormat;
    HI_U32 u32UvHeight;

    if (PIXEL_FORMAT_YVU_SEMIPLANAR_420 == enPixelFormat)
    {
        u32Size = (pVBuf->u32Stride[0]) * (pVBuf->u32Height) * 3 / 2;
        u32UvHeight = pVBuf->u32Height / 2;
    }
    else if(PIXEL_FORMAT_YVU_SEMIPLANAR_422 == enPixelFormat)
    {
        u32Size = (pVBuf->u32Stride[0]) * (pVBuf->u32Height) * 2;
        u32UvHeight = pVBuf->u32Height;
    }
    else if(PIXEL_FORMAT_YUV_400 == enPixelFormat)
    {
        u32Size = (pVBuf->u32Stride[0]) * (pVBuf->u32Height);
        u32UvHeight = pVBuf->u32Height;
    }
    else
    {
        printf("%s %d: This YUV format is not support!\n",__func__, __LINE__);
        return;
    }

    phy_addr = pVBuf->u64PhyAddr[0];

    pY_map = (HI_U8*) HI_MPI_SYS_Mmap(phy_addr, u32Size);
    if (HI_NULL == pY_map)
    {
        SAMPLE_PRT("HI_MPI_SYS_Mmap for pY_map fail!!\n");
        return;
    }

    pC_map = pY_map + (pVBuf->u32Stride[0]) * (pVBuf->u32Height);

    fprintf(stderr, "saving......Y......");
    fflush(stderr);
    for (h = 0; h < pVBuf->u32Height; h++)
    {
        pMemContent = pY_map + h * pVBuf->u32Stride[0];
        fwrite(pMemContent, pVBuf->u32Width, 1, pfd);
    }

    if(PIXEL_FORMAT_YUV_400 != enPixelFormat)
    {
        fflush(pfd);
        fprintf(stderr, "U......");
        fflush(stderr);

        pTmpBuff = (HI_U8 *)malloc(pVBuf->u32Stride[0]);
        if(HI_NULL == pTmpBuff)
        {
            SAMPLE_PRT("malloc pTmpBuff (size=%d) fail!!!\n",pVBuf->u32Stride[0]);
            return;
        }
        for (h = 0; h < u32UvHeight; h++)
        {
            pMemContent = pC_map + h * pVBuf->u32Stride[1];

            pMemContent += 1;

            for (w = 0; w < pVBuf->u32Width / 2; w++)
            {
                pTmpBuff[w] = *pMemContent;
                pMemContent += 2;
            }
            fwrite(pTmpBuff, pVBuf->u32Width / 2, 1, pfd);
        }
        fflush(pfd);

        fprintf(stderr, "V......");
        fflush(stderr);
        for (h = 0; h < u32UvHeight; h++)
        {
            pMemContent = pC_map + h * pVBuf->u32Stride[1];

            for (w = 0; w < pVBuf->u32Width / 2; w++)
            {
                pTmpBuff[w] = *pMemContent;
                pMemContent += 2;
            }
            fwrite(pTmpBuff, pVBuf->u32Width / 2, 1, pfd);
        }
        free(pTmpBuff);
        pTmpBuff = HI_NULL;
    }
    fflush(pfd);

    fprintf(stderr, "done!\n");
    fflush(stderr);

    HI_MPI_SYS_Munmap(pY_map, u32Size);
    pY_map = HI_NULL;

    return;
}

/******************************************************************************
* function : SSD software deinit
******************************************************************************/
static HI_S32 SAMPLE_SVP_NNIE_Ssd_SoftwareDeinit(SAMPLE_SVP_NNIE_SSD_SOFTWARE_PARAM_S* pstSoftWareParam)
{
    HI_S32 s32Ret = HI_SUCCESS;
    SAMPLE_SVP_CHECK_EXPR_RET(NULL== pstSoftWareParam,HI_INVALID_VALUE,SAMPLE_SVP_ERR_LEVEL_ERROR,
                              "Error, pstSoftWareParam can't be NULL!\n");
    if(0!=pstSoftWareParam->stPriorBoxTmpBuf.u64PhyAddr && 0!=pstSoftWareParam->stPriorBoxTmpBuf.u64VirAddr)
    {
        SAMPLE_SVP_MMZ_FREE(pstSoftWareParam->stPriorBoxTmpBuf.u64PhyAddr,
                            pstSoftWareParam->stPriorBoxTmpBuf.u64VirAddr);
        pstSoftWareParam->stPriorBoxTmpBuf.u64PhyAddr = 0;
        pstSoftWareParam->stPriorBoxTmpBuf.u64VirAddr = 0;
    }
    return s32Ret;
}
/******************************************************************************
* function : Ssd Deinit
******************************************************************************/
static HI_S32 SAMPLE_SVP_NNIE_Ssd_Deinit(SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam,
                                         SAMPLE_SVP_NNIE_SSD_SOFTWARE_PARAM_S* pstSoftWareParam,SAMPLE_SVP_NNIE_MODEL_S *pstNnieModel)
{
    HI_S32 s32Ret = HI_SUCCESS;
    /*hardware deinit*/
    if(pstNnieParam!=NULL)
    {
        s32Ret = SAMPLE_COMM_SVP_NNIE_ParamDeinit(pstNnieParam);
        SAMPLE_SVP_CHECK_EXPR_TRACE(HI_SUCCESS != s32Ret,SAMPLE_SVP_ERR_LEVEL_ERROR,
                                    "Error,SAMPLE_COMM_SVP_NNIE_ParamDeinit failed!\n");
    }
    /*software deinit*/
    if(pstSoftWareParam!=NULL)
    {
        s32Ret = SAMPLE_SVP_NNIE_Ssd_SoftwareDeinit(pstSoftWareParam);
        SAMPLE_SVP_CHECK_EXPR_TRACE(HI_SUCCESS != s32Ret,SAMPLE_SVP_ERR_LEVEL_ERROR,
                                    "Error,SAMPLE_SVP_NNIE_Ssd_SoftwareDeinit failed!\n");
    }
    /*model deinit*/
    if(pstNnieModel!=NULL)
    {
        s32Ret = SAMPLE_COMM_SVP_NNIE_UnloadModel(pstNnieModel);
        SAMPLE_SVP_CHECK_EXPR_TRACE(HI_SUCCESS != s32Ret,SAMPLE_SVP_ERR_LEVEL_ERROR,
                                    "Error,SAMPLE_COMM_SVP_NNIE_UnloadModel failed!\n");
    }
    return s32Ret;
}
/******************************************************************************
* function : Ssd software para init
******************************************************************************/
static HI_S32 SAMPLE_SVP_NNIE_Ssd_SoftwareInit(SAMPLE_SVP_NNIE_CFG_S* pstCfg,
                                               SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam, SAMPLE_SVP_NNIE_SSD_SOFTWARE_PARAM_S* pstSoftWareParam)
{
    HI_U32 i = 0;
    HI_S32 s32Ret = HI_SUCCESS;
    HI_U32 u32ClassNum = 0;
    HI_U32 u32TotalSize = 0;
    HI_U32 u32DstRoiSize = 0;
    HI_U32 u32DstScoreSize = 0;
    HI_U32 u32ClassRoiNumSize = 0;
    HI_U32 u32TmpBufTotalSize = 0;
    HI_U64 u64PhyAddr = 0;
    HI_U8* pu8VirAddr = NULL;

    /*Set Conv Parameters*/
    /*the SSD sample report resule is after permute operation,
     conv result is (C, H, W), after permute, the report node's
     (C1, H1, W1) is (H, W, C), the stride of report result is aligned according to C dim*/
    for(i = 0; i < 12; i++)
    {
        pstSoftWareParam->au32ConvHeight[i] = pstNnieParam->pstModel->astSeg[0].astDstNode[i].unShape.stWhc.u32Chn;
        pstSoftWareParam->au32ConvWidth[i] = pstNnieParam->pstModel->astSeg[0].astDstNode[i].unShape.stWhc.u32Height;
        pstSoftWareParam->au32ConvChannel[i] = pstNnieParam->pstModel->astSeg[0].astDstNode[i].unShape.stWhc.u32Width;
        if(i%2==1)
        {
            pstSoftWareParam->au32ConvStride[i/2] = SAMPLE_SVP_NNIE_ALIGN16(pstSoftWareParam->au32ConvChannel[i]*sizeof(HI_U32))/sizeof(HI_U32);
        }
    }

    /*Set PriorBox Parameters*/
    pstSoftWareParam->au32PriorBoxWidth[0] = 38;
    pstSoftWareParam->au32PriorBoxWidth[1] = 19;
    pstSoftWareParam->au32PriorBoxWidth[2] = 10;
    pstSoftWareParam->au32PriorBoxWidth[3] = 5;
    pstSoftWareParam->au32PriorBoxWidth[4] = 3;
    pstSoftWareParam->au32PriorBoxWidth[5] = 1;

    pstSoftWareParam->au32PriorBoxHeight[0] = 38;
    pstSoftWareParam->au32PriorBoxHeight[1] = 19;
    pstSoftWareParam->au32PriorBoxHeight[2] = 10;
    pstSoftWareParam->au32PriorBoxHeight[3] = 5;
    pstSoftWareParam->au32PriorBoxHeight[4] = 3;
    pstSoftWareParam->au32PriorBoxHeight[5] = 1;

    pstSoftWareParam->u32OriImHeight = pstNnieParam->astSegData[0].astSrc[0].unShape.stWhc.u32Height;
    pstSoftWareParam->u32OriImWidth = pstNnieParam->astSegData[0].astSrc[0].unShape.stWhc.u32Width;

    pstSoftWareParam->af32PriorBoxMinSize[0][0] = 30.0f;
    pstSoftWareParam->af32PriorBoxMinSize[1][0] = 60.0f;
    pstSoftWareParam->af32PriorBoxMinSize[2][0] = 111.0f;
    pstSoftWareParam->af32PriorBoxMinSize[3][0] = 162.0f;
    pstSoftWareParam->af32PriorBoxMinSize[4][0] = 213.0f;
    pstSoftWareParam->af32PriorBoxMinSize[5][0] = 264.0f;

    pstSoftWareParam->af32PriorBoxMaxSize[0][0] = 60.0f;
    pstSoftWareParam->af32PriorBoxMaxSize[1][0] = 111.0f;
    pstSoftWareParam->af32PriorBoxMaxSize[2][0] = 162.0f;
    pstSoftWareParam->af32PriorBoxMaxSize[3][0] = 213.0f;
    pstSoftWareParam->af32PriorBoxMaxSize[4][0] = 264.0f;
    pstSoftWareParam->af32PriorBoxMaxSize[5][0] = 315.0f;

    pstSoftWareParam->u32MinSizeNum = 1;
    pstSoftWareParam->u32MaxSizeNum = 1;
    pstSoftWareParam->bFlip= HI_TRUE;
    pstSoftWareParam->bClip= HI_FALSE;

    pstSoftWareParam->au32InputAspectRatioNum[0] = 1;
    pstSoftWareParam->au32InputAspectRatioNum[1] = 2;
    pstSoftWareParam->au32InputAspectRatioNum[2] = 2;
    pstSoftWareParam->au32InputAspectRatioNum[3] = 2;
    pstSoftWareParam->au32InputAspectRatioNum[4] = 1;
    pstSoftWareParam->au32InputAspectRatioNum[5] = 1;

    pstSoftWareParam->af32PriorBoxAspectRatio[0][0] = 2;
    pstSoftWareParam->af32PriorBoxAspectRatio[0][1] = 0;
    pstSoftWareParam->af32PriorBoxAspectRatio[1][0] = 2;
    pstSoftWareParam->af32PriorBoxAspectRatio[1][1] = 3;
    pstSoftWareParam->af32PriorBoxAspectRatio[2][0] = 2;
    pstSoftWareParam->af32PriorBoxAspectRatio[2][1] = 3;
    pstSoftWareParam->af32PriorBoxAspectRatio[3][0] = 2;
    pstSoftWareParam->af32PriorBoxAspectRatio[3][1] = 3;
    pstSoftWareParam->af32PriorBoxAspectRatio[4][0] = 2;
    pstSoftWareParam->af32PriorBoxAspectRatio[4][1] = 0;
    pstSoftWareParam->af32PriorBoxAspectRatio[5][0] = 2;
    pstSoftWareParam->af32PriorBoxAspectRatio[5][1] = 0;

    pstSoftWareParam->af32PriorBoxStepWidth[0] = 8;
    pstSoftWareParam->af32PriorBoxStepWidth[1] = 16;
    pstSoftWareParam->af32PriorBoxStepWidth[2] = 32;
    pstSoftWareParam->af32PriorBoxStepWidth[3] = 64;
    pstSoftWareParam->af32PriorBoxStepWidth[4] = 100;
    pstSoftWareParam->af32PriorBoxStepWidth[5] = 300;

    pstSoftWareParam->af32PriorBoxStepHeight[0] = 8;
    pstSoftWareParam->af32PriorBoxStepHeight[1] = 16;
    pstSoftWareParam->af32PriorBoxStepHeight[2] = 32;
    pstSoftWareParam->af32PriorBoxStepHeight[3] = 64;
    pstSoftWareParam->af32PriorBoxStepHeight[4] = 100;
    pstSoftWareParam->af32PriorBoxStepHeight[5] = 300;

    pstSoftWareParam->f32Offset = 0.5f;

    pstSoftWareParam->as32PriorBoxVar[0] = (HI_S32)(0.1f*SAMPLE_SVP_NNIE_QUANT_BASE);
    pstSoftWareParam->as32PriorBoxVar[1] = (HI_S32)(0.1f*SAMPLE_SVP_NNIE_QUANT_BASE);
    pstSoftWareParam->as32PriorBoxVar[2] = (HI_S32)(0.2f*SAMPLE_SVP_NNIE_QUANT_BASE);
    pstSoftWareParam->as32PriorBoxVar[3] = (HI_S32)(0.2f*SAMPLE_SVP_NNIE_QUANT_BASE);

    /*Set Softmax Parameters*/
    pstSoftWareParam->u32SoftMaxInHeight = 21;
    pstSoftWareParam->au32SoftMaxInChn[0] = 121296;
    pstSoftWareParam->au32SoftMaxInChn[1] = 45486;
    pstSoftWareParam->au32SoftMaxInChn[2] = 12600;
    pstSoftWareParam->au32SoftMaxInChn[3] = 3150;
    pstSoftWareParam->au32SoftMaxInChn[4] = 756;
    pstSoftWareParam->au32SoftMaxInChn[5] = 84;

    pstSoftWareParam->u32ConcatNum = 6;
    pstSoftWareParam->u32SoftMaxOutWidth = 1;
    pstSoftWareParam->u32SoftMaxOutHeight = 21;
    pstSoftWareParam->u32SoftMaxOutChn = 8732;

    /*Set DetectionOut Parameters*/
    pstSoftWareParam->u32ClassNum = 21;
    pstSoftWareParam->u32TopK = 400;
    pstSoftWareParam->u32KeepTopK = 200;
    pstSoftWareParam->u32NmsThresh = (HI_U16)(0.3f*SAMPLE_SVP_NNIE_QUANT_BASE);
    pstSoftWareParam->u32ConfThresh = 1;
    pstSoftWareParam->au32DetectInputChn[0] = 23104;
    pstSoftWareParam->au32DetectInputChn[1] = 8664;
    pstSoftWareParam->au32DetectInputChn[2] = 2400;
    pstSoftWareParam->au32DetectInputChn[3] = 600;
    pstSoftWareParam->au32DetectInputChn[4] = 144;
    pstSoftWareParam->au32DetectInputChn[5] = 16;

    /*Malloc assist buffer memory*/
    u32ClassNum = pstSoftWareParam->u32ClassNum;
    u32TotalSize = SAMPLE_SVP_NNIE_Ssd_GetResultTmpBuf(pstNnieParam,pstSoftWareParam);
    u32DstRoiSize = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum*pstCfg->u32MaxRoiNum*sizeof(HI_U32)*SAMPLE_SVP_NNIE_COORDI_NUM);
    u32DstScoreSize = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum*pstCfg->u32MaxRoiNum*sizeof(HI_U32));
    u32ClassRoiNumSize = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum*sizeof(HI_U32));
    u32TotalSize = u32TotalSize+u32DstRoiSize+u32DstScoreSize+u32ClassRoiNumSize;
    s32Ret = SAMPLE_COMM_SVP_MallocCached("SAMPLE_SSD_INIT",NULL,(HI_U64*)&u64PhyAddr,
                                          (void**)&pu8VirAddr,u32TotalSize);
    SAMPLE_SVP_CHECK_EXPR_RET(HI_SUCCESS != s32Ret,s32Ret,SAMPLE_SVP_ERR_LEVEL_ERROR,
                              "Error,Malloc memory failed!\n");
    memset(pu8VirAddr,0, u32TotalSize);
    SAMPLE_COMM_SVP_FlushCache(u64PhyAddr,(void*)pu8VirAddr,u32TotalSize);

    /*set each tmp buffer addr*/
    pstSoftWareParam->stPriorBoxTmpBuf.u64PhyAddr = u64PhyAddr;
    pstSoftWareParam->stPriorBoxTmpBuf.u64VirAddr = (HI_U64)(pu8VirAddr);

    pstSoftWareParam->stSoftMaxTmpBuf.u64PhyAddr = u64PhyAddr+
                                                   pstSoftWareParam->stPriorBoxTmpBuf.u32Size;
    pstSoftWareParam->stSoftMaxTmpBuf.u64VirAddr = (HI_U64)(pu8VirAddr+
                                                            pstSoftWareParam->stPriorBoxTmpBuf.u32Size);

    pstSoftWareParam->stGetResultTmpBuf.u64PhyAddr = u64PhyAddr+
                                                     pstSoftWareParam->stPriorBoxTmpBuf.u32Size+pstSoftWareParam->stSoftMaxTmpBuf.u32Size;
    pstSoftWareParam->stGetResultTmpBuf.u64VirAddr = (HI_U64)(pu8VirAddr+
                                                              pstSoftWareParam->stPriorBoxTmpBuf.u32Size+ pstSoftWareParam->stSoftMaxTmpBuf.u32Size);

    u32TmpBufTotalSize = pstSoftWareParam->stPriorBoxTmpBuf.u32Size+
                         pstSoftWareParam->stSoftMaxTmpBuf.u32Size + pstSoftWareParam->stGetResultTmpBuf.u32Size;

    /*set result blob*/
    pstSoftWareParam->stDstRoi.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stDstRoi.u64PhyAddr = u64PhyAddr+u32TmpBufTotalSize;
    pstSoftWareParam->stDstRoi.u64VirAddr = (HI_U64)(pu8VirAddr+u32TmpBufTotalSize);
    pstSoftWareParam->stDstRoi.u32Stride = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum*
                                                                           pstSoftWareParam->u32TopK*sizeof(HI_U32)*SAMPLE_SVP_NNIE_COORDI_NUM);
    pstSoftWareParam->stDstRoi.u32Num = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stDstRoi.unShape.stWhc.u32Width = u32ClassNum*
                                                        pstSoftWareParam->u32TopK*SAMPLE_SVP_NNIE_COORDI_NUM;

    pstSoftWareParam->stDstScore.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stDstScore.u64PhyAddr = u64PhyAddr+u32TmpBufTotalSize+u32DstRoiSize;
    pstSoftWareParam->stDstScore.u64VirAddr = (HI_U64)(pu8VirAddr+u32TmpBufTotalSize+u32DstRoiSize);
    pstSoftWareParam->stDstScore.u32Stride = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum*
                                                                             pstSoftWareParam->u32TopK*sizeof(HI_U32));
    pstSoftWareParam->stDstScore.u32Num = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stDstScore.unShape.stWhc.u32Width = u32ClassNum*
                                                          pstSoftWareParam->u32TopK;

    pstSoftWareParam->stClassRoiNum.enType = SVP_BLOB_TYPE_S32;
    pstSoftWareParam->stClassRoiNum.u64PhyAddr = u64PhyAddr+u32TmpBufTotalSize+
                                                 u32DstRoiSize+u32DstScoreSize;
    pstSoftWareParam->stClassRoiNum.u64VirAddr = (HI_U64)(pu8VirAddr+u32TmpBufTotalSize+
                                                          u32DstRoiSize+u32DstScoreSize);
    pstSoftWareParam->stClassRoiNum.u32Stride = SAMPLE_SVP_NNIE_ALIGN16(u32ClassNum*sizeof(HI_U32));
    pstSoftWareParam->stClassRoiNum.u32Num = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Chn = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Height = 1;
    pstSoftWareParam->stClassRoiNum.unShape.stWhc.u32Width = u32ClassNum;

    return s32Ret;
}


/******************************************************************************
* function : Ssd init
******************************************************************************/
static HI_S32 SAMPLE_SVP_NNIE_Ssd_ParamInit(SAMPLE_SVP_NNIE_CFG_S* pstCfg,
                                            SAMPLE_SVP_NNIE_PARAM_S *pstNnieParam, SAMPLE_SVP_NNIE_SSD_SOFTWARE_PARAM_S* pstSoftWareParam)
{
    HI_S32 s32Ret = HI_SUCCESS;
    /*init hardware para*/
    s32Ret = SAMPLE_COMM_SVP_NNIE_ParamInit(pstCfg,pstNnieParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(HI_SUCCESS != s32Ret,INIT_FAIL_0,SAMPLE_SVP_ERR_LEVEL_ERROR,
                               "Error(%#x),SAMPLE_COMM_SVP_NNIE_ParamInit failed!\n",s32Ret);

    /*init software para*/
    s32Ret = SAMPLE_SVP_NNIE_Ssd_SoftwareInit(pstCfg,pstNnieParam,
                                              pstSoftWareParam);
    SAMPLE_SVP_CHECK_EXPR_GOTO(HI_SUCCESS != s32Ret,INIT_FAIL_0,SAMPLE_SVP_ERR_LEVEL_ERROR,
                               "Error(%#x),SAMPLE_SVP_NNIE_Ssd_SoftwareInit failed!\n",s32Ret);

    return s32Ret;
    INIT_FAIL_0:
    s32Ret = SAMPLE_SVP_NNIE_Ssd_Deinit(pstNnieParam,pstSoftWareParam,NULL);
    SAMPLE_SVP_CHECK_EXPR_RET(HI_SUCCESS != s32Ret,s32Ret,SAMPLE_SVP_ERR_LEVEL_ERROR,
                              "Error(%#x),SAMPLE_SVP_NNIE_Ssd_Deinit failed!\n",s32Ret);
    return HI_FAILURE;

}

HI_VOID SAMPLE_VDEC_Usage(char *sPrgNm)
{
    printf("\n/************************************/\n");
    printf("Usage : %s <index> \n", sPrgNm);
    printf("index:\n");
    printf("\t0:  RTSP-VDEC(H264)-VPSS\n");

    printf("/************************************/\n\n");
}

VO_INTF_SYNC_E g_enIntfSync = VO_OUTPUT_1080P30;

HI_S32 SAMPLE_RTSP_VDEC_VPSS(HI_VOID) {
    VB_CONFIG_S stVbConfig;
    HI_S32 k, s32Ret = HI_SUCCESS;
    VDEC_THREAD_PARAM_S stVdecSend[VDEC_MAX_CHN_NUM];
    SIZE_S stDispSize;
    HI_U32 i, u32VdecChnNum;
    VPSS_GRP VpssGrp;
    pthread_t VdecThread[2 * VDEC_MAX_CHN_NUM];
    PIC_SIZE_E enDispPicSize;
    SAMPLE_VDEC_ATTR astSampleVdec[VDEC_MAX_CHN_NUM];
    VPSS_CHN_ATTR_S astVpssChnAttr[VPSS_MAX_CHN_NUM];
    VPSS_GRP_ATTR_S stVpssGrpAttr;
    HI_BOOL abChnEnable[VPSS_MAX_CHN_NUM];

    u32VdecChnNum = 1;
    fd_set read_fds;
    HI_S32 VdecFd[VDEC_MAX_CHN_NUM];

    HI_S32 maxfd = 0;
    struct timeval TimeoutVal;
    HI_U32 u32Size;

    SAMPLE_SVP_NNIE_MODEL_S s_stSsdModel = {0};
    SAMPLE_SVP_NNIE_PARAM_S s_stSsdNnieParam = {0};
    SAMPLE_SVP_NNIE_CFG_S   stNnieCfg = {0};
    SAMPLE_SVP_NNIE_SSD_SOFTWARE_PARAM_S s_stSsdSoftwareParam = {0};
    HI_CHAR *pcModelName = "./source_file/inst_FaceDetect_inst.wk";

    //HI_CHAR szFilename[32];

    /************************************************
    step1:  init SYS, init common VB(for VPSS and VO)
    *************************************************/
    if (VO_OUTPUT_1080P30 == g_enIntfSync) {
        enDispPicSize = PIC_1080P;
    } else {
        enDispPicSize = PIC_1080P;
    }

    s32Ret = SAMPLE_COMM_SYS_GetPicSize(enDispPicSize, &stDispSize);

    if (s32Ret != HI_SUCCESS)
    {
        SAMPLE_PRT("sys get pic size fail for %#x!\n", s32Ret);
        goto END1;
    }

    memset(&stVbConfig, 0, sizeof(VB_CONFIG_S));
    stVbConfig.u32MaxPoolCnt = 1;
    stVbConfig.astCommPool[0].u32BlkCnt = 10 * u32VdecChnNum;
    stVbConfig.astCommPool[0].u64BlkSize = COMMON_GetPicBufferSize(stDispSize.u32Width, stDispSize.u32Height,
                                                                   PIXEL_FORMAT_YVU_SEMIPLANAR_420, DATA_BITWIDTH_8,
                                                                   COMPRESS_MODE_SEG, 0);
    s32Ret = SAMPLE_COMM_SYS_Init(&stVbConfig);
    if (s32Ret != HI_SUCCESS)
    {
        SAMPLE_PRT("init sys fail for %#x!\n", s32Ret);
        goto END1;
    }

    /************************************************
    step2:  init module VB or user VB(for VDEC)
    *************************************************/
    for (i = 0; i < u32VdecChnNum; i++)
    {
        astSampleVdec[i].enType = PT_H264;
        astSampleVdec[i].u32Width = INPUT_VIDEO_WIDTH;
        astSampleVdec[i].u32Height = INPUT_VIDEO_HEIGHT;
        astSampleVdec[i].enMode = VIDEO_MODE_FRAME;
        astSampleVdec[i].stSapmleVdecVideo.enDecMode = VIDEO_DEC_MODE_IPB;
        astSampleVdec[i].stSapmleVdecVideo.enBitWidth = DATA_BITWIDTH_8;
        astSampleVdec[i].stSapmleVdecVideo.u32RefFrameNum = 3;
        astSampleVdec[i].u32DisplayFrameNum = 2;
        astSampleVdec[i].u32FrameBufCnt =
                astSampleVdec[i].stSapmleVdecVideo.u32RefFrameNum + astSampleVdec[i].u32DisplayFrameNum + 1;
    }
    s32Ret = SAMPLE_COMM_VDEC_InitVBPool(u32VdecChnNum, &astSampleVdec[0]);
    if (s32Ret != HI_SUCCESS)
    {
        SAMPLE_PRT("init mod common vb fail for %#x!\n", s32Ret);
        goto END2;
    }

    /************************************************
    step3:  start VDEC
    *************************************************/
    s32Ret = SAMPLE_COMM_VDEC_Start(u32VdecChnNum, &astSampleVdec[0]);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("start VDEC fail for %#x!\n", s32Ret);
        goto END3;
    }

    /************************************************
    step4:  start VPSS
    *************************************************/
    stVpssGrpAttr.u32MaxW = VIDEO_WIDTH;
    stVpssGrpAttr.u32MaxH = VIDEO_HEIGHT;
    stVpssGrpAttr.stFrameRate.s32SrcFrameRate = -1;
    stVpssGrpAttr.stFrameRate.s32DstFrameRate = -1;
    stVpssGrpAttr.enDynamicRange = DYNAMIC_RANGE_SDR8;
    stVpssGrpAttr.enPixelFormat = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    stVpssGrpAttr.bNrEn = HI_FALSE;

    memset(abChnEnable, 0, sizeof(abChnEnable));
    abChnEnable[0] = HI_TRUE;
    astVpssChnAttr[0].u32Width = stDispSize.u32Width;
    astVpssChnAttr[0].u32Height = stDispSize.u32Height;
    astVpssChnAttr[0].enChnMode = VPSS_CHN_MODE_USER;
    astVpssChnAttr[0].enCompressMode = COMPRESS_MODE_NONE;
    astVpssChnAttr[0].enDynamicRange = DYNAMIC_RANGE_SDR8;
    astVpssChnAttr[0].enPixelFormat = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    astVpssChnAttr[0].stFrameRate.s32SrcFrameRate = -1;
    astVpssChnAttr[0].stFrameRate.s32DstFrameRate = -1;
    astVpssChnAttr[0].u32Depth = 1;
    astVpssChnAttr[0].bMirror = HI_FALSE;
    astVpssChnAttr[0].bFlip = HI_FALSE;
    astVpssChnAttr[0].stAspectRatio.enMode = ASPECT_RATIO_NONE;
    astVpssChnAttr[0].enVideoFormat = VIDEO_FORMAT_LINEAR;

    for (i = 0; i < u32VdecChnNum; i++)
    {
        VpssGrp = i;
        s32Ret = SAMPLE_COMM_VPSS_Start(VpssGrp, &abChnEnable[0], &stVpssGrpAttr, &astVpssChnAttr[0]);
        if (s32Ret != HI_SUCCESS) {
            SAMPLE_PRT("start VPSS fail for %#x!\n", s32Ret);
            goto END4;
        }
    }

    /************************************************
    step6:  start VENC
    *************************************************/
    VENC_GOP_ATTR_S stGopAttr;
    VENC_CHN VeH264Chn = 0;
    PAYLOAD_TYPE_E enStreamType = PT_H264;
    PIC_SIZE_E aenSize[VPSS_CHN_NUM];
    aenSize[0] = PIC_1080P;
    SAMPLE_RC_E enRcMode = SAMPLE_RC_CBR;

    s32Ret = SAMPLE_COMM_VENC_GetGopAttr(VENC_GOPMODE_NORMALP, &stGopAttr);
    SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS != s32Ret, END5,
                           "Error(%#x),SAMPLE_COMM_VENC_GetGopAttr failed!\n",s32Ret);
    s32Ret = SAMPLE_COMM_VENC_Start(VeH264Chn, enStreamType,aenSize[0],enRcMode,0,&stGopAttr);
    SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS != s32Ret, END5,
                           "Error(%#x),SAMPLE_COMM_VENC_Start failed!\n",s32Ret);
    s32Ret = SAMPLE_COMM_VENC_StartGetStream(&VeH264Chn, 1);
    SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS != s32Ret, END5,
                           "Error(%#x),SAMPLE_COMM_VENC_StartGetStream failed!\n",s32Ret);

    /************************************************
    step7:  VDEC bind VPSS
    *************************************************/
    for (i = 0; i < u32VdecChnNum; i++) {
        s32Ret = SAMPLE_COMM_VDEC_Bind_VPSS(i, i);
        if (s32Ret != HI_SUCCESS) {
            SAMPLE_PRT("vdec bind vpss fail for %#x!\n", s32Ret);
            goto END5;
        }
    }

    /************************************************
    step8:  get fd from VPSS
    *************************************************/

    for (i = 0; i < u32VdecChnNum; i++) {
        VdecFd[i] = HI_MPI_VPSS_GetChnFd(VpssGrp, i);
        if (VdecFd[i] < 0) {
            SAMPLE_PRT("HI_MPI_VENC_GetFd failed with %#x!\n",
                       VdecFd[i]);
            goto END5;
        }
        if (maxfd <= VdecFd[i])
        {
            maxfd = VdecFd[i];
        }
    }

    /************************************************
    step9:  start NNIE
    *************************************************/

    s32Ret = SAMPLE_COMM_SVP_NNIE_LoadModel(pcModelName,&s_stSsdModel);
    if(s32Ret != HI_SUCCESS)
    {
        SAMPLE_PRT("NNIE load model fail for %#x!\n", s32Ret);
    }

    stNnieCfg.u32MaxInputNum = 1; //max input image num in each batch
    stNnieCfg.u32MaxRoiNum = 0;
    stNnieCfg.aenNnieCoreId[0] = SVP_NNIE_ID_0;//set NNIE core


    s_stSsdNnieParam.pstModel = &s_stSsdModel.stModel;
    s32Ret = SAMPLE_SVP_NNIE_Ssd_ParamInit(&stNnieCfg,&s_stSsdNnieParam,&s_stSsdSoftwareParam);
    if(s32Ret != HI_SUCCESS)
    {
        SAMPLE_PRT("NNIE param init fail for %#x!\n", s32Ret);
    }

    /************************************************
    step10:  send stream to VDEC
    *************************************************/
    for (i = 0; i < u32VdecChnNum; i++) {
        snprintf(stVdecSend[i].cFileName, sizeof(stVdecSend[i].cFileName), "rtsp://192.168.2.148:8554/live/t1060");
        stVdecSend[i].enType = astSampleVdec[i].enType;
        stVdecSend[i].s32StreamMode = astSampleVdec[i].enMode;
        stVdecSend[i].s32ChnId = i;
        stVdecSend[i].s32IntervalTime = 1000;
        stVdecSend[i].u64PtsInit = 0;
        stVdecSend[i].u64PtsIncrease = 0;
        stVdecSend[i].eThreadCtrl = THREAD_CTRL_START;
        stVdecSend[i].bCircleSend = HI_TRUE;
        stVdecSend[i].s32MilliSec = 0;
        stVdecSend[i].s32MinBufSize = (astSampleVdec[i].u32Width * astSampleVdec[i].u32Height * 3) >> 1;
    }
    SAMPLE_COMM_VDEC_StartSendStream(u32VdecChnNum, &stVdecSend[0], &VdecThread[0]);


    while (1)
    {
        FD_ZERO(&read_fds);
        for (i = 0; i < u32VdecChnNum; i++)
        {
            FD_SET(VdecFd[i], &read_fds);
        }

        TimeoutVal.tv_sec  = 2;
        TimeoutVal.tv_usec = 0;
        s32Ret = select(maxfd + 1, &read_fds, NULL, NULL, &TimeoutVal);

        if (s32Ret < 0)
        {
            SAMPLE_PRT("select failed!\n");
            break;
        }
        else if (s32Ret == 0)
        {
            SAMPLE_PRT("get venc stream time out, exit thread\n");
            continue;
        }
        for (i = 0; i < u32VdecChnNum; i++)
        {
            if (FD_ISSET(VdecFd[i], &read_fds))
            {
#ifdef SAVE
                FILE * pFile = fopen("123.yuv", "wb");
                FILE * pFile_bgr = fopen("123.bgr_planar_384x216", "wb");
                FILE * pFile_bgr_1080p = fopen("123.bgr_planar_1920x1080", "wb");
#endif
                VIDEO_FRAME_INFO_S *video_frame_info_s = new VIDEO_FRAME_INFO_S;
                memset(video_frame_info_s, 0, sizeof(VIDEO_FRAME_INFO_S));

                s32Ret = HI_MPI_VPSS_GetChnFrame(VpssGrp, 0, video_frame_info_s, 0);
                //printf("channel %d vdec get frame return = %x \n",i, s32Ret);
                if(s32Ret == HI_SUCCESS)
                {
                    struct timeval start;
                    struct timeval end;
                    gettimeofday(&start, NULL);
                    //u32Size = (video_frame_info_s->stVFrame.u32Stride[0]) * (video_frame_info_s->stVFrame.u32Height) * 3 / 2;
                    //video_frame_info_s->stVFrame.u64VirAddr[0] = (HI_U64)HI_MPI_SYS_Mmap(video_frame_info_s->stVFrame.u64PhyAddr[0], u32Size);
/*
                    printf("physical addr = %llx \n", video_frame_info_s->stVFrame.u64PhyAddr[0]);
                    printf("virtual addr = %llx \n", video_frame_info_s->stVFrame.u64VirAddr[0]);
                    printf("height = %d \n", video_frame_info_s->stVFrame.u32Height);
                    printf("width = %d \n", video_frame_info_s->stVFrame.u32Width);
                    printf("pixle format = %d \n", video_frame_info_s->stVFrame.enPixelFormat);
                    printf("stride 0 = %d \n", video_frame_info_s->stVFrame.u32Stride[0]);
                    printf("stride 1 = %d \n", video_frame_info_s->stVFrame.u32Stride[1]);
*/
                    IVE_IMAGE_S *pstImg = new IVE_IMAGE_S;

                    pstImg->enType = IVE_IMAGE_TYPE_U8C3_PLANAR;
                    pstImg->u32Width = video_frame_info_s->stVFrame.u32Width;
                    pstImg->u32Height = video_frame_info_s->stVFrame.u32Height;
                    pstImg->au32Stride[0] = pstImg->u32Width;

                    u32Size = pstImg->au32Stride[0] * pstImg->u32Height *3;

                    s32Ret = HI_MPI_SYS_MmzAlloc(&pstImg->au64PhyAddr[0], (HI_VOID**)&pstImg->au64VirAddr[0], NULL, HI_NULL, u32Size);
                    if (s32Ret != HI_SUCCESS)
                    {
                        SAMPLE_PRT("Mmz Alloc fail,Error(%#x)\n", s32Ret);
                        return s32Ret;
                    }
                    pstImg->au32Stride[1] = pstImg->u32Width;
                    pstImg->au32Stride[2] = pstImg->u32Width;
                    pstImg->au64PhyAddr[1] = pstImg->au64PhyAddr[0] + pstImg->u32Width * pstImg->u32Height;
                    pstImg->au64VirAddr[1] = pstImg->au64VirAddr[0] + pstImg->u32Width * pstImg->u32Height;
                    pstImg->au64PhyAddr[2] = pstImg->au64PhyAddr[0] + pstImg->u32Width * pstImg->u32Height *2;
                    pstImg->au64VirAddr[2] = pstImg->au64VirAddr[0] + pstImg->u32Width * pstImg->u32Height *2;

                    s32Ret = DGSP420ToBGR24Planar(&(video_frame_info_s->stVFrame), pstImg);
#ifdef SAVE
                    HI_U32 u32WidthInBytes = pstImg->u32Width*3;
                    HI_U32 u32Stride = ALIGN_UP(u32WidthInBytes, 16);

                    HI_U8 * pUserPageAddr = (HI_U8*) pstImg->au64VirAddr[0];
                    //printf("color space transform s32Ret=0x%x \n", s32Ret);
                    if (HI_NULL != pUserPageAddr)
                    {
                        //printf("%s %d:HI_MPI_SYS_Mmap fail!!! u32Size=%d\n",__func__, __LINE__,u32Size);
                        fprintf(stderr, "saving......RGB..%d x %d......", pstImg->u32Width, pstImg->u32Height);
                        fflush(stderr);

                        HI_U8 *pTmp = pUserPageAddr;
                        for (i = 0; i < pstImg->u32Height; i++, pTmp += u32Stride)
                        {
                            fwrite(pTmp, u32WidthInBytes, 1, pFile_bgr_1080p);
                        }
                        fflush(pFile_bgr_1080p);

                        fprintf(stderr, "done!\n");
                        fflush(stderr);
                    }
#endif
                    IVE_IMAGE_S *pstDstImg = new IVE_IMAGE_S;

                    pstDstImg->enType = IVE_IMAGE_TYPE_U8C3_PLANAR;
                    pstDstImg->u32Width = 384;
                    pstDstImg->u32Height = 216;
                    pstDstImg->au32Stride[0] = IVE_CalcStride(pstDstImg->u32Width, IVE_ALIGN);

                    u32Size = pstDstImg->au32Stride[0] * pstDstImg->u32Height * 3;
                    s32Ret = HI_MPI_SYS_MmzAlloc(&pstDstImg->au64PhyAddr[0], (HI_VOID**)&pstDstImg->au64VirAddr[0], NULL, HI_NULL, u32Size);
                    if (s32Ret != HI_SUCCESS)
                    {
                        SAMPLE_PRT("Mmz Alloc fail,Error(%#x)\n", s32Ret);
                        return s32Ret;
                    }
                    pstDstImg->au32Stride[1] = pstDstImg->au32Stride[0];
                    pstDstImg->au32Stride[2] = pstDstImg->au32Stride[0];
                    pstDstImg->au64PhyAddr[1] = pstDstImg->au64PhyAddr[0] + pstDstImg->au32Stride[0] * pstDstImg->u32Height;
                    pstDstImg->au64VirAddr[1] = pstDstImg->au64VirAddr[0] + pstDstImg->au32Stride[0] * pstDstImg->u32Height;
                    pstDstImg->au64PhyAddr[2] = pstDstImg->au64PhyAddr[0] + pstDstImg->au32Stride[0] * pstDstImg->u32Height *2;
                    pstDstImg->au64VirAddr[2] = pstDstImg->au64VirAddr[0] + pstDstImg->au32Stride[0] * pstDstImg->u32Height *2;

                    s32Ret = DGIveResize(pstImg, pstDstImg);

                    if (s32Ret != HI_SUCCESS)
                    {
                        SAMPLE_PRT("Resize fail,Error(%#x)\n", s32Ret);
                        return s32Ret;
                    }
#ifdef SAVE
                    u32WidthInBytes = pstDstImg->u32Width*3;
                    u32Stride = ALIGN_UP(u32WidthInBytes, 16);

                    pUserPageAddr = (HI_U8*) pstDstImg->au64VirAddr[0];

                    if (HI_NULL != pUserPageAddr)
                    {
                        //printf("%s %d:HI_MPI_SYS_Mmap fail!!! u32Size=%d\n",__func__, __LINE__,u32Size);
                        fprintf(stderr, "saving......RGB..%d x %d......", pstDstImg->u32Width, pstDstImg->u32Height);
                        fflush(stderr);

                        HI_U8 *pTmp = pUserPageAddr;
                        for (i = 0; i < pstDstImg->u32Height; i++, pTmp += u32Stride)
                        {
                            fwrite(pTmp, u32WidthInBytes, 1, pFile_bgr) ;
                        }
                        fflush(pFile_bgr);

                        fprintf(stderr, "done!\n");
                        fflush(stderr);
                    }
#endif
                    s_stSsdNnieParam.astSegData[0].astSrc[0].u64VirAddr = pstDstImg->au64VirAddr[0];
                    s_stSsdNnieParam.astSegData[0].astSrc[0].u64PhyAddr = pstDstImg->au64PhyAddr[0];
                    s_stSsdNnieParam.astSegData[0].astSrc[0].u32Stride = pstDstImg->au32Stride[0];

                    HI_BOOL bFinish = HI_FALSE;
                    SVP_NNIE_HANDLE hSvpNnieHandle = 0;
                    s32Ret = HI_MPI_SVP_NNIE_Forward(&hSvpNnieHandle, s_stSsdNnieParam.astSegData[0].astSrc, s_stSsdNnieParam.pstModel,
                                            s_stSsdNnieParam.astSegData[0].astDst,
                                            &s_stSsdNnieParam.astForwardCtrl[0], HI_TRUE);
                    if (s32Ret != HI_SUCCESS)
                    {
                        SAMPLE_PRT("Forward fail,Error(%#x)\n", s32Ret);
                        return s32Ret;
                    }

                    if(HI_TRUE)
                    {
                        /*Wait NNIE finish*/
                        while(HI_ERR_SVP_NNIE_QUERY_TIMEOUT == (s32Ret = HI_MPI_SVP_NNIE_Query(s_stSsdNnieParam.astForwardCtrl[0].enNnieId,
                                                                                               hSvpNnieHandle, &bFinish, HI_TRUE)))
                        {
                            usleep(100);
                            SAMPLE_SVP_TRACE(SAMPLE_SVP_ERR_LEVEL_INFO,
                                             "HI_MPI_SVP_NNIE_Query Query timeout!\n");
                        }
                    }

                    bFinish = HI_FALSE;

                    for(i = 0; i < s_stSsdNnieParam.astForwardCtrl[0].u32DstNum; i++)
                    {
                        SAMPLE_COMM_SVP_FlushCache(s_stSsdNnieParam.astSegData[0].astDst[i].u64PhyAddr,
                                                   (HI_VOID *) s_stSsdNnieParam.astSegData[0].astDst[i].u64VirAddr,
                                                   s_stSsdNnieParam.astSegData[0].astDst[i].u32Num*
                                                           s_stSsdNnieParam.astSegData[0].astDst[i].unShape.stWhc.u32Chn*
                                                           s_stSsdNnieParam.astSegData[0].astDst[i].unShape.stWhc.u32Height*
                                                           s_stSsdNnieParam.astSegData[0].astDst[i].u32Stride);
                    }

                    (HI_VOID)HI_MPI_SYS_MmzFree(pstImg->au64PhyAddr[0], (void *) (pstImg->au64VirAddr[0]));

                    (HI_VOID)HI_MPI_SYS_MmzFree(pstDstImg->au64PhyAddr[0], (void *) (pstDstImg->au64VirAddr[0]));

                    delete pstImg;
                    pstImg = nullptr;
                    delete pstDstImg;
                    pstDstImg = nullptr;

                    HI_S32 s32Ret = HI_SUCCESS;

                    std::vector<int>min_sizes = {10, 15, 20, 25, 35, 40};
                    std::vector<float>aspect_ratio_vec = {1.0, 1.25};
                    std::vector<float>variance_list = {0.1, 0.1, 0.2, 0.2};
                    float offset = 0.5;
                    int step = 8;
                    float nms_thres = 0.35;

                    SVP_BLOB_S conv_loc_ = s_stSsdNnieParam.astSegData[0].astDst[0];
                    SVP_BLOB_S conv_conf_ = s_stSsdNnieParam.astSegData[0].astDst[1];

                    HI_S32* cls_cpu = (HI_S32 *)conv_conf_.u64VirAddr;
                    HI_S32* reg_cpu = (HI_S32 *)conv_loc_.u64VirAddr;

                    int cls_height = conv_conf_.unShape.stWhc.u32Height;
                    int cls_width = conv_conf_.unShape.stWhc.u32Width;
                    int reg_width = conv_loc_.unShape.stWhc.u32Width;
                    int cstep = conv_conf_.u32Stride * conv_conf_.unShape.stWhc.u32Height / 4;

                    int aspect_ratio_num = aspect_ratio_vec.size();
                    int anchor_number = min_sizes.size() * aspect_ratio_vec.size();
                    float conf_thres_=0.4f;
                    std::vector<struct Bbox> vbbox;

                    float log_thres[anchor_number];
                    for (int i = 0; i < anchor_number; i++)
                    {
                        log_thres[i] = log(conf_thres_ / (1.0 - conf_thres_));
                    }
                    float pred_w, pred_h, center_x, center_y, pred_x, pred_y, raw_pred_x1,
                            raw_pred_y1, raw_pred_w, raw_pred_h, prior_center_x, prior_center_y;
                    for (int j = 0; j < anchor_number; j++)
                    {
                        float aspect_ratio = aspect_ratio_vec[j % aspect_ratio_num];
                        float prior_h = min_sizes[j / aspect_ratio_num] * sqrt(aspect_ratio);
                        float prior_w = min_sizes[j / aspect_ratio_num] / sqrt(aspect_ratio);

                        for (int y_index = 0; y_index < cls_height; y_index++)
                        {
                            for (int x_index = 0; x_index < cls_width; x_index++)
                            {
                                float x0 = cls_cpu[2 * j * cstep + y_index * cls_width + x_index] / 4096.0;
                                float x1 = cls_cpu[(2 * j + 1) * cstep + y_index * cls_width + x_index] / 4096.0;
                                if (x1 - x0 > log_thres[j])
                                {
                                    raw_pred_x1 = reg_cpu[j * 4 * cstep + y_index * reg_width + x_index] / 4096.0;
                                    raw_pred_y1 = reg_cpu[(j * 4 + 1) * cstep + y_index * reg_width + x_index] / 4096.0;
                                    raw_pred_w = reg_cpu[(j * 4 + 2) * cstep + y_index * reg_width + x_index] / 4096.0;
                                    raw_pred_h = reg_cpu[(j * 4 + 3) * cstep + y_index * reg_width + x_index] / 4096.0;

                                    prior_center_x = (x_index + offset) * step;
                                    prior_center_y = (y_index + offset) * step;
                                    center_x = variance_list[0] * raw_pred_x1 * prior_w + prior_center_x;
                                    center_y = variance_list[1] * raw_pred_y1 * prior_h + prior_center_y;
                                    pred_w = (exp(variance_list[2] * raw_pred_w) * prior_w);
                                    pred_h = (exp(variance_list[3] * raw_pred_h) * prior_h);
                                    pred_x = (center_x - pred_w / 2.);
                                    pred_y = (center_y - pred_h / 2.);

                                    struct Bbox bbox;
                                    bbox.confidence = 1.0 / (1.0 + exp(x0 - x1));
                                    bbox.rect = cv::Rect_<float>(pred_x, pred_y, pred_w, pred_h);
                                    bbox.deleted = false;
                                    vbbox.push_back(bbox);
                                }
                            }
                        }
                    }

                    if (vbbox.size() != 0)
                    {
                        nms(vbbox, nms_thres);
                    }

                    std::vector<struct Bbox> final_vbbox;

                    int u16Num = 0;
                    SAMPLE_RECT_ARRAY_S sample_rect_array_s;
                    for(int i=0;i<vbbox.size();i++)
                    {
                        if(!vbbox[i].deleted)
                        {

                            //final_vbbox.push_back(vbbox[i]);
                            std::cout << vbbox[i].rect.x << "  " << vbbox[i].rect.y << "  " << vbbox[i].rect.width << "  " << vbbox[i].rect.height << std::endl;
                            sample_rect_array_s.astRect[u16Num].astPoint[0].s32X = (HI_U32)(vbbox[i].rect.x * 5) & (~1);
                            sample_rect_array_s.astRect[u16Num].astPoint[0].s32Y = (HI_U32)(vbbox[i].rect.y * 5) & (~1);

                            sample_rect_array_s.astRect[u16Num].astPoint[1].s32X = (HI_U32)((vbbox[i].rect.x + vbbox[i].rect.width) * 5) & (~1);
                            sample_rect_array_s.astRect[u16Num].astPoint[1].s32Y = (HI_U32)(vbbox[i].rect.y * 5) & (~1);

                            sample_rect_array_s.astRect[u16Num].astPoint[2].s32X = (HI_U32)((vbbox[i].rect.x + vbbox[i].rect.width) * 5) & (~1);
                            sample_rect_array_s.astRect[u16Num].astPoint[2].s32Y = (HI_U32)((vbbox[i].rect.y + vbbox[i].rect.height) * 5) & (~1);

                            sample_rect_array_s.astRect[u16Num].astPoint[3].s32X = (HI_U32)(vbbox[i].rect.x * 5) & (~1);
                            sample_rect_array_s.astRect[u16Num].astPoint[3].s32Y = (HI_U32)((vbbox[i].rect.y + vbbox[i].rect.height) * 5) & (~1);
                            u16Num += 1;
                        }
                    }
                    sample_rect_array_s.u16Num = u16Num;

                    printf("detected face = %d\n", u16Num);

                    SAMPLE_COMM_VGS_FillRect(video_frame_info_s, &sample_rect_array_s, 0x0000FF00);

                    //FILE *pFile = fopen("123.yuv", "wb");
                    //SAMPLE_COMM_VDEC_SaveYUVFile_Linear8Bit(pFile, &(video_frame_info_s->stVFrame));

                    s32Ret = HI_MPI_VENC_SendFrame(VeH264Chn, video_frame_info_s, 0);
                    SAMPLE_CHECK_EXPR_GOTO(HI_SUCCESS!=s32Ret, END5, "HI_MPI_VENC_SendFrame failed, Error(%#x)!\n", s32Ret);


                    //HI_MPI_SYS_Munmap((void *)(video_frame_info_s->stVFrame.u64VirAddr[0]), u32Size);
                    gettimeofday(&end, NULL);
                    float time_use=(end.tv_sec-start.tv_sec)*1000000+(end.tv_usec-start.tv_usec);//微秒
                    //printf("post process time is %f us \n",time_use);
                    //printf("one round\n");
                }
                HI_MPI_VPSS_ReleaseChnFrame(VpssGrp, 0, video_frame_info_s);
                delete video_frame_info_s;
                video_frame_info_s = nullptr;
#ifdef SAVE
                fclose(pFile);
                fclose(pFile_bgr);
                fclose(pFile_bgr_1080p);
#endif
                k++;
            }
        }
    };

    SAMPLE_COMM_VDEC_CmdCtrl(u32VdecChnNum, &stVdecSend[0], &VdecThread[0]);

    SAMPLE_COMM_VDEC_StopSendStream(u32VdecChnNum, &stVdecSend[0], &VdecThread[0]);


END5:
    for(i=0; i<u32VdecChnNum; i++)
    {
        s32Ret = SAMPLE_COMM_VDEC_UnBind_VPSS(i, i);
        if(s32Ret != HI_SUCCESS)
        {
            SAMPLE_PRT("vdec unbind vpss fail for %#x!\n", s32Ret);
        }
    }

END4:
    for(i = VpssGrp; i >= 0; i--)
    {
        VpssGrp = i;
        SAMPLE_COMM_VPSS_Stop(VpssGrp, &abChnEnable[0]);
    }
END3:
    SAMPLE_COMM_VDEC_Stop(u32VdecChnNum);

END2:
    SAMPLE_COMM_VDEC_ExitVBPool();

END1:
    SAMPLE_COMM_SYS_Exit();

    return s32Ret;
}

/******************************************************************************
* function    : main()
* Description : video vdec sample
******************************************************************************/
int main(int argc, char *argv[])
{
    HI_S32 s32Ret = HI_SUCCESS;

    if ((argc < 2) || (1 != strlen(argv[1])))
    {
        printf("\nInvaild input!  For examples:\n");
        SAMPLE_VDEC_Usage(argv[0]);
        return HI_FAILURE;
    }

    if ((argc > 2) && ('1' == *argv[2]))
    {
        g_enIntfSync = VO_OUTPUT_1080P30;
    }
    else
    {
        g_enIntfSync = VO_OUTPUT_3840x2160_30;
    }

    signal(SIGINT, SAMPLE_VDEC_HandleSig);
    signal(SIGTERM, SAMPLE_VDEC_HandleSig);

    /******************************************
     choose the case
    ******************************************/
    switch (*argv[1])
    {
        case '0':
        {
            s32Ret = SAMPLE_RTSP_VDEC_VPSS();
            break;
        }
        default :
        {
            SAMPLE_PRT("the index is invaild!\n");
            SAMPLE_VDEC_Usage(argv[0]);
            s32Ret = HI_FAILURE;
            break;
        }
    }

    if (HI_SUCCESS == s32Ret)
    {
        SAMPLE_PRT("program exit normally!\n");
    }
    else
    {
        SAMPLE_PRT("program exit abnormally!\n");
    }

    return s32Ret;
}