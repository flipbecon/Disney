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
#include "thread"


#include "utils.h"
#include "ive.h"
#include "nnie.h"
#include "sample_comm.h"
#include "sample_comm_svp.h"
#include "sample_comm_nnie.h"
#include "sample_svp_nnie_software.h"


#define IVE_ALIGN 16
#define INPUT_VIDEO_WIDTH  1920
#define INPUT_VIDEO_HEIGHT 1080

using namespace std;
using namespace tbb;

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
    HI_U32 i, channel_index, u32VdecChnNum;
    VPSS_GRP VpssGrp;
    pthread_t VdecThread[2 * VDEC_MAX_CHN_NUM];
    pthread_t IveThread[2 * VDEC_MAX_CHN_NUM];
    pthread_t NnieThread[2 * VDEC_MAX_CHN_NUM];

    SAMPLE_VDEC_ATTR astSampleVdec[VDEC_MAX_CHN_NUM];
    VPSS_CHN_ATTR_S astVpssChnAttr[VPSS_MAX_CHN_NUM];
    VPSS_GRP_ATTR_S stVpssGrpAttr;
    HI_BOOL abChnEnable[VPSS_MAX_CHN_NUM];

    IVE_WORKER_S iveWorkerS;
    NNIE_WORKER_S nnieWorkerS;
    u32VdecChnNum = 1;

    HI_U32 u32IveChnNum = 1;
    HI_U32 u32NnieChnNum = 1;
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

    concurrent_bounded_queue<VIDEO_FRAME_INFO_S *> frameQueue;
    frameQueue.set_capacity(50);

    concurrent_bounded_queue<IVE_IMAGE_S *> imageQueue;
    imageQueue.set_capacity(50);

//    concurrent_bounded_queue<pair<VIDEO_FRAME_INFO_S *, IVE_IMAGE_S *>> pipelineQueue;
//    pipelineQueue.set_capacity(50);

    /************************************************
    step1:  init SYS, init common VB(for VPSS and VO)
    *************************************************/

    stDispSize.u32Height = INPUT_VIDEO_HEIGHT;
    stDispSize.u32Width = INPUT_VIDEO_WIDTH;

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
    s32Ret = SAMPLE_COMM_VDEC_InitVBPool(u32VdecChnNum, astSampleVdec);
    if (s32Ret != HI_SUCCESS)
    {
        SAMPLE_PRT("init mod common vb fail for %#x!\n", s32Ret);
        goto END2;
    }

    /************************************************
    step3:  start VDEC
    *************************************************/
    s32Ret = SAMPLE_COMM_VDEC_Start(u32VdecChnNum, astSampleVdec);
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
    for (i = 0; i < u32VdecChnNum; i++)
    {
        abChnEnable[i] = HI_TRUE;
        astVpssChnAttr[i].u32Width = stDispSize.u32Width;
        astVpssChnAttr[i].u32Height = stDispSize.u32Height;
        astVpssChnAttr[i].enChnMode = VPSS_CHN_MODE_USER;
        astVpssChnAttr[i].enCompressMode = COMPRESS_MODE_NONE;
        astVpssChnAttr[i].enDynamicRange = DYNAMIC_RANGE_SDR8;
        astVpssChnAttr[i].enPixelFormat = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
        astVpssChnAttr[i].stFrameRate.s32SrcFrameRate = -1;
        astVpssChnAttr[i].stFrameRate.s32DstFrameRate = -1;
        astVpssChnAttr[i].u32Depth = 1;
        astVpssChnAttr[i].bMirror = HI_FALSE;
        astVpssChnAttr[i].bFlip = HI_FALSE;
        astVpssChnAttr[i].stAspectRatio.enMode = ASPECT_RATIO_NONE;
        astVpssChnAttr[i].enVideoFormat = VIDEO_FORMAT_LINEAR;
    }
    printf("vpss start entered \n");
    for (i = 0; i < u32VdecChnNum; i++)
    {
        VpssGrp = i;
        s32Ret = SAMPLE_COMM_VPSS_Start(VpssGrp, abChnEnable, &stVpssGrpAttr, astVpssChnAttr);
        if (s32Ret != HI_SUCCESS) {
            SAMPLE_PRT("start VPSS fail for %#x!\n", s32Ret);
            goto END4;
        }
    }
    printf("vpss start end \n");
    /************************************************
    step5:  VDEC bind VPSS
    *************************************************/
    for (i = 0; i < u32VdecChnNum; i++) {
        s32Ret = SAMPLE_COMM_VDEC_Bind_VPSS(i, i);
        if (s32Ret != HI_SUCCESS) {
            SAMPLE_PRT("vdec bind vpss fail for %#x!\n", s32Ret);
            goto END5;
        }
    }

    /************************************************
    step6:  get fd from VPSS
    *************************************************/
    for (i = 0; i < u32VdecChnNum; i++) {
        VdecFd[i] = HI_MPI_VPSS_GetChnFd(i, 0);
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
    step7:  start NNIE
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
    step8:  send stream to VDEC
    *************************************************/
    for (i = 0; i < u32VdecChnNum; i++) {
        snprintf(stVdecSend[i].cFileName, sizeof(stVdecSend[i].cFileName), "rtsp://admin:admin@192.168.19.101:554/cam/realmonitor?channel=1&subtype=0");
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

    SAMPLE_COMM_VDEC_StartSendStream(u32VdecChnNum, stVdecSend, VdecThread);

    iveWorkerS.frameQueue = &frameQueue;
    iveWorkerS.imageQueue = &imageQueue;

    SAMPLE_COMM_VDEC_StartIve(u32IveChnNum, &iveWorkerS, IveThread);

    nnieWorkerS.imageQueue = &imageQueue;
    nnieWorkerS.s_stSsdNnieParam = &s_stSsdNnieParam;

    SAMPLE_COMM_VDEC_StartNnie(u32NnieChnNum, &nnieWorkerS, NnieThread);

    while (1)
    {
        FD_ZERO(&read_fds);
        for (channel_index = 0; channel_index < u32VdecChnNum; channel_index++)
        {
            FD_SET(VdecFd[channel_index], &read_fds);
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
            SAMPLE_PRT("get vdec stream time out, exit thread\n");
            continue;
        }
        for (channel_index = 0; channel_index < u32VdecChnNum; channel_index++)
        {
            if (FD_ISSET(VdecFd[channel_index], &read_fds))
            {
#ifdef SAVE
                FILE * pFile = fopen("123.yuv", "wb");
                FILE * pFile_bgr = fopen("123.bgr_planar_384x216", "wb");
                FILE * pFile_bgr_1080p = fopen("123.bgr_planar_1920x1080", "wb");
#endif
                VIDEO_FRAME_INFO_S *video_frame_info_s = new VIDEO_FRAME_INFO_S;
                memset(video_frame_info_s, 0, sizeof(VIDEO_FRAME_INFO_S));

                s32Ret = HI_MPI_VPSS_GetChnFrame(channel_index, 0, video_frame_info_s, 0);

                if(s32Ret == HI_SUCCESS)
                {
                    frameQueue.push(video_frame_info_s);
/*
                    printf("physical addr = %llx \n", video_frame_info_s->stVFrame.u64PhyAddr[0]);
                    printf("virtual addr = %llx \n", video_frame_info_s->stVFrame.u64VirAddr[0]);
                    printf("height = %d \n", video_frame_info_s->stVFrame.u32Height);
                    printf("width = %d \n", video_frame_info_s->stVFrame.u32Width);
                    printf("pixle format = %d \n", video_frame_info_s->stVFrame.enPixelFormat);
                    printf("stride 0 = %d \n", video_frame_info_s->stVFrame.u32Stride[0]);
                    printf("stride 1 = %d \n", video_frame_info_s->stVFrame.u32Stride[1]);
*/

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

//                    FILE *pFile = fopen("123.yuv", "wb");
//                    SAMPLE_COMM_VDEC_SaveYUVFile_Linear8Bit(pFile, &(video_frame_info_s->stVFrame));
                }
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
        SAMPLE_COMM_VPSS_Stop(VpssGrp, abChnEnable);
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

    signal(SIGINT, SAMPLE_VDEC_HandleSig);
    signal(SIGTERM, SAMPLE_VDEC_HandleSig);

    /******************************************
     choose the case
    ******************************************/

    s32Ret = SAMPLE_RTSP_VDEC_VPSS();

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