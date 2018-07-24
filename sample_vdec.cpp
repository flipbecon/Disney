
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

#include "sample_comm.h"

#define SAMPLE_STREAM_PATH "./source_file"

#ifndef __HuaweiLite__
HI_VOID SAMPLE_VDEC_HandleSig(HI_S32 signo)
{
    if (SIGINT == signo || SIGTSTP == signo || SIGTERM == signo)
    {
        SAMPLE_COMM_SYS_Exit();
        printf("\033[0;31mprogram exit abnormally!\033[0;39m\n");
    }

    exit(0);
}
#endif

void SAMPLE_COMM_VDEC_SaveYUVFile_Linear8Bit(FILE* pfd, VIDEO_FRAME_S* pVBuf)
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


HI_VOID SAMPLE_VDEC_Usage(char *sPrgNm)
{
    printf("\n/************************************/\n");
    printf("Usage : %s <index> <IntfSync >\n", sPrgNm);
    printf("index:\n");
    printf("\t0:  VDEC(H265)-VPSS-VO\n");
    printf("\t1:  VDEC(H264)-VPSS-VO\n");
    printf("\t2:  VDEC(JPEG->YUV)-VPSS-VO\n");
    printf("\t3:  VDEC(JPEG->RGB)\n");

    printf("\nIntfSync :\n");
    printf("\t0: VO HDMI 4K@30fps.\n");
    printf("\t1: VO HDMI 1080P@30fps.\n");
    printf("/************************************/\n\n");
}

VO_INTF_SYNC_E g_enIntfSync = VO_OUTPUT_3840x2160_30;


HI_S32 SAMPLE_H265_VDEC_VPSS_VO(HI_VOID)
{
    VB_CONFIG_S stVbConfig;
    HI_S32 i, s32Ret = HI_SUCCESS;
    VDEC_THREAD_PARAM_S stVdecSend[VDEC_MAX_CHN_NUM];
    SIZE_S stDispSize;
    VO_LAYER VoLayer;
    HI_U32 u32VdecChnNum, VpssGrpNum;
    VPSS_GRP VpssGrp;
    pthread_t   VdecThread[2*VDEC_MAX_CHN_NUM];
    PIC_SIZE_E enDispPicSize;
    SAMPLE_VDEC_ATTR astSampleVdec[VDEC_MAX_CHN_NUM];
    VPSS_CHN_ATTR_S astVpssChnAttr[VPSS_MAX_CHN_NUM];
    SAMPLE_VO_CONFIG_S stVoConfig;
    VPSS_GRP_ATTR_S stVpssGrpAttr;
    HI_BOOL abChnEnable[VPSS_MAX_CHN_NUM];
    VO_INTF_SYNC_E enIntfSync;

    u32VdecChnNum = 1;
    VpssGrpNum    = u32VdecChnNum;
    /************************************************
    step1:  init SYS, init common VB(for VPSS and VO)
    *************************************************/

    if(VO_OUTPUT_3840x2160_30 == g_enIntfSync)
    {
        enDispPicSize = PIC_3840x2160;
        enIntfSync    = VO_OUTPUT_3840x2160_30;
    }
    else
    {
        enDispPicSize = PIC_1080P;
        enIntfSync    = VO_OUTPUT_1080P30;
    }

    s32Ret =  SAMPLE_COMM_SYS_GetPicSize(enDispPicSize, &stDispSize);
    if(s32Ret != HI_SUCCESS)
    {
        SAMPLE_PRT("sys get pic size fail for %#x!\n", s32Ret);
        goto END1;
    }

    memset(&stVbConfig, 0, sizeof(VB_CONFIG_S));
    stVbConfig.u32MaxPoolCnt             = 1;
    stVbConfig.astCommPool[0].u32BlkCnt  = 10*u32VdecChnNum;
    stVbConfig.astCommPool[0].u64BlkSize = COMMON_GetPicBufferSize(stDispSize.u32Width, stDispSize.u32Height,
                                                PIXEL_FORMAT_YVU_SEMIPLANAR_420, DATA_BITWIDTH_10, COMPRESS_MODE_SEG, 0);
    s32Ret = SAMPLE_COMM_SYS_Init(&stVbConfig);
    if(s32Ret != HI_SUCCESS)
    {
        SAMPLE_PRT("init sys fail for %#x!\n", s32Ret);
        goto END1;
    }

    /************************************************
    step2:  init module VB or user VB(for VDEC)
    *************************************************/
    for(i=0; i<u32VdecChnNum; i++)
    {
        astSampleVdec[i].enType                           = PT_H265;
        astSampleVdec[i].u32Width                         = 3840;
        astSampleVdec[i].u32Height                        = 2160;
        astSampleVdec[i].enMode                           = VIDEO_MODE_FRAME;
        astSampleVdec[i].stSapmleVdecVideo.enDecMode      = VIDEO_DEC_MODE_IP;
        astSampleVdec[i].stSapmleVdecVideo.enBitWidth     = DATA_BITWIDTH_10;
        astSampleVdec[i].stSapmleVdecVideo.u32RefFrameNum = 3;
        astSampleVdec[i].u32DisplayFrameNum               = 2;
        astSampleVdec[i].u32FrameBufCnt = astSampleVdec[i].stSapmleVdecVideo.u32RefFrameNum + astSampleVdec[i].u32DisplayFrameNum + 1;
    }
    s32Ret = SAMPLE_COMM_VDEC_InitVBPool(u32VdecChnNum, &astSampleVdec[0]);
    if(s32Ret != HI_SUCCESS)
    {
        SAMPLE_PRT("init mod common vb fail for %#x!\n", s32Ret);
        goto END2;
    }

    /************************************************
    step3:  start VDEC
    *************************************************/
    s32Ret = SAMPLE_COMM_VDEC_Start(u32VdecChnNum, &astSampleVdec[0]);
    if(s32Ret != HI_SUCCESS)
    {
        SAMPLE_PRT("start VDEC fail for %#x!\n", s32Ret);
        goto END3;
    }

    /************************************************
    step4:  start VPSS
    *************************************************/
    stVpssGrpAttr.u32MaxW = 3840;
    stVpssGrpAttr.u32MaxH = 2160;
    stVpssGrpAttr.stFrameRate.s32SrcFrameRate = -1;
    stVpssGrpAttr.stFrameRate.s32DstFrameRate = -1;
    stVpssGrpAttr.enDynamicRange = DYNAMIC_RANGE_HDR10;
    stVpssGrpAttr.enPixelFormat  = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    stVpssGrpAttr.bNrEn   = HI_FALSE;
    memset(abChnEnable, 0, sizeof(abChnEnable));
    abChnEnable[0] = HI_TRUE;
    astVpssChnAttr[0].u32Width                    = stDispSize.u32Width;
    astVpssChnAttr[0].u32Height                   = stDispSize.u32Height;
    astVpssChnAttr[0].enChnMode                   = VPSS_CHN_MODE_AUTO;
    astVpssChnAttr[0].enCompressMode              = COMPRESS_MODE_SEG;
    astVpssChnAttr[0].enDynamicRange              = DYNAMIC_RANGE_HDR10;
    astVpssChnAttr[0].enPixelFormat               = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    astVpssChnAttr[0].stFrameRate.s32SrcFrameRate = -1;
    astVpssChnAttr[0].stFrameRate.s32DstFrameRate = -1;
    astVpssChnAttr[0].u32Depth                    = 0;
    astVpssChnAttr[0].bMirror                     = HI_FALSE;
    astVpssChnAttr[0].bFlip                       = HI_FALSE;
    astVpssChnAttr[0].stAspectRatio.enMode        = ASPECT_RATIO_NONE;
    astVpssChnAttr[0].enVideoFormat               = VIDEO_FORMAT_LINEAR;
    for(i=0; i<u32VdecChnNum; i++)
    {
        VpssGrp = i;
    s32Ret = SAMPLE_COMM_VPSS_Start(VpssGrp, &abChnEnable[0], &stVpssGrpAttr, &astVpssChnAttr[0]);
    if(s32Ret != HI_SUCCESS)
    {
        SAMPLE_PRT("start VPSS fail for %#x!\n", s32Ret);
        goto END4;
        }
    }

    /************************************************
    step5:  start VO
    *************************************************/
    stVoConfig.VoDev                 = SAMPLE_VO_DEV_UHD;
    stVoConfig.enVoIntfType          = VO_INTF_HDMI;
    stVoConfig.enIntfSync            = enIntfSync;
    stVoConfig.enPicSize             = enDispPicSize;
    stVoConfig.u32BgColor            = COLOR_RGB_BLUE;
    stVoConfig.u32DisBufLen          = 3;
    stVoConfig.enDstDynamicRange     = DYNAMIC_RANGE_HDR10;
    stVoConfig.enVoMode              = VO_MODE_1MUX;
    stVoConfig.enPixFormat           = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    stVoConfig.stDispRect.s32X       = 0;
    stVoConfig.stDispRect.s32Y       = 0;
    stVoConfig.stDispRect.u32Width   = stDispSize.u32Width;
    stVoConfig.stDispRect.u32Height  = stDispSize.u32Height;
    stVoConfig.stImageSize.u32Width  = stDispSize.u32Width;
    stVoConfig.stImageSize.u32Height = stDispSize.u32Height;
    stVoConfig.enVoPartMode          = VO_PART_MODE_SINGLE;

    s32Ret = SAMPLE_COMM_VO_StartVO(&stVoConfig);
    if(s32Ret != HI_SUCCESS)
    {
        SAMPLE_PRT("start VO fail for %#x!\n", s32Ret);
        goto END5;
    }

    /************************************************
    step6:  VDEC bind VPSS
    *************************************************/
    for(i=0; i<u32VdecChnNum; i++)
    {
        s32Ret = SAMPLE_COMM_VDEC_Bind_VPSS(i, i);
        if(s32Ret != HI_SUCCESS)
        {
            SAMPLE_PRT("vdec bind vpss fail for %#x!\n", s32Ret);
            goto END6;
        }
    }

    /************************************************
    step7:  VPSS bind VO
    *************************************************/
    VoLayer = stVoConfig.VoDev;
    for(i=0; i<VpssGrpNum; i++)
    {
        s32Ret = SAMPLE_COMM_VPSS_Bind_VO(i, 0, VoLayer, i);
        if(s32Ret != HI_SUCCESS)
        {
            SAMPLE_PRT("vpss bind vo fail for %#x!\n", s32Ret);
            goto END7;
        }
    }

    /************************************************
    step8:  send stream to VDEC
    *************************************************/
    for(i=0; i<u32VdecChnNum; i++)
    {
        snprintf(stVdecSend[i].cFileName, sizeof(stVdecSend[i].cFileName), "3840x2160_10b it.h265");
        snprintf(stVdecSend[i].cFilePath, sizeof(stVdecSend[i].cFilePath), "%s", SAMPLE_STREAM_PATH);
        stVdecSend[i].enType          = astSampleVdec[i].enType;
        stVdecSend[i].s32StreamMode   = astSampleVdec[i].enMode;
        stVdecSend[i].s32ChnId        = i;
        stVdecSend[i].s32IntervalTime = 1000;
        stVdecSend[i].u64PtsInit      = 0;
        stVdecSend[i].u64PtsIncrease  = 0;
        stVdecSend[i].eThreadCtrl     = THREAD_CTRL_START;
        stVdecSend[i].bCircleSend     = HI_TRUE;
        stVdecSend[i].s32MilliSec     = 0;
        stVdecSend[i].s32MinBufSize   = (astSampleVdec[i].u32Width * astSampleVdec[i].u32Height * 3)>>1;
    }
    SAMPLE_COMM_VDEC_StartSendStream(u32VdecChnNum, &stVdecSend[0], &VdecThread[0]);

    SAMPLE_COMM_VDEC_CmdCtrl(u32VdecChnNum, &stVdecSend[0], &VdecThread[0]);

    SAMPLE_COMM_VDEC_StopSendStream(u32VdecChnNum, &stVdecSend[0], &VdecThread[0]);

END7:
    for(i=0; i<VpssGrpNum; i++)
    {
        s32Ret = SAMPLE_COMM_VPSS_UnBind_VO(i, 0, VoLayer, i);
        if(s32Ret != HI_SUCCESS)
        {
            SAMPLE_PRT("vpss unbind vo fail for %#x!\n", s32Ret);
        }
    }

END6:
    for(i=0; i<u32VdecChnNum; i++)
    {
        s32Ret = SAMPLE_COMM_VDEC_UnBind_VPSS(i, i);
        if(s32Ret != HI_SUCCESS)
        {
            SAMPLE_PRT("vdec unbind vpss fail for %#x!\n", s32Ret);
        }
    }

END5:
    SAMPLE_COMM_VO_StopVO(&stVoConfig);

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


HI_S32 SAMPLE_H264_VDEC_VPSS_VO(HI_VOID) {
    VB_CONFIG_S stVbConfig;
    HI_S32 i, s32Ret = HI_SUCCESS;
    VDEC_THREAD_PARAM_S stVdecSend[VDEC_MAX_CHN_NUM];
    SIZE_S stDispSize;
    VO_LAYER VoLayer;
    HI_U32 u32VdecChnNum, VpssGrpNum;
    VPSS_GRP VpssGrp;
    pthread_t VdecThread[2 * VDEC_MAX_CHN_NUM];
    PIC_SIZE_E enDispPicSize;
    SAMPLE_VDEC_ATTR astSampleVdec[VDEC_MAX_CHN_NUM];
    VPSS_CHN_ATTR_S astVpssChnAttr[VPSS_MAX_CHN_NUM];
    SAMPLE_VO_CONFIG_S stVoConfig;
    VPSS_GRP_ATTR_S stVpssGrpAttr;
    HI_BOOL abChnEnable[VPSS_MAX_CHN_NUM];
    VO_INTF_SYNC_E enIntfSync;

    u32VdecChnNum = 1;
    VpssGrpNum = u32VdecChnNum;

    /************************************************
    step1:  init SYS, init common VB(for VPSS and VO)
    *************************************************/
    if (VO_OUTPUT_3840x2160_30 == g_enIntfSync) {
        enDispPicSize = PIC_3840x2160;
        enIntfSync = VO_OUTPUT_3840x2160_30;
    } else {
        enDispPicSize = PIC_1080P;
        enIntfSync = VO_OUTPUT_1080P30;
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
        astSampleVdec[i].u32Width = 3840;
        astSampleVdec[i].u32Height = 2160;
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
    stVpssGrpAttr.u32MaxW = 3840;
    stVpssGrpAttr.u32MaxH = 2160;
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
    step6:  VDEC bind VPSS
    *************************************************/
    for (i = 0; i < u32VdecChnNum; i++) {
        s32Ret = SAMPLE_COMM_VDEC_Bind_VPSS(i, i);
        if (s32Ret != HI_SUCCESS) {
            SAMPLE_PRT("vdec bind vpss fail for %#x!\n", s32Ret);
            goto END6;
        }
    }

    /************************************************
    step8:  send stream to VDEC
    *************************************************/
    for (i = 0; i < u32VdecChnNum; i++) {
        snprintf(stVdecSend[i].cFileName, sizeof(stVdecSend[i].cFileName), "3840x2160_8bit.h264");
        snprintf(stVdecSend[i].cFilePath, sizeof(stVdecSend[i].cFilePath), "%s", SAMPLE_STREAM_PATH);
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

    char szFilename[32];
    int k =0;
    while (1)
    {
        sprintf(szFilename, "frame%d.yuv", k);
        FILE * pFile = fopen("123.yuv", "wb");
        VIDEO_FRAME_INFO_S *video_frame_info_s = new VIDEO_FRAME_INFO_S;
        memset(video_frame_info_s, 0, sizeof(VIDEO_FRAME_INFO_S));

        s32Ret = HI_MPI_VPSS_GetChnFrame(VpssGrp, 0, video_frame_info_s, 0);
        printf("vdec get frame return = %x \n", s32Ret);
        if(s32Ret == HI_SUCCESS)
        {
            printf("physical addr = %x \n", video_frame_info_s->stVFrame.u64PhyAddr);
            printf("virtual addr = %x \n", video_frame_info_s->stVFrame.u64VirAddr);
            printf("height = %d \n", video_frame_info_s->stVFrame.u32Height);
            printf("width = %d \n", video_frame_info_s->stVFrame.u32Width);
            printf("pixle format = %d \n", video_frame_info_s->stVFrame.enPixelFormat);

            SAMPLE_COMM_VDEC_SaveYUVFile_Linear8Bit(pFile, &(video_frame_info_s->stVFrame));

            HI_MPI_VPSS_ReleaseChnFrame(VpssGrp, 0, video_frame_info_s);
        }
        delete video_frame_info_s;
        video_frame_info_s = nullptr;

        fclose(pFile);
        k++;
    };
    SAMPLE_COMM_VDEC_CmdCtrl(u32VdecChnNum, &stVdecSend[0], &VdecThread[0]);

    SAMPLE_COMM_VDEC_StopSendStream(u32VdecChnNum, &stVdecSend[0], &VdecThread[0]);

END7:
    for(i=0; i<VpssGrpNum; i++)
    {
        s32Ret = SAMPLE_COMM_VPSS_UnBind_VO(i, 0, VoLayer, i);
        if(s32Ret != HI_SUCCESS)
        {
            SAMPLE_PRT("vpss unbind vo fail for %#x!\n", s32Ret);
        }
    }

END6:
    for(i=0; i<u32VdecChnNum; i++)
    {
        s32Ret = SAMPLE_COMM_VDEC_UnBind_VPSS(i, i);
        if(s32Ret != HI_SUCCESS)
        {
            SAMPLE_PRT("vdec unbind vpss fail for %#x!\n", s32Ret);
        }
    }

END5:
    SAMPLE_COMM_VO_StopVO(&stVoConfig);

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


HI_S32 SAMPLE_JPEG_VDEC_VPSS_VO(HI_VOID)
{
    VB_CONFIG_S stVbConfig;
    HI_S32 i, s32Ret = HI_SUCCESS;
    VDEC_THREAD_PARAM_S stVdecSend[VDEC_MAX_CHN_NUM];
    SIZE_S stDispSize;
    VO_LAYER VoLayer;
    HI_U32 u32VdecChnNum, VpssGrpNum;
    VPSS_GRP VpssGrp;
    pthread_t   VdecThread[2*VDEC_MAX_CHN_NUM];
    PIC_SIZE_E enDispPicSize;
    SAMPLE_VDEC_ATTR astSampleVdec[VDEC_MAX_CHN_NUM];
    VPSS_CHN_ATTR_S astVpssChnAttr[VPSS_MAX_CHN_NUM];
    SAMPLE_VO_CONFIG_S stVoConfig;
    VPSS_GRP_ATTR_S stVpssGrpAttr;
    HI_BOOL abChnEnable[VPSS_MAX_CHN_NUM];
    VO_INTF_SYNC_E enIntfSync;

    u32VdecChnNum = 1;
    VpssGrpNum    = u32VdecChnNum;

    /************************************************
    step1:  init SYS, init common VB(for VPSS and VO)
    *************************************************/
    if(VO_OUTPUT_3840x2160_30 == g_enIntfSync)
    {
        enDispPicSize = PIC_3840x2160;
        enIntfSync    = VO_OUTPUT_3840x2160_30;
    }
    else
    {
        enDispPicSize = PIC_1080P;
        enIntfSync    = VO_OUTPUT_1080P30;
    }

    s32Ret =  SAMPLE_COMM_SYS_GetPicSize(enDispPicSize, &stDispSize);
    if(s32Ret != HI_SUCCESS)
    {
        SAMPLE_PRT("sys get pic size fail for %#x!\n", s32Ret);
        goto END1;
    }

    memset(&stVbConfig, 0, sizeof(VB_CONFIG_S));
    stVbConfig.u32MaxPoolCnt             = 1;
    stVbConfig.astCommPool[0].u32BlkCnt  = 10*u32VdecChnNum;
    stVbConfig.astCommPool[0].u64BlkSize = COMMON_GetPicBufferSize(stDispSize.u32Width, stDispSize.u32Height,
                                                PIXEL_FORMAT_YVU_SEMIPLANAR_420, DATA_BITWIDTH_8, COMPRESS_MODE_SEG, 0);
    s32Ret = SAMPLE_COMM_SYS_Init(&stVbConfig);
    if(s32Ret != HI_SUCCESS)
    {
        SAMPLE_PRT("init sys fail for %#x!\n", s32Ret);
        goto END1;
    }

    /************************************************
    step2:  init module VB or user VB(for VDEC)
    *************************************************/
    for(i=0; i<u32VdecChnNum; i++)
    {
        astSampleVdec[i].enType                            = PT_JPEG;
        astSampleVdec[i].u32Width                          = 3840;
        astSampleVdec[i].u32Height                         = 2160;
        astSampleVdec[i].enMode                            = VIDEO_MODE_FRAME;
        astSampleVdec[i].stSapmleVdecPicture.enPixelFormat = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
        astSampleVdec[i].stSapmleVdecPicture.u32Alpha      = 255;
        astSampleVdec[i].u32DisplayFrameNum                = 2;
        astSampleVdec[i].u32FrameBufCnt                    = astSampleVdec[i].u32DisplayFrameNum + 1;
    }
    s32Ret = SAMPLE_COMM_VDEC_InitVBPool(u32VdecChnNum, &astSampleVdec[0]);
    if(s32Ret != HI_SUCCESS)
    {
        SAMPLE_PRT("init mod common vb fail for %#x!\n", s32Ret);
        goto END2;
    }

    /************************************************
    step3:  start VDEC
    *************************************************/
    s32Ret = SAMPLE_COMM_VDEC_Start(u32VdecChnNum, &astSampleVdec[0]);
    if(s32Ret != HI_SUCCESS)
    {
        SAMPLE_PRT("start VDEC fail for %#x!\n", s32Ret);
        goto END3;
    }

    /************************************************
    step4:  start VPSS
    *************************************************/
    stVpssGrpAttr.u32MaxW = 3840;
    stVpssGrpAttr.u32MaxH = 2160;
    stVpssGrpAttr.stFrameRate.s32SrcFrameRate = -1;
    stVpssGrpAttr.stFrameRate.s32DstFrameRate = -1;
    stVpssGrpAttr.enDynamicRange = DYNAMIC_RANGE_SDR8;
    stVpssGrpAttr.enPixelFormat  = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    stVpssGrpAttr.bNrEn   = HI_FALSE;

    memset(abChnEnable, 0, sizeof(abChnEnable));
    abChnEnable[0] = HI_TRUE;
    astVpssChnAttr[0].u32Width                    = stDispSize.u32Width;
    astVpssChnAttr[0].u32Height                   = stDispSize.u32Height;
    astVpssChnAttr[0].enChnMode                   = VPSS_CHN_MODE_AUTO;
    astVpssChnAttr[0].enCompressMode              = COMPRESS_MODE_SEG;
    astVpssChnAttr[0].enDynamicRange              = DYNAMIC_RANGE_SDR8;
    astVpssChnAttr[0].enPixelFormat               = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    astVpssChnAttr[0].stFrameRate.s32SrcFrameRate = -1;
    astVpssChnAttr[0].stFrameRate.s32DstFrameRate = -1;
    astVpssChnAttr[0].u32Depth                    = 0;
    astVpssChnAttr[0].bMirror                     = HI_FALSE;
    astVpssChnAttr[0].bFlip                       = HI_FALSE;
    astVpssChnAttr[0].stAspectRatio.enMode        = ASPECT_RATIO_NONE;
    astVpssChnAttr[0].enVideoFormat               = VIDEO_FORMAT_LINEAR;

    for(i=0; i<u32VdecChnNum; i++)
    {
        VpssGrp = i;
        s32Ret = SAMPLE_COMM_VPSS_Start(VpssGrp, &abChnEnable[0], &stVpssGrpAttr, &astVpssChnAttr[0]);
        if(s32Ret != HI_SUCCESS)
        {
            SAMPLE_PRT("start VPSS fail for %#x!\n", s32Ret);
            goto END4;
        }
    }



    /************************************************
    step5:  start VO
    *************************************************/
    stVoConfig.VoDev                 = SAMPLE_VO_DEV_UHD;
    stVoConfig.enVoIntfType          = VO_INTF_HDMI;
    stVoConfig.enIntfSync            = enIntfSync;
    stVoConfig.enPicSize             = enDispPicSize;
    stVoConfig.u32BgColor            = COLOR_RGB_BLUE;
    stVoConfig.u32DisBufLen          = 3;
    stVoConfig.enDstDynamicRange     = DYNAMIC_RANGE_SDR8;
    stVoConfig.enVoMode              = VO_MODE_1MUX;
    stVoConfig.enPixFormat           = PIXEL_FORMAT_YVU_SEMIPLANAR_420;
    stVoConfig.stDispRect.s32X       = 0;
    stVoConfig.stDispRect.s32Y       = 0;
    stVoConfig.stDispRect.u32Width   = stDispSize.u32Width;
    stVoConfig.stDispRect.u32Height  = stDispSize.u32Height;
    stVoConfig.stImageSize.u32Width  = stDispSize.u32Width;
    stVoConfig.stImageSize.u32Height = stDispSize.u32Height;
    stVoConfig.enVoPartMode          = VO_PART_MODE_SINGLE;
    s32Ret = SAMPLE_COMM_VO_StartVO(&stVoConfig);
    if(s32Ret != HI_SUCCESS)
    {
        SAMPLE_PRT("start VO fail for %#x!\n", s32Ret);
        goto END5;
    }

    /************************************************
    step6:  VDEC bind VPSS
    *************************************************/
    for(i=0; i<u32VdecChnNum; i++)
    {
        s32Ret = SAMPLE_COMM_VDEC_Bind_VPSS(i, i);
        if(s32Ret != HI_SUCCESS)
        {
            SAMPLE_PRT("vdec bind vpss fail for %#x!\n", s32Ret);
            goto END6;
        }
    }

    /************************************************
    step7:  VPSS bind VO
    *************************************************/
    VoLayer = stVoConfig.VoDev;
    for(i=0; i<VpssGrpNum; i++)
    {
        s32Ret = SAMPLE_COMM_VPSS_Bind_VO(i, 0, VoLayer, i);
        if(s32Ret != HI_SUCCESS)
        {
            SAMPLE_PRT("vpss bind vo fail for %#x!\n", s32Ret);
            goto END7;
        }
    }

    /************************************************
    step8:  send stream to VDEC
    *************************************************/
    for(i=0; i<u32VdecChnNum; i++)
    {
        snprintf(stVdecSend[i].cFileName, sizeof(stVdecSend[i].cFileName), "3840x2160.jpg");
        snprintf(stVdecSend[i].cFilePath, sizeof(stVdecSend[i].cFilePath), "%s", SAMPLE_STREAM_PATH);
        stVdecSend[i].enType          = astSampleVdec[i].enType;
        stVdecSend[i].s32StreamMode   = astSampleVdec[i].enMode;
        stVdecSend[i].s32ChnId        = i;
        stVdecSend[i].s32IntervalTime = 1000;
        stVdecSend[i].u64PtsInit      = 0;
        stVdecSend[i].u64PtsIncrease  = 0;
        stVdecSend[i].eThreadCtrl     = THREAD_CTRL_START;
        stVdecSend[i].bCircleSend     = HI_TRUE;
        stVdecSend[i].s32MilliSec     = 0;
        stVdecSend[i].s32MinBufSize   = (astSampleVdec[i].u32Width * astSampleVdec[i].u32Height * 3)>>1;
    }
    SAMPLE_COMM_VDEC_StartSendStream(u32VdecChnNum, &stVdecSend[0], &VdecThread[0]);

    SAMPLE_COMM_VDEC_CmdCtrl(u32VdecChnNum, &stVdecSend[0], &VdecThread[0]);

    SAMPLE_COMM_VDEC_StopSendStream(u32VdecChnNum, &stVdecSend[0], &VdecThread[0]);

END7:
    for(i=0; i<VpssGrpNum; i++)
    {
        s32Ret = SAMPLE_COMM_VPSS_UnBind_VO(i, 0, VoLayer, i);
        if(s32Ret != HI_SUCCESS)
        {
            SAMPLE_PRT("vpss unbind vo fail for %#x!\n", s32Ret);
        }
    }

END6:
    for(i=0; i<u32VdecChnNum; i++)
    {
        s32Ret = SAMPLE_COMM_VDEC_UnBind_VPSS(i, i);
        if(s32Ret != HI_SUCCESS)
        {
            SAMPLE_PRT("vdec unbind vpss fail for %#x!\n", s32Ret);
        }
    }

END5:
    SAMPLE_COMM_VO_StopVO(&stVoConfig);

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


HI_S32 SAMPLE_JPEG_VDEC_To_RGB(HI_VOID)
{
    VB_CONFIG_S stVbConfig;
    HI_S32 i, s32Ret = HI_SUCCESS;
    VDEC_THREAD_PARAM_S stVdecSend[VDEC_MAX_CHN_NUM];
    HI_U32 u32VdecChnNum;
    pthread_t   VdecThread[2*VDEC_MAX_CHN_NUM];
    SAMPLE_VDEC_ATTR astSampleVdec[VDEC_MAX_CHN_NUM];

    u32VdecChnNum = 1;

    /************************************************
    step1:  init SYS
    *************************************************/
    memset(&stVbConfig, 0, sizeof(VB_CONFIG_S));
    s32Ret = SAMPLE_COMM_SYS_Init(&stVbConfig);
    if(s32Ret != HI_SUCCESS)
    {
        SAMPLE_PRT("init sys fail for %#x!\n", s32Ret);
        goto END1;
    }

    /************************************************
    step2:  init module VB or user VB(for VDEC)
    *************************************************/
    for(i=0; i<u32VdecChnNum; i++)
    {
        astSampleVdec[i].enType                            = PT_JPEG;
        astSampleVdec[i].u32Width                          = 1920;
        astSampleVdec[i].u32Height                         = 1080;
        astSampleVdec[i].enMode                            = VIDEO_MODE_FRAME;
        astSampleVdec[i].stSapmleVdecPicture.enPixelFormat = PIXEL_FORMAT_ARGB_8888;
        astSampleVdec[i].stSapmleVdecPicture.u32Alpha      = 255;
        astSampleVdec[i].u32DisplayFrameNum                = 0;
        astSampleVdec[i].u32FrameBufCnt                    = astSampleVdec[i].u32DisplayFrameNum + 1;
    }
    s32Ret = SAMPLE_COMM_VDEC_InitVBPool(u32VdecChnNum, &astSampleVdec[0]);
    if(s32Ret != HI_SUCCESS)
    {
        SAMPLE_PRT("init mod common vb fail for %#x!\n", s32Ret);
        goto END2;
    }

    /************************************************
    step3:  start VDEC
    *************************************************/
    s32Ret = SAMPLE_COMM_VDEC_Start(u32VdecChnNum, &astSampleVdec[0]);
    if(s32Ret != HI_SUCCESS)
    {
        SAMPLE_PRT("start VDEC fail for %#x!\n", s32Ret);
        goto END3;
    }

    /************************************************
    step4:  send stream to VDEC
    *************************************************/
    for(i=0; i<u32VdecChnNum; i++)
    {
        snprintf(stVdecSend[i].cFileName, sizeof(stVdecSend[i].cFileName), "1080P.jpg");
        snprintf(stVdecSend[i].cFilePath, sizeof(stVdecSend[i].cFilePath), "%s", SAMPLE_STREAM_PATH);
        stVdecSend[i].enType          = astSampleVdec[i].enType;
        stVdecSend[i].s32StreamMode   = astSampleVdec[i].enMode;
        stVdecSend[i].s32ChnId        = i;
        stVdecSend[i].s32IntervalTime = 1000;
        stVdecSend[i].u64PtsInit      = 0;
        stVdecSend[i].u64PtsIncrease  = 0;
        stVdecSend[i].eThreadCtrl     = THREAD_CTRL_START;
        stVdecSend[i].bCircleSend     = HI_FALSE;
        stVdecSend[i].s32MilliSec     = 0;
        stVdecSend[i].s32MinBufSize   = astSampleVdec[i].u32Width * astSampleVdec[i].u32Height;
    }
    SAMPLE_COMM_VDEC_StartSendStream(u32VdecChnNum, &stVdecSend[0], &VdecThread[0]);
    SAMPLE_COMM_VDEC_StartGetPic(u32VdecChnNum, &stVdecSend[0], &VdecThread[0]);

    SAMPLE_COMM_VDEC_CmdCtrl(u32VdecChnNum, &stVdecSend[0], &VdecThread[0]);

    SAMPLE_COMM_VDEC_StopSendStream(u32VdecChnNum, &stVdecSend[0], &VdecThread[0]);
    SAMPLE_COMM_VDEC_StopGetPic(u32VdecChnNum, &stVdecSend[0], &VdecThread[0]);

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
#ifdef __HuaweiLite__
    int app_main(int argc, char *argv[])
#else
    int main(int argc, char *argv[])
#endif
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

#ifndef __HuaweiLite__
    signal(SIGINT, SAMPLE_VDEC_HandleSig);
    signal(SIGTERM, SAMPLE_VDEC_HandleSig);
#endif

    /******************************************
     choose the case
    ******************************************/
    switch (*argv[1])
    {
        case '0':
        {
            s32Ret = SAMPLE_H265_VDEC_VPSS_VO();
            break;
        }
        case '1':
        {
            s32Ret = SAMPLE_H264_VDEC_VPSS_VO();
            break;
        }
        case '2':
        {
            s32Ret = SAMPLE_JPEG_VDEC_VPSS_VO();
            break;
        }
        case '3':
        {
            s32Ret = SAMPLE_JPEG_VDEC_To_RGB();
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


