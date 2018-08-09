#include "utils.h"

//static void bench_begin()
//{
//    gettimeofday(&tv_begin, NULL);
//}
//
//static void bench_end(const char *args)
//{
//    gettimeofday(&tv_end, NULL);
//    elasped = ((tv_end.tv_sec - tv_begin.tv_sec) * 1000000.0f + tv_end.tv_usec - tv_begin.tv_usec) / 1000.0f;
//    printf("time spent = %f", elasped);
//}

HI_U16 IVE_CalcStride(HI_U32 u32Width, HI_U8 u8Align)
{
    return (u32Width + (u8Align - u32Width % u8Align) % u8Align);
}

HI_S32 DGIveSP420ToBGR24(IVE_IMAGE_S *pSrcImage, IVE_IMAGE_S *pDstImage)
{
    HI_S32 		   s32Ret;
    IVE_HANDLE 	   hIveHandle;
    IVE_CSC_CTRL_S stCtrl;
    HI_BOOL 	   bFinish  = HI_FALSE;
    HI_BOOL 	   bBlock   = HI_TRUE;
    HI_BOOL        bInstant = HI_TRUE;

    stCtrl.enMode = IVE_CSC_MODE_PIC_BT601_YUV2RGB;

    s32Ret = HI_MPI_IVE_CSC(&hIveHandle, pSrcImage, pDstImage, &stCtrl, bInstant);
    if (HI_SUCCESS != s32Ret)
    {
        printf("HI_MPI_IVE_CSC error s32Ret=0x%x\n",s32Ret);
        return s32Ret;
    }

    if (HI_TRUE == bInstant)
    {
        s32Ret = HI_MPI_IVE_Query(hIveHandle, &bFinish, bBlock);

        while(HI_ERR_IVE_QUERY_TIMEOUT == s32Ret)
        {
            usleep(1000);
            s32Ret = HI_MPI_IVE_Query(hIveHandle, &bFinish, bBlock);
        }

        if (HI_SUCCESS != s32Ret)
        {
            printf("HI_MPI_IVE_Query fail,Error(%#x)\n", s32Ret);
            return s32Ret;
        }
    }

    return s32Ret;
}

HI_S32 DGIveBGR24ToSP420(IVE_IMAGE_S *pSrcImage, IVE_IMAGE_S *pDstImage)
{
    HI_S32 		   s32Ret;
    IVE_HANDLE 	   hIveHandle;
    IVE_CSC_CTRL_S stCtrl;
    HI_BOOL 	   bFinish  = HI_FALSE;
    HI_BOOL 	   bBlock   = HI_TRUE;
    HI_BOOL        bInstant = HI_TRUE;

    stCtrl.enMode = IVE_CSC_MODE_PIC_BT601_RGB2YUV;

    s32Ret = HI_MPI_IVE_CSC(&hIveHandle, pSrcImage, pDstImage, &stCtrl, bInstant);
    if (HI_SUCCESS != s32Ret)
    {
        printf("HI_MPI_IVE_CSC error s32Ret=0x%x\n",s32Ret);
        return s32Ret;
    }

    if (HI_TRUE == bInstant)
    {
        s32Ret = HI_MPI_IVE_Query(hIveHandle, &bFinish, bBlock);

        while(HI_ERR_IVE_QUERY_TIMEOUT == s32Ret)
        {
            usleep(1000);
            s32Ret = HI_MPI_IVE_Query(hIveHandle, &bFinish, bBlock);
        }

        if (HI_SUCCESS != s32Ret)
        {
            printf("HI_MPI_IVE_Query fail,Error(%#x)\n", s32Ret);
            return s32Ret;
        }
    }

    return s32Ret;
}

HI_S32 DGIveSP420ToHSV(IVE_IMAGE_S *pSrcImage, IVE_IMAGE_S *pDstImage)
{
    HI_S32 		   s32Ret;
    IVE_HANDLE 	   hIveHandle;
    IVE_CSC_CTRL_S stCtrl;
    HI_BOOL 	   bFinish  = HI_FALSE;
    HI_BOOL 	   bBlock   = HI_TRUE;
    HI_BOOL        bInstant = HI_TRUE;

    stCtrl.enMode = IVE_CSC_MODE_PIC_BT601_YUV2HSV;

    s32Ret = HI_MPI_IVE_CSC(&hIveHandle, pSrcImage, pDstImage, &stCtrl, bInstant);
    if (HI_SUCCESS != s32Ret)
    {
        printf("HI_MPI_IVE_CSC error s32Ret=0x%x\n",s32Ret);
        return s32Ret;
    }

    if (HI_TRUE == bInstant)
    {
        s32Ret = HI_MPI_IVE_Query(hIveHandle, &bFinish, bBlock);

        while(HI_ERR_IVE_QUERY_TIMEOUT == s32Ret)
        {
            usleep(1000);
            s32Ret = HI_MPI_IVE_Query(hIveHandle, &bFinish, bBlock);
        }

        if (HI_SUCCESS != s32Ret)
        {
            printf("HI_MPI_IVE_Query fail,Error(%#x)\n", s32Ret);
            return s32Ret;
        }
    }

    return s32Ret;
}

HI_S32 DGSP420ToBGR24Planar(VIDEO_FRAME_S *stSrcFrame, IVE_IMAGE_S *stDstImage)
{
    HI_S32       s32Ret;

    IVE_IMAGE_S stSrcImg;

    stSrcImg.enType         = IVE_IMAGE_TYPE_YUV420SP;
    stSrcImg.u32Width       = stSrcFrame->u32Width;
    stSrcImg.u32Height      = stSrcFrame->u32Height;
    stSrcImg.au64VirAddr[0] = stSrcFrame->u64VirAddr[0];
    stSrcImg.au64VirAddr[1] = stSrcFrame->u64VirAddr[0] + stSrcFrame->u32Stride[0] * stSrcFrame->u32Height;
    stSrcImg.au64VirAddr[2] = NULL;

    stSrcImg.au64PhyAddr[0] = stSrcFrame->u64PhyAddr[0];
    stSrcImg.au64PhyAddr[1] = stSrcFrame->u64PhyAddr[0] + stSrcFrame->u32Stride[0] * stSrcFrame->u32Height;
    stSrcImg.au64PhyAddr[2] = 0;

    stSrcImg.au32Stride[0]  = stSrcFrame->u32Stride[0];
    stSrcImg.au32Stride[1]  = stSrcFrame->u32Stride[1];
    stSrcImg.au32Stride[2]  = 0;

    s32Ret = DGIveSP420ToBGR24(&stSrcImg, stDstImage);
    if (HI_SUCCESS != s32Ret)
    {
        printf("DGIveSP420ToBGR24, error s32Ret=0x%x\n", s32Ret);
        return s32Ret;
    }

    return s32Ret;
}

HI_S32 DGIveResize(IVE_SRC_IMAGE_S astSrc[], IVE_DST_IMAGE_S astDst[])
{
    HI_S32             s32Ret;
    IVE_HANDLE         hIveHandle;
    IVE_RESIZE_CTRL_S  stCtrl;
    HI_BOOL            bFinish  = HI_FALSE;
    HI_BOOL            bBlock   = HI_TRUE;
    HI_BOOL            bInstant = HI_TRUE;

    stCtrl.u16Num = 1;
    stCtrl.enMode = IVE_RESIZE_MODE_LINEAR;
    stCtrl.stMem.u32Size = stCtrl.u16Num * 49;
    s32Ret = HI_MPI_SYS_MmzAlloc(&stCtrl.stMem.u64PhyAddr, (void**)&stCtrl.stMem.u64VirAddr, "ResizeCtrlMem", NULL, stCtrl.stMem.u32Size);
    if (s32Ret != HI_SUCCESS)
    {
        printf("Error(%#x),Create stCtrl.stMem mem failed!\n",s32Ret);
        return s32Ret;
    }

    s32Ret = HI_MPI_IVE_Resize(&hIveHandle, astSrc, astDst, &stCtrl, bInstant);
    if (HI_SUCCESS != s32Ret)
    {
        printf("HI_MPI_IVE_Resize error s32Ret=0x%x\n",s32Ret);
        return s32Ret;
    }

    if (HI_TRUE == bInstant)
    {
        s32Ret = HI_MPI_IVE_Query(hIveHandle, &bFinish, bBlock);

        while(HI_ERR_IVE_QUERY_TIMEOUT == s32Ret)
        {
            usleep(1000);
            s32Ret = HI_MPI_IVE_Query(hIveHandle, &bFinish, bBlock);
        }

        if (HI_SUCCESS != s32Ret)
        {
            printf("HI_MPI_IVE_Query fail,Error(%#x)\n", s32Ret);
            return s32Ret;
        }
    }

    HI_MPI_SYS_MmzFree(stCtrl.stMem.u64PhyAddr, &stCtrl.stMem.u64VirAddr);

    return s32Ret;
}