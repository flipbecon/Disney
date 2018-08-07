#include "ive.h"

using namespace std;

HI_VOID *SAMPLE_COMM_VDEC_IVE(HI_VOID *pArgs)
{
    printf("ive worker entered \n");

    IVE_WORKER_S* ive_worker_s = (IVE_WORKER_S *)pArgs;


    VIDEO_FRAME_INFO_S * video_frame_info_s;
    HI_S32 s32Ret;
    HI_U32 u32Size;

    while (1)
    {
        if(!(ive_worker_s->frameQueue->try_pop(video_frame_info_s)))
        {
            sleep(1);
            printf("ive wait \n");
            continue;
        }

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
            break;
        }
        pstImg->au32Stride[1] = pstImg->u32Width;
        pstImg->au32Stride[2] = pstImg->u32Width;
        pstImg->au64PhyAddr[1] = pstImg->au64PhyAddr[0] + pstImg->u32Width * pstImg->u32Height;
        pstImg->au64VirAddr[1] = pstImg->au64VirAddr[0] + pstImg->u32Width * pstImg->u32Height;
        pstImg->au64PhyAddr[2] = pstImg->au64PhyAddr[0] + pstImg->u32Width * pstImg->u32Height *2;
        pstImg->au64VirAddr[2] = pstImg->au64VirAddr[0] + pstImg->u32Width * pstImg->u32Height *2;

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
            return nullptr;
        }
        pstDstImg->au32Stride[1] = pstDstImg->au32Stride[0];
        pstDstImg->au32Stride[2] = pstDstImg->au32Stride[0];
        pstDstImg->au64PhyAddr[1] = pstDstImg->au64PhyAddr[0] + pstDstImg->au32Stride[0] * pstDstImg->u32Height;
        pstDstImg->au64VirAddr[1] = pstDstImg->au64VirAddr[0] + pstDstImg->au32Stride[0] * pstDstImg->u32Height;
        pstDstImg->au64PhyAddr[2] = pstDstImg->au64PhyAddr[0] + pstDstImg->au32Stride[0] * pstDstImg->u32Height *2;
        pstDstImg->au64VirAddr[2] = pstDstImg->au64VirAddr[0] + pstDstImg->au32Stride[0] * pstDstImg->u32Height *2;

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
        s32Ret = DGIveResize(pstImg, pstDstImg);

        if (s32Ret != HI_SUCCESS)
        {
            SAMPLE_PRT("Resize fail,Error(%#x)\n", s32Ret);
            return nullptr;
        }

        ive_worker_s->imageQueue->push(pstDstImg);

        (HI_VOID)HI_MPI_SYS_MmzFree(pstImg->au64PhyAddr[0], (void *) (pstImg->au64VirAddr[0]));

        HI_MPI_VPSS_ReleaseChnFrame(0, 0, video_frame_info_s);
        delete video_frame_info_s;
        video_frame_info_s = nullptr;
        delete pstImg;
        pstImg = nullptr;
    }
    printf("\033[Ive thread return ...  \033[0;39m\n");
    fflush(stdout);
    return nullptr;
}

HI_VOID SAMPLE_COMM_VDEC_StartIve(HI_S32 s32ChnNum, IVE_WORKER_S *pstIveWorker, pthread_t *pIveThread)
{
    printf("ive worker init \n");
    pthread_create(&pIveThread[0], 0, SAMPLE_COMM_VDEC_IVE, (HI_VOID *)pstIveWorker);
    //thread(SAMPLE_COMM_VDEC_IVE, ref(frameQueue), ref(imageQueue));
    printf("ive worker done \n");
}
