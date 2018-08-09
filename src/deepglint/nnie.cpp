#include "nnie.h"

using namespace std;

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


HI_VOID * SAMPLE_COMM_VDEC_NNIE(HI_VOID *pArgs)
{
    NNIE_WORKER_S *nnie_worker_s = (NNIE_WORKER_S *)pArgs;

    pair<std::pair<HI_U32, VIDEO_FRAME_INFO_S *>, IVE_IMAGE_S *> pairs;

    HI_S32 s32Ret;

    while (1)
    {
        if(!(nnie_worker_s->pipeline->try_pop(pairs)))
        {
            usleep(1000);
            continue;
        }

        nnie_worker_s->s_stSsdNnieParam->astSegData[0].astSrc[0].u64VirAddr = pairs.second->au64VirAddr[0];
        nnie_worker_s->s_stSsdNnieParam->astSegData[0].astSrc[0].u64PhyAddr = pairs.second->au64PhyAddr[0];
        nnie_worker_s->s_stSsdNnieParam->astSegData[0].astSrc[0].u32Stride = pairs.second->au32Stride[0];

        HI_BOOL bFinish = HI_FALSE;
        SVP_NNIE_HANDLE hSvpNnieHandle = 0;
        s32Ret = HI_MPI_SVP_NNIE_Forward(&hSvpNnieHandle, nnie_worker_s->s_stSsdNnieParam->astSegData[0].astSrc, nnie_worker_s->s_stSsdNnieParam->pstModel,
                                         nnie_worker_s->s_stSsdNnieParam->astSegData[0].astDst,
                                         &nnie_worker_s->s_stSsdNnieParam->astForwardCtrl[0], HI_TRUE);
        if (s32Ret != HI_SUCCESS)
        {
            SAMPLE_PRT("Forward fail,Error(%#x)\n", s32Ret);
            return nullptr;
        }

        if(HI_TRUE)
        {
            while(HI_ERR_SVP_NNIE_QUERY_TIMEOUT == (s32Ret = HI_MPI_SVP_NNIE_Query(nnie_worker_s->s_stSsdNnieParam->astForwardCtrl[0].enNnieId,
                                                                                   hSvpNnieHandle, &bFinish, HI_TRUE)))
            {
                usleep(100);
                SAMPLE_SVP_TRACE(SAMPLE_SVP_ERR_LEVEL_INFO,
                                 "HI_MPI_SVP_NNIE_Query Query timeout!\n");
            }
        }

        bFinish = HI_FALSE;

        (HI_VOID)HI_MPI_SYS_MmzFree(pairs.second->au64PhyAddr[0], (void *) (pairs.second->au64VirAddr[0]));


        delete pairs.second;
        pairs.second = nullptr;

        std::vector<int>min_sizes = {10, 15, 20, 25, 35, 40};
        std::vector<float>aspect_ratio_vec = {1.0, 1.25};
        std::vector<float>variance_list = {0.1, 0.1, 0.2, 0.2};
        float offset = 0.5;
        int step = 8;
        float nms_thres = 0.35;

        SVP_BLOB_S conv_loc_ = nnie_worker_s->s_stSsdNnieParam->astSegData[0].astDst[0];
        SVP_BLOB_S conv_conf_ = nnie_worker_s->s_stSsdNnieParam->astSegData[0].astDst[1];

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
        for(int i = 0 ;i<vbbox.size(); i++)
        {
            if(!vbbox[i].deleted)
            {
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

//        SAMPLE_COMM_VGS_FillRect(pairs.first, &sample_rect_array_s, 0x0000FF00);
//
//        s32Ret = HI_MPI_VENC_SendFrame(0, pairs.first, -1);
//        if (s32Ret != HI_SUCCESS)
//        {
//            SAMPLE_PRT("Venc fail,Error(%#x)\n", s32Ret);
//            return nullptr;
//        }

        printf("channel = %d, detected face = %d\n", pairs.first.first, u16Num);
        HI_MPI_VPSS_ReleaseChnFrame(0, 0, pairs.first.second);
        delete pairs.first.second;
        pairs.first.second = nullptr;
        delete pairs.second;
        pairs.second = nullptr;
    }
    printf("\033[Ive thread return ...  \033[0;39m\n");
    fflush(stdout);
    return (HI_VOID *)HI_SUCCESS;
}

HI_VOID SAMPLE_COMM_VDEC_StartNnie(HI_S32 s32ChnNum, NNIE_WORKER_S *pstNnieWorker, pthread_t *pNnieThread)
{
    pthread_create(&pNnieThread[0], 0, SAMPLE_COMM_VDEC_NNIE, (HI_VOID *)pstNnieWorker);
}