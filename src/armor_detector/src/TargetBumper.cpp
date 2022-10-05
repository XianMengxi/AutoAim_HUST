#include "TargetBumper.h"
namespace ly
{
    TargetBumper::TargetBumper()
    {
        target_loss_count = 0;
    }

    TargetBumper::~TargetBumper()
    {
    }

    float TargetBumper::getDistanceToCenter(const ArmorBlob &blob)
    {
        const cv::Point2f center_point = cv::Point2f(640, 512);
        cv::Point2f armor_blob_center = cv::Point2f(blob.rect.x + blob.rect.width / 2, blob.rect.y + blob.rect.height / 2.0);
        cv::Point2f dif = armor_blob_center - center_point;
        return dif.x * dif.x + dif.y * dif.y;
    }
    bool TargetBumper::compFunc(const ArmorBlob &blob1, const ArmorBlob &blob2)
    {
        return getDistanceToCenter(blob1) > getDistanceToCenter(blob2);
    }
    void TargetBumper::getTargetOrder(ArmorBlobs &blobs)
    {
        sort(blobs.begin(), blobs.end(), compFunc);
    }

    //新的状态判断
    //在没有获得目标的前提之下,此函数为无条件选择高优先级的,如果wanted_id 不选择的话，优先高优先级目标
    ArmorBlob TargetBumper::getAimTarget(ArmorBlobs &blobs, int wanted_id, int not_wanted_id)
    {
        ArmorBlobs shoot_target_candidate;
        int now_priority = 5;
        for (int i = 0; i < blobs.size(); i++)
        {
            if (getPriority(blobs[i]._class, wanted_id, not_wanted_id) < now_priority) //有优先级更高的目标
            {
                now_priority = getPriority(blobs[i]._class, wanted_id, not_wanted_id);
                shoot_target_candidate.clear();
                shoot_target_candidate.emplace_back(blobs[i]);
            }
            else if (getPriority(blobs[i]._class, wanted_id, not_wanted_id) == now_priority) //有相同优先级的目标
            {
                shoot_target_candidate.emplace_back(blobs[i]);
            }
        }

        getTargetOrder(shoot_target_candidate);

        return shoot_target_candidate[0];
    }
    int TargetBumper::getPriority(int id, int wanted_id, int not_wanted_id)
    {
        if (id == wanted_id)
        {
            return 0;
        }
        else if (id == not_wanted_id)
        {
            return 5;
        }
        else if (id == Hero || id == Sentry)
        {
            return 1;
        }
        else if (id == Infantry1 || id == Infantry2 || id == Infantry3)
        {
            return 2;
        }
        else if (id == Outpost || id == Engineer)
        {
            return 3;
        }
        else if (id == Base)
        {
            return 4;
        }
        else
        {
            return 5;
        }
    }

    //获得对应ID的装甲板
    bool TargetBumper::getIDTarget(ArmorBlobs &blobs, int id)
    {
        for (int i = 0; i < blobs.size(); i++)
        {
            if (blobs[i]._class != id)
            {
                blobs.erase(blobs.begin() + i);
                i--;
            }
        }
        return blobs.size() > 0;
    }

};