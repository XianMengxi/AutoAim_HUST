#ifndef _TARGETBUMPER_H
#define _TARGETBUMPER_H
#include "ArmorFinder.h"
#include <eigen3/Eigen/Core>
#include "EnemyType.h"
#include "sophus/se3.h"
#include <string>
/*装甲板缓冲器，用来判断卡尔曼滤波重启逻辑以及切换装甲板逻辑*/
namespace ly
{
#define MAX_LOSS_BUMP_COUNT 25
#define MAX_DETECT_BUMP_COUNT 5
    enum TARGET_MODE
    {
        NOT_GET_TARGET = 0,       //未获得目标
        CONTINOUS_GET_TARGET = 1, //连续获得目标
        LOST_BUMP = 2,            //缓冲阶段
        DETECT_BUMP = 3           //进入连续识别状态的缓冲
    };
    class TargetBumper
    {
    private:
        int target_loss_count = 0;
        const string target_mode_str[4] = {"NOT_GET_TARGET", "CONTINOUS_GET_TARGET", "LOST_BUMP", "DETECT_BUMP"};

        static float getDistanceToCenter(const ArmorBlob &blob);
        static bool compFunc(const ArmorBlob &blob1, const ArmorBlob &blob2);
        void getTargetOrder(ArmorBlobs &blobs);

    public:
        TargetBumper();
        ~TargetBumper();

        void printMode(int mode)
        {
            if (mode == NOT_GET_TARGET)
            {
                LOG(ERROR) << "mode:" << target_mode_str[mode];
            }
            else if (mode == CONTINOUS_GET_TARGET)
            {
                LOG(INFO) << "mode:" << target_mode_str[mode];
            }
            else
            {
                LOG(WARNING) << "mode:" << target_mode_str[mode];
            }
        }

        //缓冲
        bool lossAndCHeck(int mode) //两种缓冲形式
        {
            if (mode == LOST_BUMP)
            {
                target_loss_count++;
                if (target_loss_count >= MAX_LOSS_BUMP_COUNT) //丢帧数过多
                {
                    targetLossEmpty();
                    return false;
                }
                return true;
            }
            else if (mode == DETECT_BUMP)
            {
                target_loss_count++;
                if (target_loss_count >= MAX_DETECT_BUMP_COUNT) //丢帧数过多
                {
                    targetLossEmpty();
                    return false;
                }
                return true;
            }
            return false;
        }
        void targetLossEmpty()
        {
            target_loss_count = 0;
        }

        //新的判断体系
        ArmorBlob getAimTarget(ArmorBlobs &blobs, int wanted_id = -1, int not_wanted_id = -1);
        int getPriority(int id, int wanted_id, int not_wanted_id);

        bool getIDTarget(ArmorBlobs &blobs, int id);
    };
} // namespace ly

#endif