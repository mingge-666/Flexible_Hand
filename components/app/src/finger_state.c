#include "finger_state.h"

#include "finger_force.h"
#include "finger_motion.h"

/**
 * @brief 单根手指按当前状态推进一步（控制任务周期调用）。
 */
void finger_update(finger_t *f, int idx, float dt)
{
    if (f == NULL) {
        return;
    }

    /* 所有状态都刷新当前力值，打印不会停在旧值 */
    f->cur_force = finger_read_force(idx);

    switch (f->state) {
    case FINGER_APPROACHING:
        finger_approach_step(f, idx, dt);
        break;
    case FINGER_CATCHING:
        finger_catch_step(f, idx, dt);
        break;
    case FINGER_CATCHED:
        finger_hold_step(f, idx, dt);   /* 继续保力，掉力则重回 CATCHING */
        break;
    case FINGER_MOVING:
        finger_move_step(f, dt);
        break;
    case FINGER_STATE_IDLE:
    case FINGER_APPROACHED:
    default:
        break;   /* 保持当前位置，等待命令 */
    }
}


