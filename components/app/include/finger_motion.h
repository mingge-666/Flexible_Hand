#ifndef FINGER_MOTION_H
#define FINGER_MOTION_H

#include "app_finger.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>
#include "finger_config.h"

void finger_approach_step(finger_t *f, int idx, float dt);
void finger_catch_step(finger_t *f, int idx, float dt);
void finger_hold_step(finger_t *f, int idx, float dt);
void finger_move_step(finger_t *f, float dt);





#endif
