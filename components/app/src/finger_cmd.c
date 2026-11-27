#include "finger_cmd.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "app_finger.h"
#include "uart.h"
#include <string.h>


/* ---------- 命令解析 ---------- */

/**
 * @brief 从 UART0 驱动读取一行（CR 或 LF 结束），自动忽略空行。
 */
static void read_line(char *buf, size_t size)
{
    size_t n = 0;

    for (;;) {
        uint8_t c = 0;
        int r = uart_read_bytes(USART_UX, &c, 1, portMAX_DELAY);
        if (r != 1) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            if (n == 0) {
                continue;    /* 忽略空行以及 CRLF 的第二个字符 */
            }
            buf[n] = '\0';
            return;
        }
        if (n + 1 < size) {
            buf[n++] = (char)c;   /* 超长部分丢弃，直到换行 */
        }
    }
}

void finger_cmd_task(void *arg)//读取指令来控制舵机
{
    (void)arg;

    char line[FINGER_CMD_LINE_LEN];

    printf("Finger control ready.\n");
    printf("  approach <openness 0-180> [finger] : 0=open, 180=closed; contact -> stop\n");
    printf("  catch    <force 0-%.0f> [finger]  : PID force grasp (after approached)\n",
           (double)FINGER_MAX_FORCE);
    printf("  move     <openness 0-180> [finger] : direct position move\n");
    printf("  set_accmode <0-4>                  : motion speed/accel preset\n");
    printf("  stop                               : hold current position\n");
    fflush(stdout);

    for (;;) {
        read_line(line, sizeof(line));

        int target_finger = 0;
        char cmd[16];
        float val = 0.0f;

        if (sscanf(line, "%15s %f %d", cmd, &val, &target_finger) == 3) {
            /* <cmd> <value> <finger> */
            if (!isfinite(val)) {
                printf("invalid value (nan/inf)\n");
            }
            else if (target_finger < FINGER_1 || target_finger > FINGER_COUNT) {
                printf("Invalid finger id: %d (valid 1-%d)\n",
                       target_finger, FINGER_COUNT);
            }
            else if (strcmp(cmd, "approach") == 0) {
                app_onefinger_approach(val, (finger_id_t)target_finger);
            }
            else if (strcmp(cmd, "catch") == 0) {
                app_onefinger_catch(val, (finger_id_t)target_finger);
            }
            else if (strcmp(cmd, "move") == 0) {
                app_onefinger_move(val, (finger_id_t)target_finger);
            }
            else if (strcmp(cmd, "set_accmode") == 0) {
                printf("set_accmode does not take a finger\n");
            }
            else {
                printf("unknown command '%s'\n", cmd);
            }
        }
        else if (sscanf(line, "%15s %f", cmd, &val) == 2) {
            /* <cmd> <value>：作用于全部手指 */
            if (!isfinite(val)) {
                printf("invalid value (nan/inf)\n");
            }
            else if (strcmp(cmd, "approach") == 0) {
                app_finger_approach(val);
            }
            else if (strcmp(cmd, "catch") == 0) {
                app_finger_catch(val);
            }
            else if (strcmp(cmd, "move") == 0) {
                app_finger_move(val);
            }
            else if (strcmp(cmd, "set_accmode") == 0) {
                if (val < 0 || val > 4 || val != (float)(int)val) {
                    printf("Invalid finger acceleration mode: %.1f\n", (double)val);
                } else {
                    for (int i = 0; i < FINGER_COUNT; ++i) {
                        app_finger_set_accmode((finger_id_t)(i + 1),
                                               (finger_accel_mode_t)(int)val);
                    }
                    printf("Set finger acceleration mode to: %.1f\n", (double)val);
                }
            }
            else {
                printf("unknown command '%s'\n", cmd);
            }
        }
        else if (sscanf(line, "%15s", cmd) == 1) {
            if (strcmp(cmd, "stop") == 0) {
                app_finger_stop();
            }
            else if (strcmp(cmd, "yee") == 0 ||
                     strcmp(cmd, "ok") == 0 ||
                     strcmp(cmd, "张开") == 0) {
                printf("gesture '%s' not implemented yet\n", cmd);
            }
            else {
                printf("unknown command '%s'\n", cmd);
            }
        }
        else {
            printf("usage: approach <openness> [finger] | catch <force> [finger] | "
                   "move <openness> [finger] | set_accmode <0-4> | stop\n");
        }
        fflush(stdout);
    }
}


