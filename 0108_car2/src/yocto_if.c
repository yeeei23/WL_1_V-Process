#include "yocto_if.h"
#include "i2c_io.h"
#include "queue.h"
#include "val_msg.h"
#include "proto_wl1.h"
#include "proto_wl2.h"
#include "proto_wl3.h"
#include "proto_wl4.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <string.h>
#include "debug.h"
#include "driving_mgr.h"
#include "gps.h"


#define I2C_BUS_YOCTO  "/dev/i2c-1" 
#define YOCTO_ADDR      0x42        

extern queue_t q_wl_sec;     // 수신 파이프라인 시작점
extern queue_t q_yocto_pkt_tx; // 송신 파이프라인 시작점

extern driving_status_t g_driving_status;

extern queue_t q_val_pkt_tx, q_val_yocto, q_rx_sec_rx;
extern volatile bool g_keep_running;
extern uint32_t g_sender_id;
extern queue_t q_pkt_val;
extern queue_t q_val_yocto;
extern queue_t q_yocto_to_driving;

// 시간 측정을 위한 유틸리티 함수
static uint64_t get_now_us() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000 + tv.tv_usec;
}
// 정밀 주기 제어 함수
static void wait_next_period(struct timespec *next, long ms) {
    next->tv_nsec += ms * 1000000L;
    while (next->tv_nsec >= 1000000000L) {
        next->tv_sec++;
        next->tv_nsec -= 1000000000L;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, next, NULL);
}



void* thread_yocto_if(void* arg) {
    (void)arg;
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);
    uint32_t loop_cnt = 0;
    const uint32_t wl4_trigger = (PERIOD_WL4_S * 1000) / PERIOD_YOCTO_MS;

    DBG_INFO("Thread 9: Yocto Interface Module started.");
while (g_keep_running) {
        // 정밀 주기 제어 (예: 100ms)
        wait_next_period(&next_time, PERIOD_YOCTO_MS);

        uint8_t rx_buf[128] = {0};
        int rx_len = 0;

#ifdef SIMULATION
        /*
        // --- 시뮬레이션 데이터 생성 ---
        if (loop_cnt > 0 && loop_cnt % 100 == 0) { // 사고(WL-3)
            rx_buf[0] = TYPE_WL3;
            uint64_t aid = 2000 + loop_cnt;
            memcpy(&rx_buf[15], &aid, 8); 
            rx_len = 23;
        } else if (loop_cnt % wl4_trigger == 0) { // 주행(WL-4)
            rx_buf[0] = TYPE_WL4;
            uint16_t hdg = 180; int32_t alt = 150;
            memcpy(&rx_buf[1], &hdg, 2); memcpy(&rx_buf[3], &alt, 4);
            rx_len = 17;
        }
        */
#else
        // --- 실제 하드웨어 수신 (i2c_io.c의 함수 사용) ---
        // I2C_BUS_YOCTO는 "/dev/i2c-1", YOCTO_ADDR은 상대방 주소
        rx_len = I2C_read_data(I2C_BUS_YOCTO, YOCTO_ADDR, rx_buf, sizeof(rx_buf));
#endif

        // [1] 데이터 수신 처리
        if (rx_len > 0) {
            if (rx_buf[0] == TYPE_WL3) {
                wl3_packet_t *wl3 = malloc(sizeof(wl3_packet_t));
                if (wl3) {
                    memcpy(wl3, rx_buf, sizeof(wl3_packet_t));
                    Q_push(&q_pkt_val, wl3);
                    //printf("\x1b[31m[T9-RX] WL-3 High-Priority (AID:0x%lX)\x1b[0m\n", wl3->accident_id);
                }
            } else if (rx_buf[0] == TYPE_WL4) {
                wl4_packet_t *wl4 = malloc(sizeof(wl4_packet_t));
                if (wl4) {
                    memcpy(wl4, rx_buf, sizeof(wl4_packet_t));
                    Q_push(&q_yocto_to_driving, wl4);
                    //printf("\x1b[34m[T9-RX] WL-4 Periodic Update\x1b[0m\n");
                }
            }
        }

        // [2] 데이터 송신 처리 (WL-2 결과 전송)
        wl2_packet_t *wl2 = (wl2_packet_t *)Q_pop_nowait(&q_val_yocto);
        if (wl2) {
#ifdef SIMULATION
            /*uint16_t distance = (wl2->dist_rsv >> 4); 
            printf("\x1b[35m[T9-TX-SIM] WL-2 Result -> Dist: %dm, Lane: %d\x1b[0m\n", distance, wl2->lane);
            */
#else
            // 실제 I2C 전송 (i2c_io.c의 함수 사용)
            int tx_res = I2C_write_data(I2C_BUS_YOCTO, YOCTO_ADDR, (uint8_t*)wl2, sizeof(wl2_packet_t));
            if (tx_res > 0) {
                //printf("\x1b[35m[T9-TX-HW] WL-2 Result Sent to Yocto\x1b[0m\n");
            }
#endif
            free(wl2); // 송신 후 메모리 해제
        }
        loop_cnt++;
    }
    return NULL;
}