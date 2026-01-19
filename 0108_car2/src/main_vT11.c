#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>

#include "debug.h"
#include "queue.h"
#include "wl.h"
#include "pkt.h"
#include "sec.h"
#include "gps.h"
#include "val.h"
#include "yocto_if.h" 
#include "driving_mgr.h"
#include "i2c_io.h"


extern void *thread_rx(void *arg);       // T1
extern void *thread_sec_rx(void *arg);   // T2

extern void *thread_val(void *arg);      // T4
extern void *sub_thread_pkt_tx(void *arg);   
extern void *sub_thread_pkt_rx(void *arg);   
extern void *thread_sec_tx(void *arg);   // T7
extern void *thread_yocto_if(void *arg);
extern void *thread_driving_manager(void *arg);
extern void *thread_tx(void *arg);
extern void *thread_gps(void *arg); 
// ==========================================
// 1. 전역 상태 및 큐 정의 (vT11 파이프라인)
// ==========================================
volatile bool g_keep_running = true;
uint32_t g_sender_id;
//wl_ctx_t g_wl_tx_ctx;


// 차량 및 주행 상태 (T5 주행관리 스레드용)
driving_status_t g_driving_status = {
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .lat = 0.0, .lon = 0.0, .alt = 0, .heading = 0
};


queue_t q_rx_sec_rx;    // T1 -> T2
queue_t q_sec_rx_pkt;   // T2 -> T3
queue_t q_val_pkt_tx;   // T4 -> T6
queue_t q_pkt_val;
queue_t q_pkt_sec_tx;   // T6 -> T7
queue_t q_sec_tx_wl_tx; // T7 -> T8
queue_t q_val_yocto;      // T4 -> T9
queue_t q_yocto_to_driving; // T9 -> T5
queue_t q_yocto_pkt_tx;
queue_t q_wl_sec;
queue_t q_yocto_if_to_pkt_tx;
// ==========================================
// 2. 종료 및 초기화 로직
// ==========================================
void signal_handler(int sig) {
    (void)sig;
    g_keep_running = false;
    
    // 블로킹 상태의 큐들을 깨우기 위해 NULL 푸시
    Q_push(&q_rx_sec_rx, NULL); Q_push(&q_sec_rx_pkt, NULL);
    Q_push(&q_pkt_val, NULL); Q_push(&q_val_pkt_tx, NULL);
    Q_push(&q_pkt_sec_tx, NULL); Q_push(&q_sec_tx_wl_tx, NULL);
    Q_push(&q_val_yocto, NULL); Q_push(&q_yocto_to_driving, NULL);
    Q_push(&q_yocto_if_to_pkt_tx, NULL);
    
    printf("\n[MAIN] Shutdown signal received. Cleaning up...\n");
}



// ==========================================
// 4. 메인 진입점
// ==========================================
int main(int argc, char *argv[]) {
    // 1. 초기화 및 시그널 설정
    signal(SIGINT, signal_handler);
    g_sender_id = (argc > 1) ? (uint32_t)strtol(argv[1], NULL, 16) : 0x2222;
    
    /*Q_init(&q_rx_sec); 
    Q_init(&q_sec_pkt); Q_init(&q_pkt_val);
    Q_init(&q_val_pkt); Q_init(&q_pkt_sec); Q_init(&q_sec_tx);
    Q_init(&q_val_yocto); Q_init(&q_yocto_to_driving);*/
    //큐 초기화 
    Q_init(&q_rx_sec_rx);
    Q_init(&q_sec_rx_pkt);
    Q_init(&q_val_pkt_tx);
    Q_init(&q_pkt_sec_tx);
    Q_init(&q_sec_tx_wl_tx);
    Q_init(&q_val_yocto);
    Q_init(&q_yocto_to_driving);

    
    // 3. 하드웨어 초기화
    GPS_init();
    // I2C 디바이스들 초기화 (프로그램 시작 시 한 번)
    if (I2C_Display_Open_And_Init() < 0) {
        //printf("Failed to initialize I2C displays.\n");
    }
    //WL_init(&g_wl_tx_ctx, true);

    printf("--- Integrated V2X System (vT11) Start ---\n");
    
    // 4. 전처리기 사용 방식 수정 (함수 밖으로 분리)
//#ifdef SIMULATION
    //printf("[MAIN] Mode: SIMULATION\n");
//#else
    //printf("[MAIN] Mode: HARDWARE\n");
//#endif
    // 2. 스레드 생성
    pthread_t ths[10];
    
// 1. RX 파이프라인 (T1 ~ T4)
    pthread_create(&ths[0], NULL, thread_rx, NULL);           // T1: Wireless RX
    pthread_create(&ths[1], NULL, thread_sec_rx, NULL);       // T2: Security RX
    pthread_create(&ths[2], NULL, sub_thread_pkt_rx, NULL);       // T3: Packet RX
    pthread_create(&ths[3], NULL, thread_val, NULL);          // T4: Valuation (판단)

    // 2. 주행 데이터 통합 관리 (T5) 
    pthread_create(&ths[4], NULL, thread_driving_manager, NULL); // T5: Driving Manager

    // 3. TX 파이프라인 (T6 ~ T8)
    pthread_create(&ths[5], NULL, sub_thread_pkt_tx, NULL);       // T6: Packet TX
    pthread_create(&ths[6], NULL, thread_sec_tx, NULL);       // T7: Security TX
    
    pthread_create(&ths[7], NULL, thread_tx, NULL);   // T8: Wireless TX

    // 4. 외부 센서 인터페이스 (T9)
    pthread_create(&ths[8], NULL, thread_yocto_if, NULL);     // T9: Yocto I2C
    pthread_create(&ths[9], NULL, thread_gps, NULL);
    
    
    // 3. 메인 모니터링 루프
    
    
    while (g_keep_running) {
        
        sleep(1);
    }

    // 4. 종료 및 자원 해제
    for (int i = 0; i < 10; i++) {
        pthread_join(ths[i], NULL);
    }

    pthread_mutex_destroy(&g_driving_status.lock);
    printf("[MAIN] V2X System Gracefully Terminated.\n");

    return 0;
}
    
