#include "gps.h"
#include "val.h"
#include "debug.h"
#include <unistd.h>
#include <math.h>
#include "driving_mgr.h"

extern volatile bool g_keep_running;
extern driving_status_t g_driving_status;

// [1] GPS 초기화 함수 구현
void GPS_init(void) {
    DBG_INFO("GPS: Mock Mode Initialized.");
}

// [2] 최신 데이터 반환 함수 구현 
    gps_data_t GPS_get_latest(void) {
    gps_data_t data;
    
    // 안전하게 현재 전역 상태에 업데이트된 값을 가져옵니다.
    pthread_mutex_lock(&g_driving_status.lock);
    data.lat = g_driving_status.lat;
    data.lon = g_driving_status.lon;
    data.alt = (double)g_driving_status.alt;
    pthread_mutex_unlock(&g_driving_status.lock);
    
    return data;
}

/**
 * GPS 정보를 수집하여 전역 차량 상태를 업데이트하는 스레드
 */
void *thread_gps(void *arg) {
    DBG_INFO("Thread 7: GPS Client Module started.");

    // [고정값 설정] 차2 기준
    double current_lat = 37.5654; // (사고차와 논리적 거리 약 400m) 
    double current_lon = 126.9780;
    double current_alt = 15.0; // 고도 기본값

    while (g_keep_running) {
       
        pthread_mutex_lock(&g_driving_status.lock);
        g_driving_status.lat = current_lat;
        g_driving_status.lon = current_lon;
        g_driving_status.alt = current_alt; 
        pthread_mutex_unlock(&g_driving_status.lock);

        // 너무 잦은 업데이트 방지 (10Hz: 100ms)
        usleep(1000000); 
       
        
        // 10초마다 로그 출력
        static int count = 0;
        if (++count % 100 == 0) {
            printf("[GPS-FIXED] 차2 위치: Lat %.6f, Lon %.6f, Alt %.1f\n", 
                    current_lat, current_lon, current_alt);
    }

    DBG_INFO("Thread 7: GPS Client Module terminating.");
    return NULL;
    }
}
