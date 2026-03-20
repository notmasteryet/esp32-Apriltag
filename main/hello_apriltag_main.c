#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <stdio.h>
#include "esp_log.h"

//esp32-camera
#include "esp_camera.h"

// apriltag
#include "apriltag.h"
#include "apriltag_pose.h"
#include "tag36h11.h"
#include "common/image_u8.h"
#include "common/zarray.h"

static const char* TAG = "camera";

const float APRIL_TAG_DEFAULT_SIZE = 0.15;
/*
    Do not use sizes above QVGA when not JPEG
    FRAMESIZE_QQVGA,    // 160x120
    FRAMESIZE_QCIF,     // 176x144
    FRAMESIZE_HQVGA,    // 240x176
    FRAMESIZE_240X240,  // 240x240
    FRAMESIZE_QVGA,     // 320x240
*/
const pixformat_t PIX_FORMAT = FRAMESIZE_HQVGA;
const int CAMERA_SENSOR_WIDTH = 240;
const int CAMERA_SENSOR_HEIGHT = 176;

const int CAMERA_CENTER_X = CAMERA_SENSOR_WIDTH / 2;
const int CAMERA_CENTER_Y = CAMERA_SENSOR_HEIGHT / 2;
// FOV = 66 deg -> tan of half = 65/100
const int CAMERA_FOCUS_X = CAMERA_SENSOR_WIDTH / 2 * 100 / 65;
const int CAMERA_FOCUS_Y = CAMERA_SENSOR_HEIGHT / 2 * 100 / 65;

//M5STACK_CAM PIN Map
#define CAM_PIN_PWDN    -1 //power down is not used
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    21
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27

#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      19
#define CAM_PIN_D2      18
#define CAM_PIN_D1      5
#define CAM_PIN_D0      4

#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

#define CAM_XCLK_FREQ   20000000

static camera_config_t camera_config = {
    .pin_pwdn  = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_GRAYSCALE,//YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = PIX_FORMAT,
    // .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1 //if more than one, i2s runs in continuous mode.
};


void app_main()
{
    vTaskDelay(250 / portTICK_PERIOD_MS);
    apriltag_family_t *tf = tag36h11_create();

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_sigma = 0.0;
    td->quad_decimate = 1.0;
    td->refine_edges = 0;
    td->decode_sharpening = 0;
    td->nthreads = 1;
    td->debug = 0;
    vTaskDelay(250 / portTICK_PERIOD_MS);

    apriltag_detection_info_t info;
    info.tagsize = APRIL_TAG_DEFAULT_SIZE;
    info.fx = CAMERA_FOCUS_X;
    info.fy = CAMERA_FOCUS_Y;
    info.cx = CAMERA_CENTER_X;
    info.cy = CAMERA_CENTER_Y;

    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);

    while(1){
        //acquire a frame
        camera_fb_t * fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera Capture Failed");
            return;
        }

        image_u8_t im = {
            .width = fb->width,
            .height = fb->height,
            .stride = fb->width,
            .buf = fb->buf
        };

        zarray_t *detections = apriltag_detector_detect(td, &im);

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            printf("TAG_FOUND: %d;",det->id);

            apriltag_pose_t pose;
            info.det = det;
            float err = estimate_tag_pose(&info, &pose);
            printf("%f,%f,%f,%f,%f,%f,%f,%f,%f;",
                pose.R->data[0],pose.R->data[1],pose.R->data[2],
                pose.R->data[3],pose.R->data[4],pose.R->data[5],
                pose.R->data[6],pose.R->data[7],pose.R->data[8]);
            printf("%f,%f,%f;",
                pose.t->data[0],pose.t->data[1],pose.t->data[2]);
            matd_destroy(pose.R);
            matd_destroy(pose.t);
            printf("%f\n", err);
        }

        apriltag_detections_destroy(detections);

        float t =  timeprofile_total_utime(td->tp) / 1.0E3f;
        printf("%12.3f \n", t);

        //return the frame buffer back to the driver for reuse
        esp_camera_fb_return(fb);

    }

    // don't deallocate contents of inputs; those are the argv
    apriltag_detector_destroy(td);

    tag36h11_destroy(tf);

}
