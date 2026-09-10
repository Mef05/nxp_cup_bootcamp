#include "Config.h"
#include "app.h"
#include "board.h"
#include "esc.h"
#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "fsl_device_registers.h"
#include "fsl_pwm.h"
#include "hbridge.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "pixy.h"
#include "servo.h"

#define MAX_VECTORS 10

int main(void) {
    uint16_t vectors[MAX_VECTORS * 4];
    size_t num_vectors;

    /* ===== HARDWARE INIT ===== */
    BOARD_InitHardware();
    BOARD_InitBootPins();
    BOARD_InitBootPeripherals();

    HbridgeInit(&g_hbridge, CTIMER0_PERIPHERAL, CTIMER0_PWM_PERIOD_CH,
                CTIMER0_PWM_1_CHANNEL, CTIMER0_PWM_2_CHANNEL, GPIO0, 24U, GPIO0,
                27U);

    /* Motoarele OPRITE curat la pornire - nu se misca nimic pana nu vedem linia
     */
    HbridgeBrake(&g_hbridge);

    /* Camera Pixy2 pe I2C */
    pixy_t cam1;
    pixy_init(&cam1, LPI2C2, 0x54U, &LP_FLEXCOMM2_RX_Handle,
              &LP_FLEXCOMM2_TX_Handle);
    pixy_set_led(&cam1, 255, 0, 0);

    /* ===== Asteptare de pornire (Timer) ===== */
    PRINTF("Asteptare start %d secunde...\r\n", STARTUP_DELAY_SECONDS);
    for (int i = STARTUP_DELAY_SECONDS; i > 0; i--) {
        PRINTF("%d...\r\n", i);
        SDK_DelayAtLeastUs(1000000, SystemCoreClock);
    }
    PRINTF("START!\r\n");

    /* ===== VARIABILE DE STARE ===== */
    float last_error = 0.0f;
    int frames_lost = 0;
    float current_steer = 0.0f;
    bool line_detected_once = false;
    int frame_count = 0;

    /* ===== PARAMETRI DE TUNING ===== */
    const float KP =
        4.0f; /* Proportional - mult crescut pentru a vira puternic */
    const float KD = 0.5f;         /* Derivative - oprit */
    const float WEIGHT_CTE = 1.0f; /* Cat de mult conteaza pozitia laterala */
    const float WEIGHT_HEADING = 1.0f; /* Unghiul conteaza mult pe curbe mari */
    const float STEER_ALPHA =
        0.7f; /* 70% smoothing - tine volanul stabil dar puternic */

    const float IMAGE_CENTER_X =
        39.0f;                    /* Centrul imaginii Pixy2 Line Tracking */
    const float MIN_DY = 8.0f;    /* Ignora vectori mai plati de 8px pe Y */
    const int PRINT_EVERY_N = 30; /* Printeaza log la fiecare 30 cadre */

    /* ===== BUCLA PRINCIPALA ===== */
    while (1) {
        frame_count++;
        bool do_print = (frame_count % PRINT_EVERY_N == 0);

        status_t pixy_status =
            pixy_get_vectors(&cam1, vectors, MAX_VECTORS, &num_vectors);

        if (pixy_status != kStatus_Success) {
            /* Eroare I2C - oprire de urgenta */
            if (do_print)
                PRINTF("EROARE I2C PIXY2!\r\n");
            HbridgeBrake(&g_hbridge);
            continue;
        }

        float error = 0.0f;

        if (num_vectors > 0) {
            line_detected_once = true;

            /* Dump RAW pentru calibrare (primele ~5 secunde) */
            if (frame_count < 300 && do_print) {
                for (size_t i = 0; i < num_vectors; i++) {
                    PRINTF("RAW[%d]: (%d,%d)->(%d,%d)\r\n", (int)i,
                           (int)vectors[i * 4 + 0], (int)vectors[i * 4 + 1],
                           (int)vectors[i * 4 + 2], (int)vectors[i * 4 + 3]);
                }
            }

            /* --- Clasificare vectori stanga/dreapta --- */
            bool have_left = false, have_right = false;
            float left_bot = 0.0f, left_top = 0.0f;
            float right_bot = 0.0f, right_top = 0.0f;
            float best_left_dist = 1000.0f;
            float best_right_dist = 1000.0f;

            for (size_t i = 0; i < num_vectors; i++) {
                float vx0 = (float)vectors[i * 4 + 0];
                float vy0 = (float)vectors[i * 4 + 1];
                float vx1 = (float)vectors[i * 4 + 2];
                float vy1 = (float)vectors[i * 4 + 3];

                /* Filtram vectorii aproape orizontali */
                float dy = vy0 - vy1;
                if (dy < 0.0f)
                    dy = -dy;
                if (dy < MIN_DY)
                    continue;

                /* Determinam punctul de jos (aproape de masina, Y mare) si cel
                 * de sus */
                float bot_x, top_x;
                if (vy0 > vy1) {
                    bot_x = vx0;
                    top_x = vx1;
                } else {
                    bot_x = vx1;
                    top_x = vx0;
                }

                if (bot_x < IMAGE_CENTER_X) {
                    float dist = IMAGE_CENTER_X - bot_x;
                    if (dist < best_left_dist) {
                        best_left_dist = dist;
                        left_bot = bot_x;
                        left_top = top_x;
                        have_left = true;
                    }
                } else {
                    float dist = bot_x - IMAGE_CENTER_X;
                    if (dist < best_right_dist) {
                        best_right_dist = dist;
                        right_bot = bot_x;
                        right_top = top_x;
                        have_right = true;
                    }
                }
            }

            /* --- Calcul eroare --- */
            float center_bot, center_top;
            const float TRACK_WIDTH_PX =
                45.0f; // Latimea aproximativa a pistei in pixeli

            if (have_left && have_right) {
                center_bot = (left_bot + right_bot) * 0.5f;
                center_top = (left_top + right_top) * 0.5f;
                frames_lost = 0;
            } else if (have_left) {
                center_bot = left_bot + (TRACK_WIDTH_PX * 0.5f);
                center_top = left_top + (TRACK_WIDTH_PX * 0.5f);
                frames_lost = 0;
            } else if (have_right) {
                center_bot = right_bot - (TRACK_WIDTH_PX * 0.5f);
                center_top = right_top - (TRACK_WIDTH_PX * 0.5f);
                frames_lost = 0;
            } else {
                frames_lost++;
                center_bot = IMAGE_CENTER_X; // dummy
                center_top = IMAGE_CENTER_X; // dummy
            }

            if (frames_lost == 0) {
                float cte = center_bot - IMAGE_CENTER_X;
                float heading = center_top - center_bot;
                error = (WEIGHT_CTE * cte) + (WEIGHT_HEADING * heading);

                if (do_print)
                    PRINTF("%s cbot:%d cte:%d hdg:%d err:%d\r\n",
                           (have_left && have_right)
                               ? "L+R"
                               : (have_left ? "L  " : "  R"),
                           (int)center_bot, (int)cte, (int)heading, (int)error);
            }
        } else {
            frames_lost++;
        }

        /* ===== PD CONTROLLER ===== */
        float steer_cmd = (KP * error) + (KD * (error - last_error));
        last_error = error;

        if (steer_cmd > 100.0f)
            steer_cmd = 100.0f;
        if (steer_cmd < -100.0f)
            steer_cmd = -100.0f;

        /* Smoothing exponential (STEER_ALPHA=0 inseamna fara delay) */
        current_steer =
            STEER_ALPHA * current_steer + (1.0f - STEER_ALPHA) * steer_cmd;

        /* ===== VITEZA + FAILSAFE ===== */
        if (!line_detected_once) {
            /* Nu am vazut linia niciodata - stam pe loc */
            HbridgeBrake(&g_hbridge);
            Steer(0.0f);
            current_steer = 0.0f;
            if (do_print)
                PRINTF("Astept linia...\r\n");
        } else if (frames_lost >= 10) {
            /* Linie pierduta de prea mult timp - stop total */
            HbridgeBrake(&g_hbridge);
            Steer(0.0f);
            current_steer = 0.0f;
            if (do_print)
                PRINTF("LINIE PIERDUTA! Stop.\r\n");
        } else if (frames_lost > 0) {
            /* Pierdut temporar - dam cu spatele ca sa regasim linia! */
            Steer(current_steer);
            HbridgeSpeed(&g_hbridge, -40, -40);
        } else {
            /* Conducere normala cu Diferential Electronic */
            Steer(current_steer);
            
            int speed_L = SPEED_LEFT;
            int speed_R = SPEED_RIGHT;

            // Diferential Electronic: reducem viteza rotii interioare pe curba
            if (current_steer > 0.0f) {
                float diff_factor = 1.0f - (current_steer * 0.005f);
                speed_R = (int)((float)SPEED_RIGHT * diff_factor);
            } else if (current_steer < 0.0f) {
                float diff_factor = 1.0f - (-current_steer * 0.005f);
                speed_L = (int)((float)SPEED_LEFT * diff_factor);
            }

            HbridgeSpeed(&g_hbridge, speed_L, speed_R);

            if (do_print)
                PRINTF("STR:%d SPD_L:%d SPD_R:%d\r\n", (int)current_steer, speed_L, speed_R);
        }
    }
}
