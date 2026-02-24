/* File: square_demo.c */

#include "square_demo.h"
#include "speed_controller.h"
#include "odometry.h"
#include "cmsis_os2.h"
#include <math.h>
#include <stdbool.h>

#define LINEAR_SPEED 0.20f       /* m/s */
#define ANGULAR_SPEED (M_PI / 4) /* rad/s (45°/s) */
#define SIDE_LENGTH 0.30f        /* m */
#define TURN_ANGLE (M_PI / 2)    /* rad (90°) */

#define POS_TOL 0.005f /* tolérance position (m) */
#define ANG_TOL 0.02f  /* tolérance angle (rad) */

#define CTRL_PERIOD_MS 20 /* 50 Hz */

/* --- normalise un angle dans [-π, +π] --- */
static float normalize_angle(float a)
{
    while (a > M_PI)
        a -= 2 * M_PI;
    while (a < -M_PI)
        a += 2 * M_PI;
    return a;
}

/* --- renvoie true si a et b sont à ±ANG_TOL modulo 2π --- */
static bool angle_reached(float a, float b)
{
    float d = normalize_angle(a - b);
    return fabsf(d) < ANG_TOL;
}

/* --- pilotage linéaire sur la distance dist à speed --- */
static void drive_distance(float dist, float speed)
{
    pose_t p0;
    Odom_GetPose(&p0);
    float x0 = p0.x, y0 = p0.y;
    float target2 = dist * dist;

    for (;;)
    {
        pose_t p;
        Odom_GetPose(&p);
        float dx = p.x - x0;
        float dy = p.y - y0;
        if (dx * dx + dy * dy >= target2 - POS_TOL * POS_TOL)
            break;

        SpeedController_SetReference(speed, speed);
        osDelay(CTRL_PERIOD_MS);
    }

    SpeedController_SetReference(0, 0);
    osDelay(200);
}

/* --- rotation sur place de angle à vitesse w (positive = A-horaire) --- */
static void rotate_angle(float angle, float w)
{
    pose_t p0;
    Odom_GetPose(&p0);
    float theta0 = p0.theta;
    float target = normalize_angle(theta0 + angle);

    for (;;)
    {
        Odom_GetPose(&p0);
        if (angle_reached(p0.theta, target))
            break;

        /* roue gauche recule, droite avance */
        SpeedController_SetReference(-w, +w);
        osDelay(CTRL_PERIOD_MS);
    }

    SpeedController_SetReference(0, 0);
    osDelay(200);
}

static void Square_Task(void *arg)
{
    /* 1) reset et init */
    Odom_ResetPose(0.0f, 0.0f, 0.0f);
    SpeedController_Init();
    /* facultatif : logs PID toutes les 100 ms */
    SpeedController_StartUartLogger(100);

    osDelay(500); /* laisse le temps au PID de démarrer */

    /* 2) 4 côtés */
    for (int i = 0; i < 4; i++)
    {
        drive_distance(SIDE_LENGTH, LINEAR_SPEED);
        rotate_angle(TURN_ANGLE, ANGULAR_SPEED);
    }

    /* 3) stop final */
    SpeedController_SetReference(0, 0);
    for (;;)
        osDelay(1000);
}

void start_square_demo(void)
{
    const osThreadAttr_t attr = {
        .name = "SquareDemo",
        .priority = osPriorityNormal,
        .stack_size = 512};
    osThreadNew(Square_Task, NULL, &attr);
}
