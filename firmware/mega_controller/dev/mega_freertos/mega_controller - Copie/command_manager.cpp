#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "command_manager.h"
#include "servo_controller.h"
#include "uart_handler.h"

extern QueueHandle_t frameQueue;

// Stub pour remplacer l'implémentation moteur
void motor_set_velocity(int16_t v_l, int16_t v_r) {
    Serial.print("motor_set_velocity: v_l=");
    Serial.print(v_l);
    Serial.print(" v_r=");
    Serial.println(v_r);
    // TODO: remplacer par une Task dédiée ou pilote direct des PWM
}

void command_manager_init() {
    Serial.println("command_manager_init");
}

void command_manager_task(void *pvParameters) {
    Serial.println("command_manager_task started");
    CommandFrame_t frame;
    for (;;) {
        if (xQueueReceive(frameQueue, &frame, portMAX_DELAY) == pdTRUE) {
            Serial.print("command_manager: Frame reçue func=0x");
            Serial.print(frame.func, HEX);
            Serial.print(", len=");
            Serial.println(frame.length);

            switch (frame.func) {
                case 0x10: {  // MOVE
                    if (frame.length >= 4) {
                        int16_t v_l = (frame.payload[0] << 8) | frame.payload[1];
                        int16_t v_r = (frame.payload[2] << 8) | frame.payload[3];
                        Serial.print("command_manager: MOVE v_l=");
                        Serial.print(v_l);
                        Serial.print(" v_r=");
                        Serial.println(v_r);
                        motor_set_velocity(v_l, v_r);
                    }
                    break;
                }
                case 0x11:  // OPEN_GRIPPER
                    Serial.println("command_manager: OPEN_GRIPPER");
                    servo_open();
                    break;
                case 0x12:  // CLOSE_GRIPPER
                    Serial.println("command_manager: CLOSE_GRIPPER");
                    servo_close();
                    break;
                case 0x20:  // PING
                    Serial.println("command_manager: PING reçu");
                    uart_send_pong();
                    break;
                default:
                    Serial.print("command_manager: Inconnu func=0x");
                    Serial.println(frame.func, HEX);
                    break;
            }
        }
    }
}
