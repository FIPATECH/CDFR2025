#ifndef I2C_HANDLER_H
#define I2C_HANDLER_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define I2C_SLAVE_ADDR (0x08U) // adresse 7 bits de l’Arduino
#define I2C_TIMEOUT (1000U)    // timeout en ms

    /**
     * @brief  Initialise la partie I²C (bus-recovery éventuel).
     *         MX_I2C3_Init() a déjà été appelé dans main.c.
     */
    void I2C_Handler_Init(void);

    /**
     * @brief  Envoie une commande texte via I²C.
     * @param  cmd  chaîne C null-terminée, ex. "banner_up\n"
     * @retval HAL_OK ou code d’erreur HAL
     */
    HAL_StatusTypeDef I2C_SendCommand(const char *cmd);

    /* Commandes haut-niveau vers l’Arduino */
    static inline HAL_StatusTypeDef I2C_Cmd_BannerUp(void) { return I2C_SendCommand("banner_up\n"); }
    static inline HAL_StatusTypeDef I2C_Cmd_BannerDown(void) { return I2C_SendCommand("banner_down\n"); }
    static inline HAL_StatusTypeDef I2C_Cmd_OpenRack(void) { return I2C_SendCommand("open_rack\n"); }
    static inline HAL_StatusTypeDef I2C_Cmd_CloseRack(void) { return I2C_SendCommand("close_rack\n"); }
    static inline HAL_StatusTypeDef I2C_Cmd_PlatformUp(void) { return I2C_SendCommand("platform_up\n"); }
    static inline HAL_StatusTypeDef I2C_Cmd_PlatformDown(void) { return I2C_SendCommand("platform_down\n"); }
    static inline HAL_StatusTypeDef I2C_Cmd_TestX(void) { return I2C_SendCommand("test_x\n"); }
    static inline HAL_StatusTypeDef I2C_Cmd_TestY(void) { return I2C_SendCommand("test_y\n"); }
    static inline HAL_StatusTypeDef I2C_Cmd_TestZ(void) { return I2C_SendCommand("test_z\n"); }
    static inline HAL_StatusTypeDef I2C_Cmd_TestXY(void) { return I2C_SendCommand("test_xy\n"); }

#ifdef __cplusplus
}
#endif

#endif /* I2C_HANDLER_H */
