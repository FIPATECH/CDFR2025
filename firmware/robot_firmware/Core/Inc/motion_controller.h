#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Initialise le contrôleur de mouvement : crée la tâche Motion_Task.
     */
    void Motion_Init(void);

    /**
     * @brief Met à jour la commande (vitesse linéaire, vitesse angulaire).
     *        Appelé depuis l'ISR UART dès qu'un nouveau /cmd_vel arrive.
     */
    void Motion_SetNewCommand(float v, float w);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_CONTROLLER_H */
