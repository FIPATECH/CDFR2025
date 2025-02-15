#ifndef STRATEGY_MANAGER_H
#define STRATEGY_MANAGER_H

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        HAUT_GAUCHE = 0,
        CENTRE_DROITE = 1,
        BAS_GAUCHE = 2,
        HAUT_DROITE = 3,
        CENTRE_GAUCHE = 4,
        BAS_DROITE = 5
    } Zone;

    typedef enum
    {
        JAUNE = 0,
        BLEUE = 1
    } Color;

    typedef struct
    {
        int couleur;
        int myzone;
        int ennemyzone;
    } Strategie;

    void applyStrategy(Color teamColor, Zone myZone, Zone enemyZone);

#ifdef __cplusplus
}
#endif

#endif /* __STRATEGY_MANAGER_H */
