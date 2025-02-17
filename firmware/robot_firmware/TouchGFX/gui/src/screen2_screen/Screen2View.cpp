#include <gui/screen2_screen/Screen2View.hpp>
#include "../../../Core/Inc/strategy_manager.h"

Screen2View::Screen2View()
{
}

void Screen2View::setupScreen()
{
    Screen2ViewBase::setupScreen();
}

void Screen2View::tearDownScreen()
{
    Screen2ViewBase::tearDownScreen();
}

void Screen2View::Send_Strategy()
{
    // Récupération des coordonnées des deux cercles
    int circle1_x = circle1.getX(); // Mon robot
    int circle1_y = circle1.getY();
    int circle2_x = circle2.getX(); // Robot adverse
    int circle2_y = circle2.getY();

    Zone teamZone;
    Zone enemyZone;
    Color teamColor;

    // Pour le test, si mon robot se trouve sur la moitié gauche de l'écran, alors il est JAUNE, sinon BLEUE.
    if (circle1_x < 160)
    {
        teamColor = JAUNE;
    }
    else
    {
        teamColor = BLEUE;
    }

    // Cas 1 : les deux robots sur la moitié gauche
    if (circle1_x < 160 && circle2_x < 160)
    {
        if (circle1_y < 160)
            teamZone = HAUT_GAUCHE;
        else if (circle1_y > 160)
            teamZone = BAS_GAUCHE;
        else
            teamZone = CENTRE_GAUCHE;

        if (circle2_y < 160)
            enemyZone = HAUT_GAUCHE;
        else if (circle2_y > 160)
            enemyZone = BAS_GAUCHE;
        else
            enemyZone = CENTRE_GAUCHE;

        Apply_Strategy(teamColor, teamZone, enemyZone);
    }
    // Cas 2 : les deux robots sur la moitié droite
    else if (circle1_x >= 160 && circle2_x >= 160)
    {
        if (circle1_y < 160)
            teamZone = HAUT_DROITE;
        else if (circle1_y > 160)
            teamZone = BAS_DROITE;
        else
            teamZone = CENTRE_DROITE;

        if (circle2_y < 160)
            enemyZone = HAUT_DROITE;
        else if (circle2_y > 160)
            enemyZone = BAS_DROITE;
        else
            enemyZone = CENTRE_DROITE;

        Apply_Strategy(teamColor, teamZone, enemyZone);
    }
    // Cas 3 : les robots se trouvent dans des moitiés différentes
    else
    {
        // Pour mon robot
        if (circle1_x < 160)
        {
            if (circle1_y < 160)
                teamZone = HAUT_GAUCHE;
            else if (circle1_y > 160)
                teamZone = BAS_GAUCHE;
            else
                teamZone = CENTRE_GAUCHE;
        }
        else
        {
            if (circle1_y < 160)
                teamZone = HAUT_DROITE;
            else if (circle1_y > 160)
                teamZone = BAS_DROITE;
            else
                teamZone = CENTRE_DROITE;
        }
        // Pour le robot adverse
        if (circle2_x < 160)
        {
            if (circle2_y < 160)
                enemyZone = HAUT_GAUCHE;
            else if (circle2_y > 160)
                enemyZone = BAS_GAUCHE;
            else
                enemyZone = CENTRE_GAUCHE;
        }
        else
        {
            if (circle2_y < 160)
                enemyZone = HAUT_DROITE;
            else if (circle2_y > 160)
                enemyZone = BAS_DROITE;
            else
                enemyZone = CENTRE_DROITE;
        }

        Apply_Strategy(teamColor, teamZone, enemyZone);
    }
}
