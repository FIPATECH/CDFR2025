#include <Arduino.h>
#include "strategy_manager.h"

void Apply_Strategy(Color teamColor, Zone teamZone, Zone enemyZone) {
    Serial.print("strategy_manager: Apply_Strategy color=");
    Serial.print(teamColor);
    Serial.print(", zoneEquipe=");
    Serial.print(teamZone);
    Serial.print(", zoneAdverse=");
    Serial.println(enemyZone);
    // stub : ne fait rien d’autre sur Mega
}
