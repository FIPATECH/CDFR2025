#pragma once
#include <gui_generated/screen4_screen/Screen4ViewBase.hpp>
#include <cstdint>

class Screen4View : public Screen4ViewBase
{
public:
    Screen4View();
    void setupScreen() override;
    void tearDownScreen() override;
    void handleTickEvent() override; // ← ajoute
    void updatePoints(uint32_t pts); // appelé par le Presenter

private:
    static constexpr uint16_t TEXTPOINTS_SIZE = 6;
    uint32_t pendingPts{0}; // valeur en attente
    bool dirty{false};      // flag « il faut rafraîchir »
};
