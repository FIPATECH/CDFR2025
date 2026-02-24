#ifndef SCREEN2_1VIEW_HPP
#define SCREEN2_1VIEW_HPP

#include <gui_generated/screen2_1_screen/Screen2_1ViewBase.hpp>
#include <gui/screen2_1_screen/Screen2_1Presenter.hpp>

class Screen2_1View : public Screen2_1ViewBase
{
public:
    Screen2_1View();
    virtual ~Screen2_1View() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:
};

#endif // SCREEN2_1VIEW_HPP
