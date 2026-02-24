#ifndef SCREEN4_1VIEW_HPP
#define SCREEN4_1VIEW_HPP

#include <gui_generated/screen4_1_screen/Screen4_1ViewBase.hpp>
#include <gui/screen4_1_screen/Screen4_1Presenter.hpp>

class Screen4_1View : public Screen4_1ViewBase
{
public:
    Screen4_1View();
    virtual ~Screen4_1View() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:
};

#endif // SCREEN4_1VIEW_HPP
