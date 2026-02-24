#ifndef SCREEN2_1PRESENTER_HPP
#define SCREEN2_1PRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class Screen2_1View;

class Screen2_1Presenter : public touchgfx::Presenter, public ModelListener
{
public:
    Screen2_1Presenter(Screen2_1View& v);

    /**
     * The activate function is called automatically when this screen is "switched in"
     * (ie. made active). Initialization logic can be placed here.
     */
    virtual void activate();

    /**
     * The deactivate function is called automatically when this screen is "switched out"
     * (ie. made inactive). Teardown functionality can be placed here.
     */
    virtual void deactivate();

    virtual ~Screen2_1Presenter() {}

private:
    Screen2_1Presenter();

    Screen2_1View& view;
};

#endif // SCREEN2_1PRESENTER_HPP
