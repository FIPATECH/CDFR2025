#include <gui/screen3_screen/Screen3Presenter.hpp>
#include <gui_generated/common/FrontendApplicationBase.hpp>
#include <touchgfx/Application.hpp> // pour Application::getInstance()

Screen3Presenter::Screen3Presenter(Screen3View &v)
    : view(v)
{
}

void Screen3Presenter::activate()
{
}

void Screen3Presenter::deactivate()
{
}

void Screen3Presenter::startMatch()
{
    // 1) Récupérer le pointeur générique
    touchgfx::Application *baseApp = touchgfx::Application::getInstance();
    // 2) Le caster vers votre classe dérivée
    FrontendApplicationBase *app =
        static_cast<FrontendApplicationBase *>(baseApp);
    // 3) Appeler la transition
    // app->gotoScreen4ScreenSlideTransitionEast();
    app->gotoScreen4_1ScreenSlideTransitionEast();
}