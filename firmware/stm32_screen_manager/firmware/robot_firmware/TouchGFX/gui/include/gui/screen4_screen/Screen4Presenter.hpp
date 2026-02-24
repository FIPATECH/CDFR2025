#pragma once

#include <cstdint>
#include <mvp/Presenter.hpp>
#include <gui/model/ModelListener.hpp>

class Screen4View;

class Screen4Presenter : public touchgfx::Presenter,
                         public ModelListener
{
public:
    explicit Screen4Presenter(Screen4View &v);
    ~Screen4Presenter() override = default;

    void activate() override;
    void deactivate() override;

    void updatePoints(uint32_t pts);

    void bind(Model *m) { model = m; }

private:
    Screen4View &view;
    Model *model{nullptr};
};
