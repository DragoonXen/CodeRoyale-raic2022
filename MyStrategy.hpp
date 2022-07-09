#ifndef _MY_STRATEGY_HPP_
#define _MY_STRATEGY_HPP_

#include "DebugInterface.hpp"
#include "model/Game.hpp"
#include "model/Order.hpp"
#include "model/Constants.hpp"

class MyStrategy {
    model::Constants constants;
    std::optional<model::Game> last_tick_game;

    std::optional<model::Vec2> point_move_to;
    std::optional<model::Vec2> point_look_to;

public:
    MyStrategy(const model::Constants& constants);
    model::Order getOrder(const model::Game& game, DebugInterface* debugInterface);
    void debugUpdate(DebugInterface& debugInterface);
    void finish();
};

#endif