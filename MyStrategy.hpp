#ifndef _MY_STRATEGY_HPP_
#define _MY_STRATEGY_HPP_

#include "DebugInterface.hpp"
#include "model/Game.hpp"
#include "model/Order.hpp"
#include "model/Constants.hpp"
#include "utils/Simulation.hpp"
#include "utils/TickStartUpdate.hpp"
#include <functional>
#include <any>
#include <list>

class MyStrategy {
public:
    std::optional<model::Game> last_tick_game;

    std::optional<model::Vec2> point_move_to;
    std::optional<model::Vec2> point_look_to;
    std::unordered_map<int, double> lastUpdatePrecision;
    std::unordered_map<int, MoveRule> usedAvoidRule;
    std::unordered_map<int, double> incomingDamage;
    // radarState, starting angle, last finish tick
    std::unordered_map<int, std::tuple<int, double, int>> radarTaskData;
    std::unordered_map<int, std::list<double>> unknownDamage;
    std::unordered_map<int, MovementStat> unitMovementMem;
    std::unordered_map<int, Vec2> prevPos;
    std::unordered_map<int, int> ticksWithoutMovement;
    std::unordered_map<int, std::tuple<int, Vec2>> unstuckMovement;
    std::unordered_map<int, model::UnitOrder> prevOrders;
    std::shared_ptr<std::vector<std::vector<std::pair<int, double>>>> dangerMatrix;
    std::vector<std::vector<int>> lastSeenArray;
    static std::unordered_map<int, std::function<bool(const std::any&, const std::any&)>> taskFilter;

    MyStrategy(const model::Constants& constants);
    model::Order getOrder(const model::Game& game, DebugInterface* priority);
    void debugUpdate(DebugInterface& debugInterface);
    void finish();
};

#endif