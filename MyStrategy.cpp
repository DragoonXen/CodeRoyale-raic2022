#include "MyStrategy.hpp"
#include <exception>
#include <iostream>
#include <algorithm>

namespace {
    using namespace model;

    template<typename T>
    std::vector<Unit *> filterUnits(std::vector<model::Unit> &units, const T filter) {
        std::vector<Unit*> result;
        for (auto& unit : units) {
            if (!filter(unit)) {
                continue;
            }
            result.push_back(&unit);
        }
        std::sort(result.begin(), result.end(), [](const auto &f, const auto &s) {
            return f->id < s->id;
        });
        return result;
    }
}

MyStrategy::MyStrategy(const model::Constants &constants) : constants(constants) {
    this->constants.Update();
}



model::Order MyStrategy::getOrder(const model::Game &game_base, DebugInterface *debugInterface) {
    Game game = game_base;
    for (auto& unit : game.units) {
        DRAW(([&unit, &debugInterface](){
            debugInterface->addPlacedText(unit.position + model::Vec2{1., 1.},
                                          "unit " + std::to_string(unit.id),
                                          {0.5, 0.5}, 1,
                                          debugging::Color(0, 0, 0, 1));
            debugInterface->addPlacedText(unit.position + model::Vec2{1., 0.},
                                          to_string_p(unit.shield, 1) + "|" + to_string_p(unit.health, 1),
                                          {0.5, 0.5}, 1,
                                          debugging::Color(0, 0, 0, 1));
        }));
    }

    const auto my_units_filter = [id = game.myId](const auto &unit) { return unit.playerId == id; };
    const auto enemies_filter = [id = game.myId](const auto &unit) { return unit.playerId != id; };

    auto my_units = filterUnits(game.units, my_units_filter);
    auto enemy_units = filterUnits(game.units, enemies_filter);

    double closest_enemy_dst = std::numeric_limits<double>::max();
    const Unit* enemy_to_attack = nullptr;
    for (const auto &enemy_unit: enemy_units) {
        const double distance_sum = [&enemy_unit, &my_units]() {
            double sum = 0.;
            for (const auto &my_unit: my_units) {
                sum += (my_unit->position - enemy_unit->position).norm();
            }
            return sum;
        }();
        if (distance_sum < closest_enemy_dst) {
            closest_enemy_dst = distance_sum;
            enemy_to_attack = enemy_unit;
        }
    }

    if (debugInterface) {
        for (const auto& item : debugInterface->getState().pressedKeys) {
            if (item.find('W') != std::string::npos) {
                this->point_move_to = debugInterface->getState().cursorWorldPosition;
            }
            if (item.find('Q') != std::string::npos) {
                this->point_look_to = debugInterface->getState().cursorWorldPosition;
            }
            if (item.find('C') != std::string::npos) {
                this->point_look_to.reset();
                this->point_move_to.reset();
            }
        }
    }

    if (last_tick_game) {
        auto my_units_last_tick = filterUnits(last_tick_game->units, my_units_filter);
        for (const auto& unit : my_units) {
            const auto old_unit = [unit, &my_units_last_tick]() -> Unit * {
                for (const auto &old_unit: my_units_last_tick) {
                    if (old_unit->id == unit->id) {
                        return old_unit;
                    }
                }
                return nullptr;
            }();
            sstr << "pos " << unit->position.toString() << " change "
                 << (unit->position - old_unit->position).toString()
                 << " velocity "
                 << unit->velocity.toString() << "(" << unit->velocity.norm() << ") distance "
                 << (unit->position - *this->point_move_to).norm()
                 << std::endl;
            if (expected && (*expected - unit->velocity).norm() > 1e-5) {
                std::cerr << sstr.str() << std::endl;
            }
            sstr.clear();
        }

    }

    std::unordered_map<int, model::UnitOrder> orders;
    for (const auto& unit : my_units) {
        DRAW(([look = this->point_look_to, move = this->point_move_to, &unit, &debugInterface]() {
            if (move) {
                debugInterface->addSegment(unit->position, *move, 0.1, debugging::Color(1., 0., 0., 1.));
            }
            if (look) {
                debugInterface->addSegment(unit->position, *look, 0.1, debugging::Color(0., 1., 0., 1.));
            }
        }));

        model::UnitOrder order(*unit);
        if (point_look_to) {
            order.targetDirection = *point_look_to - unit->position;
        }
        if (point_move_to) {
            const Vec2 speedCenterPoint = unit->position +
                                          unit->direction *
                                          ((constants.maxUnitForwardSpeed - constants.maxUnitBackwardSpeed) * .5);
            const Vec2 target_vector = (speedCenterPoint + (*point_move_to - speedCenterPoint).toLen(
                    (constants.maxUnitForwardSpeed + constants.maxUnitBackwardSpeed) * .5)) - unit->position;
            const Vec2 vector_change = (target_vector - unit->velocity);
            const Vec2 result_vector = unit->velocity +
                                       (vector_change.norm() >= constants.unitAccelerationPerTick ? vector_change.toLen(
                                               constants.unitAccelerationPerTick) : vector_change);
            DRAW(([&]() {
                debugInterface->addRing(speedCenterPoint, 0.1, 0.05, debugging::Color(0., 1., 0., 1.));
                debugInterface->addRing(speedCenterPoint,
                                        (constants.maxUnitForwardSpeed + constants.maxUnitBackwardSpeed) * .5, 0.1,
                                        debugging::Color(0., 1., 0., 1.));
                debugInterface->addGradientSegment(speedCenterPoint, debugging::Color(1., 0., 0., 1.),
                                                   *point_move_to,
                                                   debugging::Color(0., 0., 1., 1.), 0.1);
                debugInterface->addGradientSegment(unit->position, debugging::Color(0., 1., 0., 1.),
                                                   unit->position + target_vector,
                                                   debugging::Color(0., 0., 1., 1.), 0.1);
            }));
            sstr << "target " << target_vector.toString() << "(" << target_vector.norm() << ") expected vector "
                 << result_vector.toString() << " (" << result_vector.norm() << ") pos "
                 << (unit->position + result_vector * (1. / constants.ticksPerSecond)).toString() << std::endl;
            order.targetVelocity = target_vector;
            expected = result_vector;
        } else {
            expected.reset();
        }


        orders[unit->id] = std::move(order);
    }

    this->last_tick_game = std::move(game);

    DRAW(([&debugInterface]() { debugInterface->flush(); }));
    return model::Order(std::move(orders));
}

void MyStrategy::debugUpdate(DebugInterface &debugInterface) {}

void MyStrategy::finish() {}