#include "MyStrategy.hpp"
#include "utils/Movement.hpp"
#include <exception>
#include <iostream>
#include <algorithm>

namespace {
    using namespace model;

    template<typename T>
    std::vector<Unit *> filterUnits(std::vector<model::Unit> &units, const T filter) {
        std::vector<Unit *> result;
        for (auto &unit: units) {
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
    std::cerr << game.currentTick << std::endl;
    for (auto &unit: game.units) {
        DRAW(
                debugInterface->addPlacedText(unit.position + model::Vec2{1., 1.},
                                              "unit " + std::to_string(unit.id),
                                              {0., 0.}, 0.1,
                                              debugging::Color(0, 0, 0, 1));
                debugInterface->addPlacedText(unit.position + model::Vec2{1., 0.7},
                                              to_string_p(unit.shield, 1) + "|" + to_string_p(unit.health, 1),
                                              {0., 0.}, 0.1,
                                              debugging::Color(0, 0, 0, 1));
        );
    }

    const auto my_units_filter = [id = game.myId](const auto &unit) { return unit.playerId == id; };
    const auto enemies_filter = [id = game.myId](const auto &unit) { return unit.playerId != id; };

    auto my_units = filterUnits(game.units, my_units_filter);
    auto enemy_units = filterUnits(game.units, enemies_filter);

    double closest_enemy_dst = std::numeric_limits<double>::max();
    const Unit *enemy_to_attack = nullptr;
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
        for (const auto &item: debugInterface->getState().pressedKeys) {
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
        for (const auto &unit: my_units) {
            const auto old_unit = [unit, &my_units_last_tick]() -> Unit * {
                for (const auto &old_unit: my_units_last_tick) {
                    if (old_unit->id == unit->id) {
                        return old_unit;
                    }
                }
                return nullptr;
            }();
            if ((old_unit->position - unit->position).norm() > 1e-10) {
                std::cerr << "pos expected " << old_unit->position.toString() << " actual " << unit->position.toString() << " diff " << (old_unit->position - unit->position).norm() << std::endl;
                std::cerr << "vel expected " << old_unit->velocity.toString() << " actual " << unit->velocity.toString() << " diff " << (old_unit->velocity - unit->velocity).norm() << std::endl;
            }
        }

    }

    std::unordered_map<int, model::UnitOrder> orders;
    for (const auto &unit: my_units) {
//        DRAW(
//                if (point_move_to) {
//                    debugInterface->addSegment(unit->position, *point_move_to, 0.1, debugging::Color(1., 0., 0., 1.));
//                }
//                if (point_look_to) {
//                    debugInterface->addSegment(unit->position, *point_look_to, 0.1, debugging::Color(0., 1., 0., 1.));
//                }
//        );
//        DRAW(
//                if (&unit != my_units.data()) {
//                    for (double i = unit->position.x - 30; i < unit->position.x + 31; i += 1) {
//                        for (double j = unit->position.y - 30; j < unit->position.y + 31; j += 1) {
//                            int count = constants.Get(i, j).size();
//                            if (count == 0) {
//                                continue;
//                            }
//                            int centerX = constants.toI(i);
//                            int centerY = constants.toI(j);
//
//                            debugInterface->addRect({centerX - 0.5, centerY - 0.5}, {1., 1.},
//                                                    debugging::Color(1., 0., 0., 0.2 * count));
//                        }
//                    }
//                });


        model::UnitOrder order(*unit);
        if (point_look_to) {
            order.targetDirection = *point_look_to - unit->position;
            const double prev_direction = unit->direction.toRadians();
            unit->direction = applyNewDirection(unit->direction, order.targetDirection,
                                                rotationSpeed(unit->aim, unit->weapon, constants));
        }
        if (point_move_to) {
            const auto vector = MaxSpeedVector(unit->position, unit->direction, *point_move_to, constants);
            order.targetVelocity = vector;
            unit->velocity = ResultSpeedVector(unit->velocity, vector, constants);
            updateForCollision(*unit, constants, debugInterface);
        }

        for (const auto &target: kMoveDirections) {
            const auto point = unit->position + target.toLen(30);
            const auto vector = MaxSpeedVector(unit->position, unit->direction, point, constants);
//            DRAW(
//                    debugInterface->addGradientSegment(unit->position, debugging::Color(1., 0., 0., 1.),
//                                                       unit->position + vector, debugging::Color(0., 1., 0., 1.), .1);
//            );
        }

        orders[unit->id] = std::move(order);
    }

    this->last_tick_game = std::move(game);

    DRAW(debugInterface->flush(););
    return model::Order(std::move(orders));
}

void MyStrategy::debugUpdate(DebugInterface &debugInterface) {}

void MyStrategy::finish() {}