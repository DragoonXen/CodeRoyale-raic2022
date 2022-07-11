#include "MyStrategy.hpp"
#include "utils/Movement.hpp"
#include "utils/TimeMeasure.hpp"
#include "utils/Simulation.hpp"
#include "utils/TickStartUpdate.hpp"
#include "utils/Visible.hpp"
#include <exception>
#include <iostream>
#include <algorithm>
#include <unordered_map>

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

MyStrategy::MyStrategy(const model::Constants &constants) {
    TimeMeasure::start();
    Constants::INSTANCE = constants;
    Constants::INSTANCE.Update();
    TimeMeasure::end(0);
}

model::Order MyStrategy::getOrder(const model::Game &game_base, DebugInterface *debugInterface) {
    TimeMeasure::start();
    DebugInterface::INSTANCE = debugInterface;
    Game game = game_base;
    for (auto &unit: game.units) {
        DRAW(
                debugInterface->addPlacedText(unit.position + model::Vec2{1., 1.},
                                              "unit " + std::to_string(unit.id) + "\n" + to_string_p(unit.shield, 1) +
                                              "|" + to_string_p(unit.health, 1) + "\n" +
                                              to_string_p(unit.position.x, 2) + "|" + to_string_p(unit.position.y, 2),
                                              {0., 1.}, 0.7,
                                              debugging::Color(0, 0, 0, 1));
        );
    }


    const auto my_units_filter = [id = game.myId](const auto &unit) { return unit.playerId == id; };
    const auto enemies_filter = [id = game.myId](const auto &unit) { return unit.playerId != id; };

    auto my_units = filterUnits(game.units, my_units_filter);
    auto enemy_units = filterUnits(game.units, enemies_filter);

    std::unordered_map<int, VisibleFilter> filters;
    for (const auto unit : my_units) {
        unit->currentFieldOfView = FieldOfView(*unit);
        filters[unit->id] = FilterObstacles(unit->position, unit->direction, unit->currentFieldOfView);
    }

    UpdateProjectiles(game, last_tick_game, my_units, filters);
    UpdateLoot(game, last_tick_game, my_units, filters);
    TimeMeasure::end(1);

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
                std::cerr << game.currentTick << " pos expected " << old_unit->position.toString() << " actual "
                          << unit->position.toString() << " diff " << (old_unit->position - unit->position).norm()
                          << std::endl;
                std::cerr << game.currentTick << " vel expected " << old_unit->velocity.toString() << " actual "
                          << unit->velocity.toString() << " diff " << (old_unit->velocity - unit->velocity).norm()
                          << std::endl;
            }
        }

    }

    std::unordered_map<int, model::UnitOrder> orders;
    for (const auto &unit: my_units) {
        DRAWK('O', {
            if (point_move_to) {
                debugInterface->addSegment(unit->position, *point_move_to, 0.1, debugging::Color(1., 0., 0., 1.));
            }
            if (point_look_to) {
                debugInterface->addSegment(unit->position, *point_look_to, 0.1, debugging::Color(0., 1., 0., 1.));
            }
        });
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

        /** AVOID LOGIC **/
        if (point_move_to) {
            const auto [resultUnit, damageScore, firstProjectile] = Simulate(*unit, game, *point_move_to, point_look_to,
                                                                             false);
            if (damageScore != 0.) {
                Vec2 norm = {firstProjectile->velocity.y, -firstProjectile->velocity.x};
                if ((unit->velocity - norm).sqrNorm() > (unit->velocity + norm).sqrNorm()) {
                    norm = -norm;
                }
                std::vector<AvoidRule> avoidRules;
                constexpr double kMoveLength = 30.;
                avoidRules.push_back({unit->position + norm.toLen(kMoveLength), point_look_to, false,
                                      std::numeric_limits<double>::infinity()});
                avoidRules.push_back({unit->position - norm.toLen(kMoveLength), point_look_to, false,
                                      std::numeric_limits<double>::infinity()});
                for (const auto &dir: kMoveDirections) {
                    avoidRules.push_back({unit->position + dir * kMoveLength, point_look_to, false,
                                          std::numeric_limits<double>::infinity()});
                }
                avoidRules.push_back(
                        {unit->position + norm.toLen(kMoveLength), unit->position + norm.toLen(kMoveLength), false,
                         std::numeric_limits<double>::infinity()});
                avoidRules.push_back(
                        {unit->position - norm.toLen(kMoveLength), unit->position - norm.toLen(kMoveLength), false,
                         std::numeric_limits<double>::infinity()});
                for (const auto &dir: kMoveDirections) {
                    Vec2 to = unit->position + dir * kMoveLength;
                    avoidRules.push_back({to, to, false, std::numeric_limits<double>::infinity()});
                }
                size_t rule_id = ChooseBest(*unit, game, avoidRules);
                orders[unit->id] = ApplyAvoidRule(*unit, avoidRules[rule_id]);
                continue;
            }
        }

        model::UnitOrder order(*unit);
        if (point_look_to) {
            order.targetDirection = *point_look_to - unit->position;
            unit->direction = applyNewDirection(unit->direction, order.targetDirection,
                                                rotationSpeed(unit->aim, unit->weapon));
        }
        // aim
        if (point_move_to) {
            const auto vector = MaxSpeedVector(unit->position, unit->direction, *point_move_to,
                                               CalcAimSpeedModifier(*unit));
            order.targetVelocity = vector;
            unit->velocity = ResultSpeedVector(unit->velocity, vector);
            updateForCollision(unit->position, unit->velocity);
        }

//        for (const auto &target: kMoveDirections) {
//            const auto point = unit->position + target.toLen(30);
//            const auto vector = MaxSpeedVector(unit->position, unit->direction, point, constants);
////            DRAW(
////                    debugInterface->addGradientSegment(unit->position, debugging::Color(1., 0., 0., 1.),
////                                                       unit->position + vector, debugging::Color(0., 1., 0., 1.), .1);
////            );
//        }

        orders[unit->id] = std::move(order);
    }

    this->last_tick_game = std::move(game);

    DRAW(debugInterface->flush(););
    return model::Order(std::move(orders));
}

void MyStrategy::debugUpdate(DebugInterface &debugInterface) {
    DRAW(
            static bool drawed = false;
            if (drawed) {
                return;
            }
            for (const auto &obstacle: Constants::INSTANCE.obstacles) {
                debugInterface->addPlacedText(obstacle.position,
                                              "o " + std::to_string(obstacle.id),
                                              {0.5, 0.5}, 0.7,
                                              debugging::Color(0, 0, 0, 1));
            }
            drawed = true;
    );
}

void MyStrategy::finish() {}