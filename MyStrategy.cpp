#include "MyStrategy.hpp"
#include "utils/Movement.hpp"
#include "utils/TimeMeasure.hpp"
#include "utils/Simulation.hpp"
#include "utils/TickStartUpdate.hpp"
#include "utils/Visible.hpp"
#include "utils/Task.hpp"
#include "utils/TaskFunctions.hpp"
#include <exception>
#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <queue>

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
    const auto& constants = Constants::INSTANCE;
    DebugInterface::INSTANCE = debugInterface;
    Game game = game_base;

    const auto my_units_filter = [id = game.myId](const auto &unit) { return unit.playerId == id; };
    const auto enemies_filter = [id = game.myId](const auto &unit) { return unit.playerId != id; };

    auto myUnits = filterUnits(game.units, my_units_filter);

    std::unordered_map<int, VisibleFilter> visibilityFilters;
    for (auto& unit : game.units) {
        unit.currentFieldOfView = FieldOfView(unit);
    }
    for (const auto unit : myUnits) {
        visibilityFilters[unit->id] = FilterObstacles(unit->position, unit->direction, unit->currentFieldOfView);
    }

    TimeMeasure::end(1);
    UpdateProjectiles(game, last_tick_game, myUnits, visibilityFilters);
    TimeMeasure::end(2);
    UpdateLoot(game, last_tick_game, myUnits, visibilityFilters);
    TimeMeasure::end(3);
    UpdateUnits(game, last_tick_game, myUnits, visibilityFilters);
    TimeMeasure::end(4);
    myUnits = filterUnits(game.units, my_units_filter);
    auto enemyUnits = filterUnits(game.units, enemies_filter);

    DRAW({
             for (auto &unit: game.units) {
                 debugInterface->addPlacedText(unit.position + model::Vec2{1., 1.},
                                               "unit " + std::to_string(unit.id) + "\n" +
                                               to_string_p(unit.shield, 1) +
                                               "|" + to_string_p(unit.health, 1) + "\n" +
                                               to_string_p(unit.position.x, 2) + "|" +
                                               to_string_p(unit.position.y, 2),
                                               {0., 1.}, 0.7,
                                               debugging::Color(0, 0, 0, 1));
             }
         });

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
#ifdef DEBUG_INFO
        if (last_tick_game) {
            auto my_units_last_tick = filterUnits(last_tick_game->units, my_units_filter);
            for (const auto &unit: myUnits) {
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
                    std::cerr << game.currentTick << " vel expected " << old_unit->velocity.toString() << "|"
                              << old_unit->velocity.norm() << " actual "
                              << unit->velocity.toString() << "|"
                              << unit->velocity.norm() << " actual " << " diff "
                            << (old_unit->velocity - unit->velocity).norm() << " angle diff "
                            << old_unit->velocity.toRadians() - unit->velocity.toRadians()
                            << std::endl;
                    std::cerr << game.currentTick << " aim expected " << std::to_string(old_unit->aim) << " actual "
                              << std::to_string(unit->aim) << std::endl;
                }
            }
        }
#endif
    }

    Vec2 centerPoint = {0., 0.};
    for (auto& unit : myUnits) {
        centerPoint += unit->position;
    }
    centerPoint *= (1. / myUnits.size());

//    double closest_enemy_dst = std::numeric_limits<double>::max();
//    const Unit *enemy_to_attack = nullptr;
//    for (const auto &enemyUnit: enemyUnits) {
//        double distance_sqr = (enemyUnit->position - centerPoint).sqrNorm();
//        if (distance_sqr < closest_enemy_dst) {
//            closest_enemy_dst = distance_sqr;
//            enemy_to_attack = enemyUnit;
//        }
//    }
//
//    const auto getClosest = [&enemyUnits](Vec2 position) {
//        double distance = std::numeric_limits<double>::infinity();
//        Unit *selected = nullptr;
//        for (auto enemyUnit: enemyUnits) {
//            const double currDist = (enemyUnit->position - position).sqrNorm();
//            if (currDist < distance) {
//                distance = currDist;
//                selected = enemyUnit;
//            }
//        }
//        return selected;
//    };
    std::priority_queue<Task> tasks;
    if (!enemyUnits.empty()) {
        for (const Unit* unit: myUnits) {
            if (!unit->weapon.has_value() || unit->ammo[*unit->weapon] == 0) {
                continue;
            }
             std::vector<std::pair<double, int>> distances;
            distances.reserve(enemyUnits.size());
            for (size_t i = 0; i != enemyUnits.size(); ++i) {
                const auto& enUnit = enemyUnits[i];
                distances.emplace_back(std::make_pair((unit->position - enUnit->position).sqrNorm(), i));
            }
            std::sort(distances.begin(), distances.end());
            for (const auto& item : distances) {
                 const auto& enUnit = enemyUnits[item.second];
                Task attackTask{0, unit->id,
                                std::to_string(unit->id) + " attack enemy " + std::to_string(enUnit->id),
                                {OrderType::kAction, OrderType::kRotate}};
                constexpr double kMaxDangerDistanceSqr = 64.;
                attackTask.score = kMaxDangerDistanceSqr * 1000. / std::max(kMaxDangerDistanceSqr, item.first);
                if (!IsVisible<VisionFilter::kShootFilter>(unit->position, unit->direction, unit->currentFieldOfView,
                                                           enUnit->position, visibilityFilters[unit->id])) {
                    attackTask.score /= 10.;
                }
                if (enUnit->shield + enUnit->health < constants.weapons[*unit->weapon].projectileDamage + 1e-5) {
                    attackTask.score *= 100.;
                }
                attackTask.func = [unit, enUnit, tick = game.currentTick, &visibilityFilters](POrder &order) -> std::vector<OrderType> {
                    return ApplyAttackTask(*unit, *enUnit, tick, visibilityFilters, order);
                };
                tasks.push(attackTask);

                Task moveTask{1, unit->id,
                              std::to_string(unit->id) + " move to attack enemy " + std::to_string(enUnit->id),
                              {OrderType::kMove}};
                moveTask.score = attackTask.score;
                if (!IsReachable(unit->position, enUnit->position, visibilityFilters[unit->id])) {
                    moveTask.score /= 10.;
                }
                moveTask.func = [unit, enUnit, &visibilityFilters](POrder &order) -> std::vector<OrderType> {
                    return ApplyMoveToUnitTask(*unit, *enUnit, visibilityFilters, order);
                };
                tasks.push(moveTask);
            };
        }
    }

    for (const Unit* unit: myUnits) {
        DRAWK('O', {
            if (point_move_to) {
                debugInterface->addSegment(unit->position, *point_move_to, 0.1, debugging::Color(1., 0., 0., 1.));
            }
            if (point_look_to) {
                debugInterface->addSegment(unit->position, *point_look_to, 0.1, debugging::Color(0., 1., 0., 1.));
            }
        });
        if (point_move_to) {
            Task moveTask{2, unit->id,
                          std::to_string(unit->id) + " Manual move to point " + point_move_to->toString(),
                          {OrderType::kMove}};
            moveTask.score = 1e100;
            moveTask.func = [unit, moveTo = *point_move_to, &filter = visibilityFilters[unit->id]](
                    POrder &order) -> std::vector<OrderType> {
                return ApplyMoveTo(*unit, moveTo, filter, std::numeric_limits<double>::infinity(), order);
            };
            tasks.push(moveTask);
        }
        if (point_look_to) {
            Task lookTask{3, unit->id,
                          std::to_string(unit->id) + " Manual look to point " + point_look_to->toString(),
                          {OrderType::kRotate}};
            lookTask.score = 1e100;
            lookTask.func = [lookTo = *point_look_to](POrder &order) -> std::vector<OrderType> {
                return ApplyLookTo(lookTo, order);
            };
            tasks.push(lookTask);
        }
    }
    for (const Unit *unit: myUnits) {
        Task moveTask{4, unit->id,
                      std::to_string(unit->id) + " Go to circle center " + game.zone.currentCenter.toString(),
                      {OrderType::kMove}};
        moveTask.score = 0.;
        moveTask.func = [unit, moveTo = game.zone.currentCenter, &filter = visibilityFilters[unit->id]](
                POrder &order) -> std::vector<OrderType> {
            return ApplyMoveTo(*unit, moveTo, filter, std::numeric_limits<double>::infinity(), order);
        };
        tasks.push(moveTask);
        Task lookTask{5, unit->id,
                      std::to_string(unit->id) + " Look to movement direction",
                      {OrderType::kRotate}};
        lookTask.score = -1.;
        ///here
        lookTask.func = [lookTo =
        unit->velocity.sqrNorm() > 1e-5 ? unit->position + unit->velocity.toLen(10.) : game.zone.currentCenter](
                POrder &order) -> std::vector<OrderType> {
            return ApplyLookTo(lookTo, order);
        };
        tasks.push(lookTask);
    }
    TimeMeasure::end(5);

    std::unordered_map<int, POrder> pOrders;
    for (Unit* unit: myUnits) {
        pOrders[unit->id] = POrder();
    }
    for (;!tasks.empty(); tasks.pop()) {
        const auto& task = tasks.top();
        POrder& curr = pOrders[task.unitId];
        if (!curr.IsAbleToAcceptTask(task.actionTypes)) {
            continue;
        }
        curr.Accept(task.func(curr), task.type);
    }
    TimeMeasure::end(6);

    std::unordered_map<int, model::UnitOrder> orders;

    for (Unit *unit: myUnits) {
        const auto &pOrder = pOrders[unit->id];
        MoveRule orderedRule = pOrder.toMoveRule();
        const auto [resultUnit, damageScore, firstProjectile] = Simulate(*unit, game, orderedRule);
        if (damageScore == 0.) {
            auto order = ApplyAvoidRule(*unit, orderedRule);
            order.action = pOrder.action;
//            std::cerr << order.toString() << std::endl;
            orders[unit->id] = order;
            continue;
        }
        TimeMeasure::end(7);
        std::vector<MoveRule> basicRules = [&]() {
            std::vector<MoveRule> avoidRules;
            Vec2 norm = {firstProjectile->velocity.y, -firstProjectile->velocity.x};
            if ((unit->velocity - norm).sqrNorm() > (unit->velocity + norm).sqrNorm()) {
                norm = -norm;
            }
            constexpr double kMoveLength = 30.;
            avoidRules.push_back({unit->position + norm.toLen(kMoveLength), orderedRule.lookDirection, orderedRule.keepAim,
                                  std::numeric_limits<double>::infinity()});
            avoidRules.push_back({unit->position - norm.toLen(kMoveLength), orderedRule.lookDirection, orderedRule.keepAim,
                                  std::numeric_limits<double>::infinity()});
            for (const auto &dir: kMoveDirections) {
                avoidRules.push_back({unit->position + dir * kMoveLength, orderedRule.lookDirection, orderedRule.keepAim,
                                      std::numeric_limits<double>::infinity()});
            }
            return avoidRules;
        }();

        std::vector<ComplexMoveRule> complexRules;
        for (const auto& rule : basicRules) {
            complexRules.emplace_back(ComplexMoveRule({{orderedRule, 1},
                                                       {{rule.moveDirection,
                                                                rule.moveDirection,
                                                                false,
                                                                std::numeric_limits<double>::infinity()},
                                                                     0}}));
        }
        for (const auto& rule : basicRules) {
            complexRules.emplace_back(rule);
        }
        const size_t notApplyAimAboveId = complexRules.size();
        for (auto &rule: basicRules) {
            rule.lookDirection = rule.moveDirection;
            complexRules.emplace_back(rule);
        }
        if (orderedRule.keepAim) {
            for (auto &rule: basicRules) {
                rule.keepAim = false;
                complexRules.emplace_back(rule);
            }
        }

        auto [score, rule_id] = ChooseBest(*unit, game, complexRules);
        auto order = ApplyAvoidRule(*unit, complexRules[rule_id].storage.front());
        if (rule_id < notApplyAimAboveId || !pOrder.aim) {
            order.action = pOrder.action;
        }
        orders[unit->id] = order;
        TimeMeasure::end(8);
    }

    TimeMeasure::end(7);


//    std::unordered_map<int, model::UnitOrder> orders;
//    for (const auto &unit: myUnits) {
//        /** AVOID LOGIC **/
//        if (point_move_to) {
//            const auto [resultUnit, damageScore, firstProjectile] = Simulate(*unit, game, *point_move_to, point_look_to,
//                                                                             false);
//            if (damageScore != 0.) {
//                Vec2 norm = {firstProjectile->velocity.y, -firstProjectile->velocity.x};
//                if ((unit->velocity - norm).sqrNorm() > (unit->velocity + norm).sqrNorm()) {
//                    norm = -norm;
//                }
//                std::vector<MoveRule> avoidRules;
//                constexpr double kMoveLength = 30.;
//                avoidRules.push_back({unit->position + norm.toLen(kMoveLength), point_look_to, false,
//                                      std::numeric_limits<double>::infinity()});
//                avoidRules.push_back({unit->position - norm.toLen(kMoveLength), point_look_to, false,
//                                      std::numeric_limits<double>::infinity()});
//                for (const auto &dir: kMoveDirections) {
//                    avoidRules.push_back({unit->position + dir * kMoveLength, point_look_to, false,
//                                          std::numeric_limits<double>::infinity()});
//                }
//                avoidRules.push_back(
//                        {unit->position + norm.toLen(kMoveLength), unit->position + norm.toLen(kMoveLength), false,
//                         std::numeric_limits<double>::infinity()});
//                avoidRules.push_back(
//                        {unit->position - norm.toLen(kMoveLength), unit->position - norm.toLen(kMoveLength), false,
//                         std::numeric_limits<double>::infinity()});
//                for (const auto &dir: kMoveDirections) {
//                    Vec2 to = unit->position + dir * kMoveLength;
//                    avoidRules.push_back({to, to, false, std::numeric_limits<double>::infinity()});
//                }
//                size_t rule_id = ChooseBest(*unit, game, avoidRules);
//                orders[unit->id] = ApplyAvoidRule(*unit, avoidRules[rule_id]);
//                continue;
//            }
//        }
//
//        model::UnitOrder order(*unit);
//        if (point_look_to) {
//            order.targetDirection = *point_look_to - unit->position;
//            unit->direction = applyNewDirection(unit->direction, order.targetDirection,
//                                                RotationSpeed(unit->aim, unit->weapon));
//        }
//        // aim
//        if (point_move_to) {
//            const auto vector = MaxSpeedVector(unit->position, unit->direction, *point_move_to,
//                                               CalcAimSpeedModifier(*unit));
//            order.targetVelocity = vector;
//            unit->velocity = ResultSpeedVector(unit->velocity, vector);
//            updateForCollision(unit->position, unit->velocity);
//        }
//
////        for (const auto &target: kMoveDirections) {
////            const auto point = unit->position + target.toLen(30);
////            const auto vector = MaxSpeedVector(unit->position, unit->direction, point, constants);
//////            DRAW(
//////                    debugInterface->addGradientSegment(unit->position, debugging::Color(1., 0., 0., 1.),
//////                                                       unit->position + vector, debugging::Color(0., 1., 0., 1.), .1);
//////            );
////        }
//
//        orders[unit->id] = std::move(order);
//    }


    this->last_tick_game = std::move(game);

    DRAW(debugInterface->flush(););
    TimeMeasure::end(10);
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

void MyStrategy::finish() {
}