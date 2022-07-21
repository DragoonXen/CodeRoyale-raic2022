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

    void debugActions(const POrder &pOrder, const Unit &unit, int avoidance) {
        DRAW({
                 std::stringstream result;
                 for (size_t i = 0; i != (int) OrderType::kItemsCount; ++i) {
                     if (pOrder.picked[i] == -1) {
                         continue;
                     }
                     result << pOrder.picked[i] << ": " << pOrder.description[i] << std::endl;
                 }
                 if (avoidance != -1) {
                     result << avoidance;
                 }
                 debugInterface->addPlacedText(unit.position - Vec2{1., -1},
                                               result.str(),
                                               {1., 1.}, 0.7,
                                               debugging::Color(0, 0, 0, 1));
             });
    }

    double EvaluatePathDanger(Vec2 from, Vec2 to, std::vector<std::vector<std::pair<int, double>>> &dangerMatrix,
                              const model::Game &game) {
        double sumDanger = 0.;

        const Vec2 vector = to - from;
        const Vec2 dir = vector.toLen(1.);
        const size_t kMinDistance = 0.;
        constexpr double attenuationConstant = 0.986232704493359; // distance of 50 - 0.5 value
        double currentCoeff = attenuationConstant;
        size_t maxSteps = vector.norm() - kMinDistance;
        for (size_t i = 1; i <= maxSteps; ++i) {
            Vec2 nextPos = from + dir * (double) i;
            sumDanger += EvaluateDanger(nextPos, dangerMatrix, game) * currentCoeff;
            currentCoeff *= attenuationConstant;
        }
        return sumDanger;
    }
}

std::unordered_map<int, std::function<bool(const std::any&, const std::any&)>> MyStrategy::taskFilter;

MyStrategy::MyStrategy(const model::Constants &constants) {
    TimeMeasure::start();
    Constants::INSTANCE = constants;
    Constants::INSTANCE.Update();
    taskFilter[6] = [](const std::any &newTask, const std::any &applied) -> bool {
        auto newPair = std::any_cast<std::pair<int, int>>(newTask);
        auto appliedPair = std::any_cast<std::pair<int, int>>(applied);
        return newPair.first == appliedPair.first && newPair.second != appliedPair.second;
    };
    taskFilter[101] = [](const std::any &newTask, const std::any &applied) -> bool {
        auto newPos = std::any_cast<Vec2>(newTask);
        auto prevPos = std::any_cast<Vec2>(applied);
        return (newPos - prevPos).sqrNorm() < 45. * 45.;
    };
    {
        std::vector<std::pair<int, double>> baseDanger(Constants::INSTANCE.obstacleMatrix[0].size(),
                                                       std::make_pair(-1, 0.));
        dangerMatrix = std::make_shared<std::vector<std::vector<std::pair<int, double>>>>();
        dangerMatrix->resize(Constants::INSTANCE.obstacleMatrix.size(), baseDanger);
        int sizeX = (dangerMatrix->size() + kLastSeenArrayStep - 1) / kLastSeenArrayStep + 1;
        int sizeY = ((*dangerMatrix)[0].size() + kLastSeenArrayStep - 1) / kLastSeenArrayStep + 1;
        std::vector<int> yDimVector(sizeY, -1);
        lastSeenArray.resize(sizeX, yDimVector);
        std::cerr << lastSeenArray.size() << ":" << lastSeenArray[0].size() << std::endl;
    }
    TimeMeasure::end(0);
}

model::Order MyStrategy::getOrder(const model::Game &game_base, DebugInterface *debugInterface) {
#ifdef DEBUG_INFO
    if (game_base.currentTick == 0) {
        debugging::DebugState::processKeys({"I"});
    }
#endif
    TimeMeasure::start();
    auto &dangerMatrix = *this->dangerMatrix;
    const auto &constants = Constants::INSTANCE;
    DebugInterface::INSTANCE = debugInterface;
    Game game = game_base;

    const auto my_units_filter = [id = game.myId](const auto &unit) { return unit.playerId == id; };
    const auto enemies_filter = [id = game.myId](const auto &unit) { return unit.playerId != id; };

    auto myUnits = filterUnits(game.units, my_units_filter);
    std::unordered_map<int, double> unknownIncomingDamageSum;
    if (game_base.currentTick == 0) {
        for (auto unit: myUnits) {
            unknownDamage[unit->id] = {};
            unknownIncomingDamageSum[unit->id] = 0.;
            radarTaskData[unit->id] = {0, 0., -10000};
        }
    }

    std::unordered_map<int, VisibleFilter> visibilityFilters;
    for (auto &unit: game.units) {
        unit.currentFieldOfView = FieldOfView(unit);
    }
    for (const auto unit: myUnits) {
        visibilityFilters[unit->id] = FilterObstacles(unit->position, unit->direction, unit->currentFieldOfView);
    }
    ZoneMover zoneMover(game.zone);

    {
        TimeMeasure::end(1);
        auto projectilesUnitInfo = UpdateProjectiles(game, last_tick_game, myUnits, game.units, visibilityFilters);
        TimeMeasure::end(2);
        UpdateLoot(game, last_tick_game, myUnits, game.units, visibilityFilters);
        TimeMeasure::end(3);
        UpdateVisited(lastSeenArray, myUnits, game.currentTick);
        TimeMeasure::end(9);
        UpdateUnits(game, last_tick_game, myUnits, game.units, visibilityFilters, std::move(projectilesUnitInfo), this->unitMovementMem);
        TimeMeasure::end(4);
    }
    myUnits = filterUnits(game.units, my_units_filter);
    auto enemyUnits = filterUnits(game.units, enemies_filter);

    std::unordered_map<int, std::unordered_map<int, double>> unitDanger;
    for (const auto &unit: myUnits) {
        unitDanger[unit->id] = std::unordered_map<int, double>();
        auto &myUnitMap = unitDanger[unit->id];
        for (const auto &enemyUnit: enemyUnits) {
            myUnitMap[enemyUnit->id] = CalculateDanger(unit->position, *enemyUnit);
//            DRAW({
//                     debugInterface->addPlacedText(enemyUnit->position + distanceVec * .5,
//                                                   std::to_string(enemyUnit->id) + " danger " + std::to_string(danger),
//                                                   {0.5, 0.5}, 0.7,
//                                                   debugging::Color(0.8, 0, 0, 1.));
//                 });
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
        for (const auto &unit: myUnits) {
            const auto old_unit = [unit, &my_units_last_tick]() -> Unit * {
                for (const auto &old_unit: my_units_last_tick) {
                    if (old_unit->id == unit->id) {
                        return old_unit;
                    }
                }
                return nullptr;
            }();
            const double actualIncomingDamage = (old_unit->shield + old_unit->health) - (unit->shield + unit->health);
            const double unknownIncomingDamage = std::max(actualIncomingDamage - incomingDamage[unit->id], 0.);
            constexpr size_t maxDamageCountingPeriod = 90;
            auto &list = unknownDamage[unit->id];
            list.push_back(unknownIncomingDamage);
            if (list.size() > maxDamageCountingPeriod) {
                list.pop_front();
            }
            double sumDamage = 0.;
            for (const auto &item: list) {
                sumDamage += item;
            }
            unknownIncomingDamageSum[unit->id] = sumDamage;

#ifdef DEBUG_INFO
            if ((old_unit->position - unit->position).norm() > 1e-8) {
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
#endif
        }
    }

    DRAW({
             for (auto &unit: game.units) {
                 debugInterface->addPlacedText(unit.position + model::Vec2{1., 1.},
                                               "unit " + std::to_string(unit.id) + "\n" +
                                               to_string_p(unit.shield, 1) +
                                               "|" + to_string_p(unit.health, 1) + "\n" +
                                               to_string_p(unit.position.x, 2) + "|" +
                                               to_string_p(unit.position.y, 2) + "\n" +
                                               to_string_p(unknownIncomingDamageSum[unit.id], 1),
                                               {0., 1.}, 0.7,
                                               debugging::Color(0, 0, 0, 1));
             }
         });
//    DRAW({
//             if (myUnits.empty()) {
//                 return;
//             }
//             auto pos = myUnits[0]->position;
//             double minVal = std::numeric_limits<double>::infinity();
//             double maxVal = -std::numeric_limits<double>::infinity();
//             for (double dx = -30.; dx <= 30; dx += 1.) {
//                 for (double dy = -30.; dy <= 30; dy += 1.) {
//                     //MyStrategy& strategy, Vec2 pos, model::Game& game, const std::vector<model::Unit> units
//                     auto val = EvaluateDanger(*this->dangerMatrix, pos + Vec2{dx, dy}, game);
//                     minVal = std::min(val, minVal);
//                     maxVal = std::max(val, maxVal);
//                 }
//             }
//             double diffVal = 1. / (maxVal - minVal);
//             debugInterface->setAutoFlush(false);
//             for (double dx = -30.; dx <= 30; dx += 1.) {
//                 double posX = Constants::toI(pos.x + dx);
//                 for (double dy = -30.; dy <= 30; dy += 1.) {
//                     double posY = Constants::toI(pos.y + dy);
//                     auto val = EvaluateDanger(*this->dangerMatrix, {posX, posY}, game);
//                     debugInterface->addRect({posX - 0.5, posY - 0.5}, {1., 1.},
//                                             debugging::Color((val - minVal) * diffVal, 0., 0., .8));
//                 }
//             }
//             debugInterface->flush();
//             debugInterface->setAutoFlush(true);
//         });

    Vec2 centerPoint = {0., 0.};
    for (auto &unit: myUnits) {
        centerPoint += unit->position;
    }
    centerPoint *= (1. / myUnits.size());

    std::unordered_map<int, const Unit *> unitById;
    const std::vector<Unit> unitsBackup = game.units;
    for (const auto &unit: unitsBackup) {
        unitById[unit.id] = &unit;
    }
    std::unordered_map<int, const Loot *> lootById;
    std::priority_queue<Task> tasks;
    const auto evalPathDanger = [&game, &dangerMatrix = this->dangerMatrix](Vec2 from, Vec2 to) {
        return EvaluatePathDanger(from, to, *dangerMatrix, game);
    };
    if (!enemyUnits.empty()) {
        for (const Unit *unit: myUnits) {
            if (!unit->weapon.has_value() || unit->ammo[*unit->weapon] == 0) {
                continue;
            }
            std::vector<std::pair<double, int>> distances;
            distances.reserve(enemyUnits.size());
            for (size_t i = 0; i != enemyUnits.size(); ++i) {
                const auto &enUnit = enemyUnits[i];
                const double distanceSqr = (unit->position - enUnit->position).sqrNorm();
                if (ShootWhileSpawning(*unit, *enUnit, 0.)) {
                    continue;
                }
                distances.emplace_back(distanceSqr, i);
            }
            std::sort(distances.begin(), distances.end());
            const double damageUnitCouldCause =
                    unit->ammo[*unit->weapon] * constants.weapons[*unit->weapon].projectileDamage;
            for (const auto &item: distances) {
                const auto &enUnit = enemyUnits[item.second];
                Task moveTask{1, unit->id,
                              std::to_string(unit->id) + " move to attack enemy " + std::to_string(enUnit->id),
                              {OrderType::kMove}};
                constexpr double kMaxDangerDistanceSqr = 64.;
                moveTask.score = kMaxDangerDistanceSqr * 1000. / std::max(kMaxDangerDistanceSqr, item.first);
                if (!IsVisible<VisionFilter::kShootFilter>(unit->position, unit->direction, unit->currentFieldOfView,
                                                           enUnit->position, visibilityFilters[unit->id])) {
                    moveTask.score /= 10.;
                }
                if (item.first < 60. * 60. && enUnit->shield + enUnit->health < constants.weapons[*unit->weapon].projectileDamage + 1e-5) {
                    moveTask.score *= 20.;
                }
                if (enUnit->health + enUnit->shield > damageUnitCouldCause) {
                    moveTask.score /= 100.;
                }
//                if (!IsReachable(unit->position, enUnit->position, visibilityFilters[unit->id])) {
//                    moveTask.score /= 10.;
//                }
                moveTask.preEval = [unit, enUnit, &visibilityFilter = visibilityFilters[unit->id], &myUnits, &zoneMover, &dangerMatrix, &game]() -> std::tuple<double, std::any> {
                    return {0., ApplyMoveToUnitTask(*unit, myUnits, *enUnit, visibilityFilter, zoneMover, dangerMatrix, game)};
                };

                moveTask.func = [unit, &visibilityFilter = visibilityFilters[unit->id]](
                        const std::any &evalData, POrder &order) -> std::vector<OrderType> {
                    const auto &[position, maxSpeed] = std::any_cast<DestinationWithMaxSpeed>(evalData);
                    return ApplyMoveTo(*unit, position, visibilityFilter, maxSpeed, order);
                };
                tasks.push(moveTask);
                if (item.first > 30. * 30.) { // no attack task fot this case. Going to danger control task, type 9
                    continue;
                }
                Task attackTask{0, unit->id,
                                std::to_string(unit->id) + " attack enemy " + std::to_string(enUnit->id),
                                {OrderType::kAction, OrderType::kRotate}};
                attackTask.score = moveTask.score * std::max((unit->aim - 0.45) * 20., 1.);
                attackTask.func = [unit, &myUnits, enUnit, tick = game.currentTick, &visibilityFilters, &mem = this->unitMovementMem](
                        const std::any &evalData, POrder &order) -> std::vector<OrderType> {
                    return ApplyAttackTask(*unit, myUnits, *enUnit, tick, visibilityFilters, mem, order);
                };
                tasks.push(attackTask);
            };
        }
    }
    /***
     * point_move_to / point_look_to
     */
    for (const Unit *unit: myUnits) {
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
                    const std::any &evalData, POrder &order) -> std::vector<OrderType> {
                return ApplyMoveTo(*unit, moveTo, filter, std::numeric_limits<double>::infinity(), order);
            };
            tasks.push(moveTask);
        }
        if (point_look_to) {
            Task lookTask{3, unit->id,
                          std::to_string(unit->id) + " Manual look to point " + point_look_to->toString(),
                          {OrderType::kRotate}};
            lookTask.score = 1e100;
            lookTask.func = [lookTo = *point_look_to](const std::any &evalData,
                                                      POrder &order) -> std::vector<OrderType> {
                return ApplyLookTo(lookTo, order);
            };
            tasks.push(lookTask);
        }
    }
//    std::unordered_map<int, std::pair<double, Vec2>> bestExploreScore;
//    for (const Unit *unit: myUnits) {
//        bestExploreScore[unit->id] = std::make_pair(0., Vec2{});
//    }
    for (size_t x = 0; x != lastSeenArray.size(); ++x) {
        for (size_t y = 0; y != lastSeenArray[0].size(); ++y) {
            int val = lastSeenArray[x][y];
            if (val == game.currentTick) {
                continue;
            }
            Vec2 targetPos = GetLastSeenCoord(x, y);
            if ((game.zone.currentCenter - targetPos).norm() + kLastSeenArrayStep > game.zone.currentRadius) {
                continue;
            }
            const double basicScore = val == -1 ? 1. : (game.currentTick - val) / 100000.;

            for (const Unit *unit: myUnits) {
                Task moveTask{101, unit->id, std::to_string(unit->id) + " Smart explore zone " + targetPos.toString(),
                              {OrderType::kMove}};
                moveTask.score = basicScore * 28000. / (unit->position - targetPos).sqrNorm();
//                moveTask.preEval = [unit, &targetPos, &evalPathDanger]() -> std::tuple<double, std::any> {
//                    return {evalPathDanger(unit->position, targetPos),
//                            DestinationWithMaxSpeed{targetPos, std::numeric_limits<double>::infinity()}};
//                };
//                auto& pair = bestExploreScore[unit->id];
//                if (pair.first < moveTask.score) {
//                    pair.first = moveTask.score;
//                    pair.second = targetPos;
//                }
                moveTask.taskData = targetPos;
                moveTask.func = [unit, targetPos, &visibilityFilters](
                        const std::any &evalData, POrder &order) -> std::vector<OrderType> {
//                    const auto &[position, maxSpeed] = std::any_cast<DestinationWithMaxSpeed>(evalData);
                    return ApplyMoveTo(*unit, targetPos, visibilityFilters[unit->id],
                                       std::numeric_limits<double>::infinity(),
                                       order);
                };

                tasks.push(moveTask);
            }
        }
    }
//    {
//        for (const Unit *unit: myUnits) {
//            std::cerr << game.currentTick << "|" << unit->id << ":" << to_string_p(bestExploreScore[unit->id].first, 6)
//                      << ";" << bestExploreScore[unit->id].second.toString() << " d "
//                      << (bestExploreScore[unit->id].second - unit->position).norm() << std::endl;
//        }
//    }
    for (const Unit *unit: myUnits) {
        Task moveTask{4, unit->id, std::to_string(unit->id) + " Explore zone ", {OrderType::kMove}};
        moveTask.score = 0.;
        moveTask.preEval = [unit, &zone = game.zone, &filter = visibilityFilters[unit->id], &evalPathDanger]() -> std::tuple<double, std::any> {
            Vec2 zoneDst = unit->position - zone.currentCenter;
            if (zoneDst.sqrNorm() < 1.) {
                zoneDst = {1., 0};
            }
            const Vec2 moveDirection = zone.currentCenter + Vec2(zoneDst.toRadians() + M_PI / 6).toLen(
                    std::max(0., zone.currentRadius - Constants::INSTANCE.viewDistance));
            // evaluate here
            return {evalPathDanger(unit->position, moveDirection),
                    DestinationWithMaxSpeed{moveDirection, std::numeric_limits<double>::infinity()}};
        };

        moveTask.func = [unit, &visibilityFilter = visibilityFilters[unit->id]](
                const std::any &evalData, POrder &order) -> std::vector<OrderType> {
            const auto &[position, maxSpeed] = std::any_cast<DestinationWithMaxSpeed>(evalData);
            return ApplyMoveTo(*unit, position, visibilityFilter, maxSpeed, order);
        };

        tasks.push(moveTask);
        Task lookTask{5, unit->id,
                      std::to_string(unit->id) + " Look to movement direction",
                      {OrderType::kRotate}};
        lookTask.score = -1.;
        ///here
        lookTask.func = [lookTo =
        unit->velocity.sqrNorm() > 1e-5 ? unit->position + unit->velocity.toLen(10.) : game.zone.currentCenter](
                const std::any &evalData, POrder &order) -> std::vector<OrderType> {
            return ApplyLookTo(lookTo, order);
        };
        tasks.push(lookTask);
    }
    // loot && potions usage
    for (const Unit *unit: myUnits) {
        std::vector<std::pair<double, int>> ammos;
        std::vector<std::pair<double, int>> weapons;
        std::vector<std::pair<double, int>> shieldPotions;
        for (size_t i = 0; i != game.loot.size(); ++i) {
            const auto &loot = game.loot[i];
            lootById[loot.id] = &loot;
            const double distance = (unit->position - loot.position).norm();
            switch (loot.tag) {
                case LootType::Weapon:
                    weapons.emplace_back(distance, i);
                    break;
                case LootType::ShieldPotions:
                    shieldPotions.emplace_back(distance, i);
                    break;
                case LootType::Ammo:
                    ammos.emplace_back(distance, i);
                    break;
            }
        }
        std::sort(weapons.begin(), weapons.end());
        std::sort(ammos.begin(), ammos.end());
        std::sort(shieldPotions.begin(), shieldPotions.end());
        const auto pickTask = [&unit, &visibilityFilters, &tasks, &myUnits, &evalPathDanger, &zoneMover]
                (const double distance, const Loot &loot, const double priority) {
            if (distance > Constants::INSTANCE.unitRadius &&
                zoneMover.zone.currentRadius > Constants::INSTANCE.unitRadius * 2) {
                auto ticksToMove = (size_t) round(distance / Constants::INSTANCE.maxUnitForwardSpeed * 1.15);
                const double distanceTo =
                        zoneMover.DistanceFromZone(loot.position, ticksToMove) - Constants::INSTANCE.unitRadius;
                if (distanceTo < 0.) {
                    return false;
                }
            }

            DRAW({
                     if (unit != myUnits[0]) {
                         return;
                     }
                     debugInterface->addPlacedText(loot.position, to_string_p(priority, 2), {0.5, 0.5}, 0.1,
                                                   debugging::Color(0., 0., 0., 1.));
                 });

            if (distance <= Constants::INSTANCE.unitRadius && !unit->action.has_value() &&
                !unit->remainingSpawnTime.has_value()) {
                Task pickTask{6, unit->id,
                              std::to_string(unit->id) + " pick up " + std::to_string(loot.id) + "|" +
                              loot.position.toString(),
                              {OrderType::kAction}};
                pickTask.score = priority;
                if (unit->aim < 1e-5) {
                    pickTask.score *= 10.;
                }
                pickTask.taskData = std::make_pair(loot.id, unit->id);
                pickTask.func = [unit, &loot](const std::any &evalData, POrder &order) -> std::vector<OrderType> {
                    return ApplyPickUp(*unit, loot, order);
                };
                tasks.push(pickTask);
            }
            if (distance > Constants::INSTANCE.unitRadius || unit->action.has_value() ||
                unit->remainingSpawnTime.has_value() ||
                unit->aim > 1e-5) {
                bool inside = distance <= Constants::INSTANCE.unitRadius;
                Task moveTask{6, unit->id,
                              std::to_string(unit->id) + " move to pick up " + std::to_string(loot.id) + "|" +
                              loot.position.toString(),
                              {OrderType::kMove}};
                moveTask.score = priority / std::max(distance, 1.);
                moveTask.taskData = std::make_pair(loot.id, unit->id);
                //const std::any &evalData,
                moveTask.preEval = [unit, &loot, &evalPathDanger, inside]() -> std::tuple<double, std::any> {
                    return {evalPathDanger(unit->position, loot.position),
                            DestinationWithMaxSpeed{loot.position,
                                                    inside ? 1. : std::numeric_limits<double>::infinity()}};
                };

                moveTask.func = [unit, &visibilityFilters](
                        const std::any &evalData, POrder &order) -> std::vector<OrderType> {
                    const auto &[position, maxSpeed] = std::any_cast<DestinationWithMaxSpeed>(evalData);
                    return ApplyMoveTo(*unit, position, visibilityFilters[unit->id], maxSpeed, order);
                };
                tasks.push(moveTask);
            }
            return true;
        };
        const auto pickWeapon =
                [&weapons, &game, &pickTask, unit](int weaponType, double priority) {
                    if (priority <= 0.) {
                        return;
                    }
                    size_t count = 10;
                    for (const auto &weapon: weapons) {
                        const Loot &loot = game.loot[weapon.second];
                        if (loot.weaponTypeIndex != weaponType) {
                            continue;
                        }
                        if (pickTask((unit->position - loot.position).norm(), loot, priority)) {
                            if (--count == 0) {
                                return;
                            }
                        }
                    }
                };
        {
            std::vector<double> priority;
            constexpr double kRelativeWeight[] = {100., 1000., 10000.};
            for (int i = 0; i != 3; ++i) {
                priority.push_back(
                        (kRelativeWeight[i] * unit->ammo[i]) / Constants::INSTANCE.weapons[i].maxInventoryAmmo);
            }
            for (int i = 0; i != 3; ++i) {
                pickWeapon(i, priority[i] - (unit->weapon ? priority[*unit->weapon] : 0));
            }
            for (int i = 0; i != 3; ++i) {
                priority[i] = kRelativeWeight[i] - priority[i];
                if (*unit->weapon == i) {
                    priority[i] *= 2;
                }
                const int diff = constants.weapons[i].maxInventoryAmmo - unit->ammo[i];
                if (diff == 0) {
                    continue;
                }
                const double portion = diff / (double) constants.weapons[i].maxInventoryAmmo;
                for (auto &item: ammos) {
                    const Loot &loot = game.loot[item.second];
                    if (loot.weaponTypeIndex != i) {
                        continue;
                    }
                    pickTask((unit->position - loot.position).norm(), loot,
                             priority[i] * std::min(loot.amount, diff) * sqr(portion) /
                             Constants::INSTANCE.weapons[i].maxInventoryAmmo);
                }
            }
        }

        if (unit->shield + constants.shieldPerPotion <= constants.maxShield && unit->shieldPotions > 0) {
            Task useShieldPotion{8, unit->id, std::to_string(unit->id) + " use shield potion", {OrderType::kAction}};
            useShieldPotion.score = 800. * (constants.maxShield - unit->shield) / constants.shieldPerPotion;
            useShieldPotion.func = [](const std::any &evalData, POrder &order) -> std::vector<OrderType> {
                order.action = std::make_shared<ActionOrder::UseShieldPotion>();
                return {OrderType::kAction};
            };
            tasks.push(useShieldPotion);
            const double currentDanger = EvaluateDanger(unit->position, dangerMatrix, game);
            if (currentDanger > 0.2 && (unit->weapon.has_value() &&
                unit->ammo[*unit->weapon] * constants.weapons[*unit->weapon].projectileDamage > 200. || currentDanger > 1.)) { // just survive
                Task battleMovementWhileDrinking{12, unit->id, std::to_string(unit->id) + " battle movement",
                                                 {OrderType::kMove}};
                battleMovementWhileDrinking.score = useShieldPotion.score;
                battleMovementWhileDrinking.func = [unit, &dangerMatrix, &game, &visibilityFilters](
                        const std::any &evalData, POrder &order) -> std::vector<OrderType> {
                    return ApplyBattleMovement(*unit, dangerMatrix, game, visibilityFilters[unit->id], order);
                };
                tasks.push(battleMovementWhileDrinking);
            }
        }

        if (unit->shieldPotions < constants.maxShieldPotionsInInventory) {
            int diff = constants.maxShieldPotionsInInventory - unit->shieldPotions;
            if (unit->action.has_value() && unit->action->actionType == ActionType::USE_SHIELD_POTION) {
                ++diff;
            }
            for (auto &item: shieldPotions) {
                const Loot &loot = game.loot[item.second];
                pickTask((unit->position - loot.position).norm(), loot, 10 * std::min(loot.amount, diff) * sqr(diff));
            }
        }
    }
    /** ==================================================================================
     *  Danger control
     *  ==================================================================================
     */
    for (const Unit *unit: myUnits) {
        auto &dangerList = unitDanger[unit->id];

        double dangerSum = 0.;
        for (auto &[id, danger]: dangerList) {
            dangerSum += danger;
        }
        if (dangerSum < 1e-2) {
            continue;
        }
        auto &filteredObstacles = visibilityFilters[unit->id].closeObstacles;

        const auto directionBonus = [moveDirection = unit->velocity.sqrNorm() > 1e-5 ? unit->velocity.toRadians() : 0](
                auto direction) {
            return 1e-5 * (1. - std::abs(AngleDiff(direction, moveDirection)) / M_PI);
        };
//        const auto currDirection = unit->direction.toRadians();
        for (const auto &enUnit: enemyUnits) {
            const Vec2 diff = enUnit->position - unit->position;
            if (diff.sqrNorm() > sqr(55.)) {
                continue;
            }

            const auto angle = (enUnit->position - unit->position).toRadians();
            auto [pointForRight, pointForLeft] =
                    diff.sqrNorm() < sqr(constants.unitRadius * 2) ?
                    std::make_pair(enUnit->position, enUnit->position) :
                    TangentialPoints(unit->position, enUnit->position,
                                     Constants::INSTANCE.unitRadius);
            constexpr double kAdditionalAngleToControl = M_PI / 180;
            const double pointProposedRight = AddAngle((pointForRight - unit->position).toRadians(),
                                                       kAdditionalAngleToControl);
            const double proposedRightAngle = SubstractAngle(pointProposedRight, unit->currentFieldOfView);
            const double pointProposedLeft = SubstractAngle((pointForLeft - unit->position).toRadians(),
                                                            kAdditionalAngleToControl);
            const double proposedLeftAngle = AddAngle(pointProposedLeft, unit->currentFieldOfView);
            const auto GetDanger = [&enemyUnits, &dangerList, &unit, &filteredObstacles]
                    (double leftAngle, double rightAngle) {
                double danger = 0.;
                for (const auto &checkUnit: enemyUnits) {
                    if (IsVisible<VisionFilter::kVisibilityFilter>(unit->position, leftAngle, rightAngle,
                                                                   checkUnit->position, filteredObstacles)) {
                        danger += dangerList[checkUnit->id];
                    }
                }
                return danger;
            };
            {
                Task dangerControl{9, unit->id,
                                   std::to_string(unit->id) + " right danger control of " + std::to_string(enUnit->id),
                                   {OrderType::kRotate}};
                const double resultAngle = SubstractAngle(pointProposedRight, unit->currentFieldOfView / 2.);
                const double dirBonus = directionBonus(resultAngle);
                const double danger = GetDanger(proposedRightAngle, pointProposedRight);
                dangerControl.score = danger + dirBonus;
                dangerControl.func = [unit, resultAngle](const std::any &evalData,
                                                         POrder &order) -> std::vector<OrderType> {
                    return ApplyLookTo(unit->position + Vec2{resultAngle} * 30., order);
                };
                tasks.push(dangerControl);
            }
            {
                Task dangerControl{9, unit->id,
                                   std::to_string(unit->id) + " left danger control of " + std::to_string(enUnit->id),
                                   {OrderType::kRotate}};
                const double resultAngle = AddAngle(pointProposedLeft, unit->currentFieldOfView / 2.);
                const double dirBonus = directionBonus(resultAngle);
                const double danger = GetDanger(pointProposedLeft, proposedLeftAngle);
                dangerControl.score = danger + dirBonus;
                dangerControl.func = [unit, resultAngle](const std::any &evalData,
                                                         POrder &order) -> std::vector<OrderType> {
                    return ApplyLookTo(unit->position + Vec2{resultAngle} * 30., order);
                };
                tasks.push(dangerControl);
            }
            {
                Task dangerControl{9, unit->id,
                                   std::to_string(unit->id) + " center danger control of " + std::to_string(enUnit->id),
                                   {OrderType::kRotate}};
                const double dirBonus = directionBonus(angle);
                const double danger = GetDanger(angle - unit->currentFieldOfView / 2.,
                                                angle + unit->currentFieldOfView / 2.);
                dangerControl.score = danger + dirBonus;
                dangerControl.func = [unit, angle](const std::any &evalData, POrder &order) -> std::vector<OrderType> {
                    return ApplyLookTo(unit->position + Vec2{angle} * 30., order);
                };
                tasks.push(dangerControl);
            }
        }
    }
    /** ==================================================================================
     *  Radar
     *  ==================================================================================
     */
    for (const Unit *unit: myUnits) {
        auto &[radarState, startingAngle, lastEndTick] = radarTaskData[unit->id];
        if (unit->remainingSpawnTime.has_value() && *unit->remainingSpawnTime > 0.2) {
            Task radarTask{10, unit->id, std::to_string(unit->id) + " radar mode", {OrderType::kRotate}};
            radarTask.score = 1e3;
            radarTask.func = [unit](const std::any &evalData, POrder &order) -> std::vector<OrderType> {
                return ApplyLookTo(unit->position + unit->direction.clone().rotate90() * 30., order);
            };
            tasks.push(radarTask);
            lastEndTick = game.currentTick;
            continue;
        }
        constexpr int kTicksToRepeatRadar = 90;
        if (radarState == 2 &&
            std::abs(AngleDiff(startingAngle, unit->direction.toRadians())) < unit->currentFieldOfView * .5) {
            // got full rotation
            lastEndTick = game.currentTick;
        }
        radarState = radarState == 2 ? 1 : 0;
        if (lastEndTick + kTicksToRepeatRadar > game.currentTick) {
            // last task finished previously
            continue;
        }

        Task radarTask{10, unit->id, std::to_string(unit->id) + " radar mode", {OrderType::kRotate}};
        radarTask.score = -0.5;// + unknownIncomingDamageSum[unit->id];
        radarTask.func = [unit, &radarState = radarState, &startingAngle = startingAngle](const std::any &evalData,
                                                                                          POrder &order) -> std::vector<OrderType> {
            if (radarState == 0) {
                startingAngle = AddAngle(unit->direction.toRadians(), unit->currentFieldOfView * .5);
            }
            radarState = 2;
            return ApplyLookTo(unit->position + unit->direction.clone().rotate90() * 30., order);
        };
        tasks.push(radarTask);
    }
    TimeMeasure::end(5);

    /** ==================================================================================
     *  Тут начинаются сами действия
     *  ==================================================================================
     */
    std::unordered_map<int, POrder> pOrders;
    for (Unit *unit: myUnits) {
        pOrders[unit->id] = POrder();
    }
    std::unordered_map<int, std::vector<std::any>> usedData;
    incomingDamage.clear();
    for (; !tasks.empty(); tasks.pop()) {
        const auto &task = tasks.top();
        POrder &curr = pOrders[task.unitId];
        if (!curr.IsAbleToAcceptTask(task.actionTypes)) {
            continue;
        }
        if (task.taskData.has_value() && [&usedData, &task]() {
            const auto &filter = taskFilter[task.type];
            for (const auto &value: usedData[task.type]) {
                if (filter(task.taskData, value)) {
                    return true;
                }
            }
            return false;
        }()) {
            continue;
        }
#ifdef TICK_DEBUG_ENABLED
        if (task.func == nullptr) {
            std::cerr << "[No function implemented]: " << task.type << ": " << task.description << std::endl;
        }
#endif
        if (task.preEval != nullptr && !task.evalData.has_value()) {
            const auto &[priorityChange, data] = task.preEval();
            if (!data.has_value()) {
                continue;
            }
            auto newTask = task;
            newTask.evalData = data;
            newTask.penalty = priorityChange * 50;
            newTask.score -= newTask.penalty;
            tasks.push(newTask);
            continue;
        }
        if (curr.Accept(task) && task.taskData.has_value()) {
            usedData.try_emplace(task.type).first->second.push_back(task.taskData);
        }
    }
    TimeMeasure::end(6);

    std::unordered_map<int, model::UnitOrder> orders;

    for (Unit *unit: myUnits) {
        const auto &pOrder = pOrders[unit->id];
        MoveRule orderedRule = pOrder.toMoveRule();
        const auto [_, damageScore, firstProjectile] = Simulate(*unit, game, orderedRule);
        if (damageScore == 0.) {
            auto order = ApplyAvoidRule(*unit, orderedRule);
            order.action = pOrder.action;
//            std::cerr << order.toString() << std::endl;
            orders[unit->id] = order;
            debugActions(pOrder, *unit, -1);
            usedAvoidRule.erase(unit->id);
            incomingDamage[unit->id] = 0;
            continue;
        }
        TimeMeasure::end(7);
        std::vector<MoveRule> basicRules = [firstProjectile = firstProjectile, &unit, &usedAvoidRule = this->usedAvoidRule, &orderedRule]() {
            std::vector<MoveRule> avoidRules;
            if (usedAvoidRule.count(unit->id)) {
                avoidRules.push_back(usedAvoidRule[unit->id]);
                avoidRules.back().keepAim = orderedRule.keepAim;
            }
            constexpr double kMoveLength = 30.;
            if (firstProjectile != nullptr) {
                // TODO: possible a better directions
                Vec2 norm = {firstProjectile->velocity.y, -firstProjectile->velocity.x};
                if ((unit->velocity - norm).sqrNorm() > (unit->velocity + norm).sqrNorm()) {
                    norm = -norm;
                }
                avoidRules.push_back(
                        {unit->position + norm.toLen(kMoveLength), orderedRule.lookDirection, orderedRule.keepAim,
                         std::numeric_limits<double>::infinity()});
                avoidRules.push_back(
                        {unit->position - norm.toLen(kMoveLength), orderedRule.lookDirection, orderedRule.keepAim,
                         std::numeric_limits<double>::infinity()});
            }
            for (const auto &dir: kMoveDirections) {
                avoidRules.push_back(
                        {unit->position + dir * kMoveLength, orderedRule.lookDirection, orderedRule.keepAim,
                         std::numeric_limits<double>::infinity()});
            }
            return avoidRules;
        }();

        std::vector<ComplexMoveRule> complexRules;
        complexRules.emplace_back(orderedRule);
        for (const auto &rule: basicRules) {
            complexRules.emplace_back(
                    ComplexMoveRule({{orderedRule, 1},
                                     {{rule.moveDirection, rule.moveDirection, false, std::numeric_limits<double>::infinity()},
                                                   0}}));
        }
        for (const auto &rule: basicRules) {
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

        auto [score, ruleId] = ChooseBest(*unit, game, complexRules, dangerMatrix);

        const auto [resultUnit, dScore, _2] = Simulate(*unit, game, complexRules[ruleId], 1);
        incomingDamage[unit->id] = (unit->shield + unit->health) - (resultUnit.shield + resultUnit.health);
        auto order = ApplyAvoidRule(*unit, complexRules[ruleId].storage.front());
        if (pOrder.action.has_value() &&
            (complexRules[ruleId].storage.front().keepAim == orderedRule.keepAim ||
             std::dynamic_pointer_cast<ActionOrder::Aim>(*pOrder.action) == nullptr)) {
            auto aimMessage = std::dynamic_pointer_cast<ActionOrder::Aim>(*pOrder.action);
            if (aimMessage != nullptr && aimMessage->shoot) {
                aimMessage->shoot &=
                        (*complexRules[ruleId].storage.front().lookDirection - *orderedRule.lookDirection).sqrNorm() <
                        1e-10;
            }
            order.action = pOrder.action;
        }
        orders[unit->id] = order;
        usedAvoidRule[unit->id] = complexRules[ruleId].storage.back();
        debugActions(pOrder, *unit, (int) ruleId);
        TimeMeasure::end(8);
    }
    TimeMeasure::end(7);

    // remove picker loot
    std::unordered_set<int> pickedIds;
    for (auto &[unitId, order]: orders) {
        if (!order.action) {
            continue;
        }
        auto pickup = std::dynamic_pointer_cast<ActionOrder::Pickup>(*order.action);
        if (!pickup) {
            continue;
        }
        auto loot = lootById[pickup->loot];
        auto unit = unitById[unitId];
        if (unit->playerId == game.myId && !unit->remainingSpawnTime.has_value() &&
            (unit->position - loot->position).sqrNorm() <= sqr(constants.unitRadius) &&
            !unit->action.has_value()) {
            if (unit->aim < 1e-5) {
                pickedIds.insert(pickup->loot);
            }
#ifdef DEBUG_INFO
        } else {
            std::cerr << "Unit " << unitId << " can't pick up loot with id " << loot->id << std::endl;
#endif
        }
    }
    for (size_t i = 0; !pickedIds.empty() && i != game.loot.size(); ++i) {
        auto &id = game.loot[i].id;
        if (pickedIds.count(id)) {
            pickedIds.erase(id);
            game.loot[i] = game.loot.back();
            game.loot.pop_back();
            --i;
        }
    }

//    DRAW({
//             double minVal = std::numeric_limits<double>::infinity();
//             double maxVal = -std::numeric_limits<double>::infinity();
//             for (size_t i = 0; i != dangerMatrix.size(); ++i) {
//                 for (size_t j = 0; j != dangerMatrix[0].size(); ++j) {
//                     if (dangerMatrix[i][j].first != game.currentTick) {
//                         continue;
//                     }
//                     minVal = std::min(dangerMatrix[i][j].second, minVal);
//                     maxVal = std::max(dangerMatrix[i][j].second, maxVal);
//                 }
//             }
//             double diffVal = 1. / (maxVal - minVal);
//
//             debugInterface->setAutoFlush(false);
//             for (int i = 0; i != dangerMatrix.size(); ++i) {
//                 for (int j = 0; j != dangerMatrix[0].size(); ++j) {
//                     if (dangerMatrix[i][j].first != game.currentTick) {
//                         continue;
//                     }
//                     debugInterface->addRect({constants.minX + i - 0.5, constants.minY + j - 0.5}, {1., 1.},
//                                             debugging::Color((dangerMatrix[i][j].second - minVal) * diffVal, 0., 0.,
//                                                              .8));
//                     debugInterface->addPlacedText({constants.minX + i - 0.5, constants.minY + j + 0.5},
//                                                   to_string_p(dangerMatrix[i][j].second, 4), {0., 1.},
//                                                   0.05, debugging::Color(1., 1., 1., 0.9));
//                 }
//             }
//             debugInterface->flush();
//             debugInterface->setAutoFlush(true);
//         });
//    DRAWK('K', {
//             for (size_t i = 0; i != lastSeenArray.size(); ++i) {
//                 for (size_t j = 0; j != lastSeenArray[0].size(); ++j) {
//                     auto pos = GetLastSeenCoord(i, j);
//                     const int val = lastSeenArray[i][j];
//                     double strength = val == -1 ? 1. : (game.currentTick - val) / 100000.;
//                     debugInterface->addRect(pos - Vec2(kLastSeenArrayStep / 2., kLastSeenArrayStep / 2.),
//                                             Vec2(kLastSeenArrayStep, kLastSeenArrayStep),
//                                             debugging::Color(0., 0., 0., strength * .5));
//                 }
//             }
//         });

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
    TimeMeasure::printTimings();
}