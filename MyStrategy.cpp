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
}

MyStrategy::MyStrategy(const model::Constants &constants) {
    TimeMeasure::start();
    Constants::INSTANCE = constants;
    Constants::INSTANCE.Update();
    TimeMeasure::end(0);
}

model::Order MyStrategy::getOrder(const model::Game &game_base, DebugInterface *debugInterface) {
#ifdef DEBUG_INFO
    if (game_base.currentTick == 0) {
        debugging::DebugState::processKeys({"I"});
    }
#endif
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

    {
        TimeMeasure::end(1);
        auto projectilesUnitInfo = UpdateProjectiles(game, last_tick_game, myUnits, visibilityFilters);
        TimeMeasure::end(2);
        UpdateLoot(game, last_tick_game, myUnits, visibilityFilters);
        TimeMeasure::end(3);
        UpdateUnits(game, last_tick_game, myUnits, visibilityFilters, std::move(projectilesUnitInfo));
    }
    TimeMeasure::end(4);
    myUnits = filterUnits(game.units, my_units_filter);
    auto enemyUnits = filterUnits(game.units, enemies_filter);

    std::unordered_map<int, std::unordered_map<int, double>> unitDanger;
    for (const auto& unit: myUnits) {
        unitDanger[unit->id] = std::unordered_map<int, double>();
        auto &myUnitMap = unitDanger[unit->id];
        for (const auto &enemyUnit: enemyUnits) {
             double danger = 1.;
            const Vec2 distanceVec = unit->position - enemyUnit->position;
            // angle danger
            const double angleDiff = std::abs(AngleDiff(enemyUnit->direction.toRadians(), distanceVec.toRadians()));
            // normalize from 0 to M_PI * 5 / 6
            const auto diff = std::min(std::max(angleDiff - (M_PI / 6), 0.), M_PI * 2 / 3);
            danger *= 1. - diff / (M_PI * 5 / 6);
            // distance danger
            constexpr double maxDangerDistance = 7.;
            const double distance = distanceVec.norm();
            const double dangerDistanceCheck = maxDangerDistance / std::max(distance, maxDangerDistance);
            danger *= dangerDistanceCheck;
            // weapon danger
            if (enemyUnit->weapon && enemyUnit->ammo[*enemyUnit->weapon] > 0) {
                constexpr double kWeaponDanger[] = {0.34, .6666, 1.};
                danger *= kWeaponDanger[*enemyUnit->weapon];
                if (*enemyUnit->weapon == 2 && distance < 10.) {
                    danger *= 1.55;
                }
            } else {
                danger *= 1e-2;
            }
            myUnitMap[enemyUnit->id] = danger;
//            DRAW({
//                     debugInterface->addPlacedText(enemyUnit->position + distanceVec * .5,
//                                                   std::to_string(enemyUnit->id) + " danger " + std::to_string(danger),
//                                                   {0.5, 0.5}, 0.7,
//                                                   debugging::Color(0.8, 0, 0, 1.));
//                 });
        }
    }

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

    Vec2 centerPoint = {0., 0.};
    for (auto& unit : myUnits) {
        centerPoint += unit->position;
    }
    centerPoint *= (1. / myUnits.size());

    std::unordered_map<int, const Unit *> unitById;
    const std::vector<Unit> unitsBackup = game.units;
    for (const auto& unit : unitsBackup) {
        unitById[unit.id] = &unit;
    }
    std::unordered_map<int, const Loot*> lootById;
    std::vector<std::pair<double, int>> ammos;
    std::vector<std::pair<double, int>> weapons;
    std::vector<std::pair<double, int>> shieldPotions;
    for (size_t i = 0; i != game.loot.size(); ++i) {
        const auto& loot = game.loot[i];
        lootById[loot.id] = &loot;
        const double distance = (centerPoint - loot.position).norm();
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
                distances.emplace_back((unit->position - enUnit->position).sqrNorm(), i);
            }
            std::sort(distances.begin(), distances.end());
            for (const auto& item : distances) {
                 const auto& enUnit = enemyUnits[item.second];
                Task moveTask{1, unit->id,
                              std::to_string(unit->id) + " move to attack enemy " + std::to_string(enUnit->id),
                              {OrderType::kMove}};
                constexpr double kMaxDangerDistanceSqr = 64.;
                moveTask.score = kMaxDangerDistanceSqr * 1000. / std::max(kMaxDangerDistanceSqr, item.first);
                if (!IsVisible<VisionFilter::kShootFilter>(unit->position, unit->direction, unit->currentFieldOfView,
                                                           enUnit->position, visibilityFilters[unit->id])) {
                    moveTask.score /= 10.;
                }
                if (enUnit->shield + enUnit->health < constants.weapons[*unit->weapon].projectileDamage + 1e-5) {
                    moveTask.score *= 100.;
                }
//                if (!IsReachable(unit->position, enUnit->position, visibilityFilters[unit->id])) {
//                    moveTask.score /= 10.;
//                }
                moveTask.func = [unit, enUnit, &visibilityFilters](POrder &order) -> std::vector<OrderType> {
                    return ApplyMoveToUnitTask(*unit, *enUnit, visibilityFilters, order);
                };
                tasks.push(moveTask);
                if (item.first > 30. * 30.) { // no attack task fot this case. Going to danger control task, type 9
                    continue;
                }
                Task attackTask{0, unit->id,
                                std::to_string(unit->id) + " attack enemy " + std::to_string(enUnit->id),
                                {OrderType::kAction, OrderType::kRotate}};

                attackTask.score = moveTask.score;
                attackTask.func = [unit, enUnit, tick = game.currentTick, &visibilityFilters](POrder &order) -> std::vector<OrderType> {
                    return ApplyAttackTask(*unit, *enUnit, tick, visibilityFilters, order);
                };
                tasks.push(attackTask);
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
        Task moveTask{4, unit->id, std::to_string(unit->id) + " Explore zone ", {OrderType::kMove}};
        moveTask.score = 0.;
        moveTask.func = [unit, &zone = game.zone, &filter = visibilityFilters[unit->id]](
                POrder &order) -> std::vector<OrderType> {
            Vec2 zoneDst = unit->position - zone.currentCenter;
            if (zoneDst.sqrNorm() < 1.) {
                zoneDst = {1., 0};
            }
            const Vec2 moveDirection =
                    zone.currentCenter + Vec2(zoneDst.toRadians() + M_PI / 6).toLen(
                            std::max(2., zone.currentRadius - Constants::INSTANCE.viewDistance));
            return ApplyMoveTo(*unit, moveDirection, filter, std::numeric_limits<double>::infinity(), order);
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
    // loot && potions usage
    for (const Unit *unit: myUnits) {
        const auto pickTask = [&unit, &visibilityFilters, &tasks, &game](const double distance, const Loot &loot,
                                                                  const double priority) {
            if ((loot.position - game.zone.currentCenter).sqrNorm() >= sqr(game.zone.currentRadius)) {
                return false;
            }
            if (distance <= Constants::INSTANCE.unitRadius && !unit->action && !unit->remainingSpawnTime) {
                Task pickTask{6, unit->id,
                              std::to_string(unit->id) + " pick up " + std::to_string(loot.id) + "|" +
                              loot.position.toString(),
                              {OrderType::kAction}};
                pickTask.score = priority;
                pickTask.func = [unit, &loot](
                        POrder &order) -> std::vector<OrderType> {
                    return ApplyPickUp(*unit, loot, order);
                };
                tasks.push(pickTask);
            } else {
                Task moveTask{7, unit->id,
                              std::to_string(unit->id) + " move to pick up " + std::to_string(loot.id) + "|" +
                              loot.position.toString(),
                              {OrderType::kMove}};
                moveTask.score = priority / std::max(distance, 1.);
                moveTask.func = [unit, &loot, &visibilityFilters](
                        POrder &order) -> std::vector<OrderType> {
                    return ApplyMoveTo(*unit, loot.position, visibilityFilters[unit->id],
                                       std::numeric_limits<double>::infinity(), order);
                };
                tasks.push(moveTask);
            }
            return true;
        };
        const auto pickWeapon =
                [&weapons, &game, &pickTask](int weaponType, double priority) {
                     if (priority <= 0.) {
                        return;
                    }
                    for (const auto &weapon: weapons) {
                        const Loot &loot = game.loot[weapon.second];
                        if (loot.weaponTypeIndex != weaponType) {
                            continue;
                        }
                        if (pickTask(weapon.first, loot, priority)) {
                            return;
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
                priority[i] = kRelativeWeight[i]  - priority[i];
                if (*unit->weapon == i) {
                    priority[i] *= 2;
                }
                const int diff = constants.weapons[i].maxInventoryAmmo - unit->ammo[i];
                if (diff == 0) {
                    continue;
                }
                const double portion = diff / (double) constants.weapons[i].maxInventoryAmmo;
                int maxAmmo = 0;
                for (auto &item: ammos) {
                    const Loot &loot = game.loot[item.second];
                    if (loot.weaponTypeIndex != i || loot.amount <= maxAmmo) {
                        continue;
                    }
                    if (pickTask(item.first, loot, priority[i] * std::min(loot.amount, diff) * sqr(portion) /
                                                   Constants::INSTANCE.weapons[i].maxInventoryAmmo)) {
                        maxAmmo = loot.amount;
                        if (maxAmmo >= diff) {
                            break;
                        }
                    }
                }
            }
        }

        if (unit->shield + constants.shieldPerPotion <= constants.maxShield && unit->shieldPotions > 0) {
            Task useShieldPotion{8, unit->id, std::to_string(unit->id) + " use shield potion ", {OrderType::kAction}};
            useShieldPotion.score = 800. * (constants.maxShield - unit->shield) / constants.shieldPerPotion;
            useShieldPotion.func = [](
                    POrder &order) -> std::vector<OrderType> {
                order.action = std::make_shared<ActionOrder::UseShieldPotion>();
                return {OrderType::kAction};
            };
            tasks.push(useShieldPotion);
        }

        if (unit->shieldPotions < constants.maxShieldPotionsInInventory) {
            int diff = constants.maxShieldPotionsInInventory - unit->shieldPotions;
            // 200
            int maxPotions = 0;
            for (auto &item: shieldPotions) {
                const Loot &loot = game.loot[item.second];
                if (loot.amount <= maxPotions) {
                    continue;
                }
                maxPotions = loot.amount;
                pickTask(item.first, loot, 200 * std::min(loot.amount, diff) * sqr(diff));
                if (maxPotions >= diff) {
                    break;
                }
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
        auto& filteredObstacles = visibilityFilters[unit->id].closeObstacles;

        const auto directionBonus = [moveDirection = unit->velocity.sqrNorm() > 1e-5 ? unit->velocity.toRadians() : 0](
                auto direction) {
            return 1e-5 * (1. - std::abs(AngleDiff(direction, moveDirection)) / M_PI);
        };
//        const auto currDirection = unit->direction.toRadians();
        for (const auto& enUnit : enemyUnits) {
            const Vec2 diff = enUnit->position - unit->position;
            if (diff.sqrNorm() > sqr(55.)) {
                continue;
            }

            const auto angle = (enUnit->position - unit->position).toRadians();
            const auto proposedRightAngle = SubstractAngle(angle, unit->currentFieldOfView);
            const auto proposedLeftAngle = AddAngle(angle, unit->currentFieldOfView);
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
                const double resultAngle = SubstractAngle(angle, unit->currentFieldOfView / 2.);
                const double dirBonus = directionBonus(resultAngle);
                const double danger = GetDanger(proposedRightAngle, angle);
                dangerControl.score = danger + dirBonus;
                dangerControl.func = [unit, resultAngle](POrder &order) -> std::vector<OrderType> {
                    return ApplyLookTo(unit->position + Vec2{resultAngle} * 30., order);
                };
                tasks.push(dangerControl);
            }
            {
                Task dangerControl{9, unit->id,
                                   std::to_string(unit->id) + " left danger control of " + std::to_string(enUnit->id),
                                   {OrderType::kRotate}};
                const double resultAngle = AddAngle(angle, unit->currentFieldOfView / 2.);
                const double dirBonus = directionBonus(resultAngle);
                const double danger = GetDanger(angle, proposedLeftAngle);
                dangerControl.score = danger + dirBonus;
                dangerControl.func = [unit, resultAngle](POrder &order) -> std::vector<OrderType> {
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
                dangerControl.func = [unit, angle](POrder &order) -> std::vector<OrderType> {
                    return ApplyLookTo(unit->position + Vec2{angle} * 30., order);
                };
                tasks.push(dangerControl);
            }
        }
    }
    TimeMeasure::end(5);

    /** ==================================================================================
     *  Тут начинаются сами действия
     *  ==================================================================================
     */
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
#ifdef TICK_DEBUG_ENABLED
        if (task.func == nullptr) {
            std::cerr << "[No function implemented]: " << task.type << ": " << task.description << std::endl;
        }
#endif
        curr.Accept(task.func(curr), task.type, task.description);
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
            debugActions(pOrder, *unit, -1);
            usedAvoidRule.erase(unit->id);
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
        for (const auto &rule: basicRules) {
            complexRules.emplace_back(
                    ComplexMoveRule({{orderedRule, 1},
                                     {{rule.moveDirection, rule.moveDirection, false, std::numeric_limits<double>::infinity()},
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

        auto [score, ruleId] = ChooseBest(*unit, game, complexRules);
        auto order = ApplyAvoidRule(*unit, complexRules[ruleId].storage.front());
        if (complexRules[ruleId].storage.front().keepAim == orderedRule.keepAim) {
            order.action = pOrder.action;
        }
        orders[unit->id] = order;
        usedAvoidRule[unit->id] = complexRules[ruleId].storage.back();
        debugActions(pOrder, *unit, (int) ruleId);
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


    // remove picker loot
    std::unordered_set<int> pickedIds;
    for (auto& [unitId, order] : orders) {
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
            pickedIds.insert(pickup->loot);
#ifdef DEBUG_INFO
        } else {
            std::cerr << "Unit " << unitId << " can't pick up loot with id " << loot->id << std::endl;
#endif
        }
    }
    for (size_t i = 0; !pickedIds.empty() && i != game.loot.size(); ++i) {
        auto& id = game.loot[i].id;
        if (pickedIds.count(id)) {
            pickedIds.erase(id);
            game.loot[i] = game.loot.back();
            game.loot.pop_back();
            --i;
        }
    }

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