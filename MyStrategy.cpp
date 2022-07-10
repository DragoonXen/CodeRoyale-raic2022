#include "MyStrategy.hpp"
#include "utils/Movement.hpp"
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

    void UpdateLifetime(Projectile &projectile) {
        const Constants &constants = Constants::INSTANCE;
        double remaining_time = projectile.lifeTime;
        Vec2 position = projectile.position;
        while (remaining_time > 1e-10) {
            const double currSimTime = std::min(constants.tickTime, remaining_time);
            Vec2 next_pos = position + projectile.velocity * currSimTime;
            const auto &[obstacle, point] = GetClosestCollision<true>(position, next_pos, 0);
            if (obstacle) {
                projectile.lifeTime = projectile.lifeTime - remaining_time
                                      + (point - position).norm() / (next_pos - position).norm() * currSimTime;
                return;
            }
            remaining_time -= constants.tickTime;
            position = next_pos;
        }
    }

    void UpdateProjectiles(Game &game, std::optional<Game>& last_tick) {
        std::unordered_map<int, Projectile *> from_prev_tick;
        const Constants &constants = Constants::INSTANCE;
        if (last_tick) {
            for (auto &shot: last_tick->projectiles) {
                shot.lifeTime -= constants.tickTime;
                if (shot.lifeTime > 0.) {
                    from_prev_tick[shot.id] = &shot;
                }
            }
        }

        for (auto &curr_shot: game.projectiles) {
            if (from_prev_tick.count(curr_shot.id)) {
                curr_shot.lifeTime = from_prev_tick[curr_shot.id]->lifeTime;
                from_prev_tick.erase(curr_shot.id);
            } else {
                UpdateLifetime(curr_shot);
            }
        }
        for (auto &[_, projectile]: from_prev_tick) {
            projectile->position += projectile->velocity * constants.tickTime;
            game.projectiles.push_back(*projectile);
        }
        DRAW(
                for (auto &curr_shot: game.projectiles) {
                    debugInterface->addGradientSegment(curr_shot.position, debugging::Color(1., 0., 0., .8),
                                                       curr_shot.position +
                                                       curr_shot.velocity * curr_shot.lifeTime,
                                                       debugging::Color(0., 1., 0., .8),
                                                       .1);
                }
        );
    }

    std::tuple<Unit, double, const Projectile *>
    Simulate(Unit unit, const Game &game, const Vec2 &moveDirection, std::optional<Vec2> lookDirection,
             bool keepAim,
             const double speedLimit = Constants::INSTANCE.maxUnitForwardSpeed,
             size_t deep = 20) {
        const auto &constants = Constants::INSTANCE;
        double damagePenalty = 0.;
        Vec2 last_position;
        std::vector<const Projectile *> remained_projectiles(game.projectiles.size());
        const Projectile* firstProjectile = nullptr;
        for (size_t i = 0; i != game.projectiles.size(); ++i) {
            remained_projectiles[i] = &game.projectiles[i];
        }
        for (size_t tick = 0; tick != deep; ++tick) {
            last_position = unit.position;
            if (lookDirection) {
                unit.direction = applyNewDirection(unit.direction, *lookDirection - unit.position,
                                                   rotationSpeed(unit.aim, unit.weapon));
            }
            unit.aim = CalcResultAim(keepAim, unit.aim, unit.weapon);

            const auto vector = MaxSpeedVector(unit.position, unit.direction, moveDirection,
                                               CalcAimSpeedModifier(unit)).LimitLength(speedLimit);
            unit.velocity = ResultSpeedVector(unit.velocity, vector);
            updateForCollision(unit.position, unit.velocity);

            const double tickTime = constants.tickTime;
            const double passedTime = tickTime * tick;
            double damage = 0.;
            for (size_t i = 0; i != remained_projectiles.size(); ++i) {
                const auto projectile = remained_projectiles[i];
                if (projectile->lifeTime <= passedTime) {
                    continue;
                }
                const double simTime = std::min(projectile->lifeTime - passedTime, tickTime);
                const Vec2 unitVelocity = (unit.position - last_position) * (1. / tickTime);
                if (!IsCollide(last_position, unitVelocity, projectile->position + projectile->velocity * passedTime,
                               projectile->velocity, simTime, constants.unitRadius + 1e-5)) {
                    continue;
                }
                remained_projectiles[i] = remained_projectiles.back();
                remained_projectiles.pop_back();
                --i;
                if (unit.playerId == projectile->shooterPlayerId && !constants.friendlyFire) {
                    continue;
                }
                if (firstProjectile == nullptr) {
                    firstProjectile = projectile;
                }
                damage += constants.weapons[projectile->weaponTypeIndex].projectileDamage;
            }
            DRAW(
                    debugInterface->addCircle(unit.position, constants.unitRadius,
                                              damage == 0 ? debugging::Color(1., .7, 0., .1) : debugging::Color(1., 0.,
                                                                                                                0.,
                                                                                                                .1));
            );
            constexpr double damageExp = 1.001;
            damagePenalty *= damageExp;
            damagePenalty += std::min(damage, unit.health + unit.shield);
            ApplyDamage(unit, damage, game.currentTick);
            if (unit.health < 1e-3) {
                for (++tick; tick != deep; ++tick) {
                    damagePenalty *= damageExp;
                }
                return {unit, damagePenalty, firstProjectile};
            }
        }

        return {unit, damagePenalty, firstProjectile};
    }

    struct AvoidRule {
        Vec2 moveDirection;
        std::optional<Vec2> lookDirection;
        bool keepAim;
        double speedLimit;
    };

    size_t ChooseBest(const Unit &unit, const Game &game, const std::vector<AvoidRule> &rules) {
        double minScore = std::numeric_limits<double>::infinity();
        size_t best_id = 0;
        for (size_t id = 0; id != rules.size(); ++id) {
            const auto& rule = rules[id];
            const auto [_, score, _2] = Simulate(unit, game, rule.moveDirection, rule.lookDirection, rule.keepAim,
                                                rule.speedLimit);
            if (score < minScore) {
                if (score == 0.) {
                    return id;
                }
                minScore = score;
                best_id = id;
            }
        }
        return best_id;
    }
}

MyStrategy::MyStrategy(const model::Constants &constants) {
    Constants::INSTANCE = constants;
    Constants::INSTANCE.Update();
}

model::Order MyStrategy::getOrder(const model::Game &game_base, DebugInterface *debugInterface) {
    DebugInterface::INSTANCE = debugInterface;
    Game game = game_base;
//    std::cerr << game.currentTick << std::endl;
    for (auto &unit: game.units) {
        DRAW(
                debugInterface->addPlacedText(unit.position + model::Vec2{1., 1.},
                                              "unit " + std::to_string(unit.id) + "\n" + to_string_p(unit.shield, 1) +
                                              "|" + to_string_p(unit.health, 1) + "\n" +
                                              to_string_p(unit.position.x, 2) + "|" + to_string_p(unit.position.y, 2) +
                                              "\n",
                                              {0., 0.}, 1,
                                              debugging::Color(0, 0, 0, 1));
        );
    }

    const auto my_units_filter = [id = game.myId](const auto &unit) { return unit.playerId == id; };
    const auto enemies_filter = [id = game.myId](const auto &unit) { return unit.playerId != id; };

    auto my_units = filterUnits(game.units, my_units_filter);
    auto enemy_units = filterUnits(game.units, enemies_filter);

    UpdateProjectiles(game, last_tick_game);

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
                const auto &selected_rule = avoidRules[rule_id];
                model::UnitOrder order;
                if (selected_rule.lookDirection) {
                    unit->direction = applyNewDirection(unit->direction, *selected_rule.lookDirection - unit->position,
                                                        rotationSpeed(unit->aim, unit->weapon));
                }
                order.targetDirection = unit->direction;

                const auto velocity = MaxSpeedVector(unit->position, unit->direction, selected_rule.moveDirection,
                                                     CalcAimSpeedModifier(*unit));
                if (sqr(selected_rule.speedLimit) < velocity.sqrNorm()) {
                    velocity.toLen(selected_rule.speedLimit);
                }
                order.targetVelocity = velocity;
                unit->velocity = ResultSpeedVector(unit->velocity, velocity);
                updateForCollision(unit->position, unit->velocity);
                orders[unit->id] = std::move(order);
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

void MyStrategy::debugUpdate(DebugInterface &debugInterface) {}

void MyStrategy::finish() {}