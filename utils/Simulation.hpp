//
// Created by dragoon on 10.07.2022.
//

#include <tuple>
#include "model/Unit.hpp"
#include "model/Projectile.hpp"
#include "model/Game.hpp"
#include "model/Constants.hpp"
#include "Movement.hpp"
#include "DebugInterface.hpp"
#include "model/UnitOrder.hpp"
#include "Visualization.h"

#ifndef AI_CUP_22_SIMULATION_HPP
#define AI_CUP_22_SIMULATION_HPP

namespace {
    using namespace model;
}

struct MoveRule {
    Vec2 moveDirection;
    std::optional<Vec2> lookDirection;
    bool keepAim;
    double speedLimit;
};

struct ComplexMoveRule {
    std::vector<MoveRule> storage;
    std::vector<size_t> usage;

    ComplexMoveRule() : storage{}, usage{} {}

    ComplexMoveRule(MoveRule rule) : storage{rule}, usage{} {}

    ComplexMoveRule(const std::vector<std::pair<MoveRule, size_t>> &toAdd) {
        usage.clear();
        storage.clear();
        for (const auto &pair: toAdd) {
            AddRule(pair.first, pair.second);
        }
    }

    void AddRule(const MoveRule &rule, size_t count = 1) {
        for (size_t i = 0; i != count; ++i) {
            usage.push_back(storage.size());
        }
        storage.push_back(rule);
    }
};

struct ComplexMoveRuleViewer {
    const ComplexMoveRule &rule;
    size_t currStep = 0;

    ComplexMoveRuleViewer(const ComplexMoveRule &rule) : rule(rule) {};

    const MoveRule &nextRule() {
        size_t returnIdx = currStep++;
        return returnIdx >= rule.usage.size() ? rule.storage.back() : rule.storage[rule.usage[returnIdx]];
    }
};

UnitOrder ApplyAvoidRule(Unit& unit, const MoveRule& selected_rule);

inline std::tuple<Unit, double, const Projectile *>
Simulate(Unit unit, const Game &game, const ComplexMoveRule &moveRule, size_t deep = 20, const bool draw = false) {
    const auto &constants = Constants::INSTANCE;
    double damagePenalty = 0.;
    Vec2 last_position;
    std::vector<const Projectile *> remained_projectiles(game.projectiles.size());
    const Projectile* firstProjectile = nullptr;
    for (size_t i = 0; i != game.projectiles.size(); ++i) {
        remained_projectiles[i] = &game.projectiles[i];
    }
    ComplexMoveRuleViewer ruleViewer{moveRule};
    ZoneMover zoneMover{game.zone};
    constexpr double damageExp = 1.001;
    constexpr double kDeathPenalty = 1e7;
    for (size_t tick = 0; tick != deep; ++tick) {
        const auto& rule = ruleViewer.nextRule();
        const auto& moveDirection = rule.moveDirection;
        const auto& lookDirection = rule.lookDirection;
        const auto& keepAim = rule.keepAim;
        const auto& speedLimit = rule.speedLimit;
        if (tick != 0) {
            zoneMover.nextTick();
        }
        damagePenalty *= damageExp;
        if (zoneMover.IsTouchingZone(unit)) {
            unit.health -= constants.zoneDamagePerTick;
            damagePenalty += constants.zoneDamagePerTick;
            if (unit.health < 1e-3) {
                damagePenalty += kDeathPenalty;
                for (++tick; tick != deep; ++tick) {
                    damagePenalty *= damageExp;
                }
                return {unit, damagePenalty, firstProjectile};
            }
        }

        last_position = unit.position;
        ApplyAvoidRule(unit, rule);

        const double tickTime = constants.tickTime;
        const double passedTime = tickTime * tick;

        if (unit.remainingSpawnTime.has_value()) {
            if (passedTime + 1e-5 < *unit.remainingSpawnTime) {
                continue;
            }
            const bool hasCollision = [&unit, &game, &passedTime]() {
                const auto &obstacles = constants.Get(unit.position);
                for (auto &obstacle: obstacles) {
                    if ((obstacle->position - unit.position).sqrNorm() <=
                        sqr(obstacle->radius + constants.unitRadius)) {
                        return true;
                    }
                }
                for (auto& checkUnit : game.units) {
                    if (checkUnit.id == unit.id || checkUnit.remainingSpawnTime.value_or(0) > passedTime) {
                        continue;
                    }
                    if ((checkUnit.position - unit.position).sqrNorm() < sqr(Constants::INSTANCE.unitRadius * 2)) {
                        return true;
                    }
                }
                return false;
            }();
            if (hasCollision) {
                unit.health -= constants.spawnCollisionDamagePerTick;
                damagePenalty += constants.spawnCollisionDamagePerTick;
                if (unit.health < 1e-3) {
                    damagePenalty += kDeathPenalty;
                    for (++tick; tick != deep; ++tick) {
                        damagePenalty *= damageExp;
                    }
                    return {unit, damagePenalty, firstProjectile};
                }
            } else {
                unit.remainingSpawnTime.reset();
            }
            continue;
        }
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
        DRAWK('I',
              if (!draw) {
                  return;
              }
              debugInterface->addRing(unit.position, constants.unitRadius, 0.01,
                                      damage == 0 ? debugging::Color(1., .7, 0., .8) :
                                      debugging::Color(1., 0., 0., .8));
        );
        damagePenalty += std::min(damage, unit.health + unit.shield);
        ApplyDamage(unit, damage, game.currentTick);
        if (unit.health < 1e-3) {
            damagePenalty += kDeathPenalty;
            for (++tick; tick != deep; ++tick) {
                damagePenalty *= damageExp;
            }
            return {unit, damagePenalty, firstProjectile};
        }
    }

    return {unit, damagePenalty, firstProjectile};
}

inline std::tuple<double, size_t>
ChooseBest(const Unit &unit, const Game &game, const std::vector<ComplexMoveRule> &rules, std::vector<std::vector<std::pair<int, double>>>& dangerMatrix) {
    double minScore = std::numeric_limits<double>::infinity();
    size_t best_id = 0;
    constexpr size_t kSimulationDeep = 20;
    ZoneMover zoneMover{game.zone};
    for (size_t i = 0; i != kSimulationDeep; ++i) {
        zoneMover.nextTick();
    }

    double damageCouldCause = 0.;
    const auto& baseRule = rules.front().storage.front();
    if (baseRule.keepAim) {
        const double weaponDamage = Constants::INSTANCE.weapons[*unit.weapon].projectileDamage;
        int ticksToAim = (int) round((1. - unit.aim) / Constants::INSTANCE.weapons[*unit.weapon].aimPerTick);
        ticksToAim = std::max(unit.nextShotTick - game.currentTick, ticksToAim);
        int ticksBetweenShots =
                Constants::INSTANCE.ticksPerSecond / Constants::INSTANCE.weapons[*unit.weapon].roundsPerSecond;
        while (ticksToAim <= kSimulationDeep) {
            damageCouldCause += weaponDamage;
            ticksToAim += ticksBetweenShots;
        }
    }

    const auto checkShootAvailable = [&baseRule](const ComplexMoveRule &complexMoveRule) {
        for (const auto &rule: complexMoveRule.storage) {
            if (!rule.keepAim || !rule.lookDirection.has_value() ||
                (*baseRule.lookDirection - *rule.lookDirection).sqrNorm() > 1e-8) {
                return false;
            }
        }
        return true;
    };

    Vec2 resultPosition;

    constexpr double kIncomingDamageIgnorance = 0.12;
    constexpr double kDistanceFromZone = 3.;
    constexpr double kDangerCoeff = 1.;

    Vec2 bestResultPosition;

    for (size_t id = 0; id != rules.size(); ++id) {
        const auto &rule = rules[id];
        const auto [resultUnit, incomeDamageScore, _2] = Simulate(unit, game, rule, kSimulationDeep);
        if (id == 0) {
            resultPosition = resultUnit.position;
        }
        double resultScore = incomeDamageScore;
        if (checkShootAvailable(rule)) { // starting from this
            resultScore -= damageCouldCause * kIncomingDamageIgnorance;
        }
        double distance = zoneMover.DistanceFromZone(resultUnit) - kDistanceFromZone;
        if (distance < 0) {
            // 10. скорость / 30 тиков = 1/3 максимальная скорость. Поставим 1/5 - умножим дистанцию на 5
            resultScore += -distance * 5 * Constants::INSTANCE.zoneDamagePerTick;
        }
        resultScore += EvaluateDangerIncludeObstacles(unit.id, resultUnit.position, dangerMatrix, game) * kDangerCoeff;
        resultScore += (resultUnit.position - resultPosition).norm() * 0.05; // further from destination penalty

        if (resultScore < minScore) {
            minScore = resultScore;
            bestResultPosition = resultUnit.position;
            best_id = id;
        }
    }
    DRAW({ DrawCross(bestResultPosition, 0.5, debugging::Color(1., 0., 0., 0.6), debugInterface); });
    return {minScore, best_id};
}

inline UnitOrder ApplyAvoidRule(Unit& unit, const MoveRule& selected_rule) {
    if (selected_rule.lookDirection) {
        unit.direction = applyNewDirection(unit.direction, *selected_rule.lookDirection - unit.position,
                                           RotationSpeed(unit.aim, unit.weapon));
    }

    unit.aim = CalcResultAim(selected_rule.keepAim, unit.aim, unit.weapon);

    const auto velocity = MaxSpeedVector(unit, selected_rule.moveDirection, CalcAimSpeedModifier(unit))
            .LimitLength(selected_rule.speedLimit);
    UnitOrder order;
    order.targetDirection = unit.direction;
    order.targetVelocity = velocity;
    unit.velocity = ResultSpeedVector(unit.velocity, velocity);
    if (unit.remainingSpawnTime.has_value() || unit.velocity.sqrNorm() < 1e-20) {
        unit.position += unit.velocity * Constants::INSTANCE.tickTime;
    } else {
        updateForCollision(unit.position, unit.velocity);
    }
    return order;
}

#endif //AI_CUP_22_SIMULATION_HPP
