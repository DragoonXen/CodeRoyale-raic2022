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

std::tuple<Unit, double, const Projectile *>
Simulate(Unit unit, const Game &game, const ComplexMoveRule &moveRule, size_t deep = 20) {
    const auto &constants = Constants::INSTANCE;
    double damagePenalty = 0.;
    Vec2 last_position;
    std::vector<const Projectile *> remained_projectiles(game.projectiles.size());
    const Projectile* firstProjectile = nullptr;
    for (size_t i = 0; i != game.projectiles.size(); ++i) {
        remained_projectiles[i] = &game.projectiles[i];
    }
    ComplexMoveRuleViewer ruleViewer{moveRule};
    for (size_t tick = 0; tick != deep; ++tick) {
        const auto& rule = ruleViewer.nextRule();
        const auto& moveDirection = rule.moveDirection;
        const auto& lookDirection = rule.lookDirection;
        const auto& keepAim = rule.keepAim;
        const auto& speedLimit = rule.speedLimit;

        last_position = unit.position;
        ApplyAvoidRule(unit, rule);

        const double tickTime = constants.tickTime;
        const double passedTime = tickTime * tick;
        double damage = 0.;
        DRAWK('I',
              debugInterface->addCircle(unit.position, constants.unitRadius,
                                        damage == 0 ? debugging::Color(1., .7, 0., .1) :
                                        debugging::Color(1., 0., 0., .1));
        );
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

std::tuple<double, size_t> ChooseBest(const Unit &unit, const Game &game, const std::vector<ComplexMoveRule> &rules) {
    double minScore = std::numeric_limits<double>::infinity();
    size_t best_id = 0;
    for (size_t id = 0; id != rules.size(); ++id) {
        const auto &rule = rules[id];
        const auto [_, score, _2] = Simulate(unit, game, rule);
        if (score < minScore) {
            if (score == 0.) {
                return {0., id};
            }
            minScore = score;
            best_id = id;
        }
    }
    return {minScore, best_id};
}

inline UnitOrder ApplyAvoidRule(Unit& unit, const MoveRule& selected_rule) {
    if (selected_rule.lookDirection) {
        unit.direction = applyNewDirection(unit.direction, *selected_rule.lookDirection - unit.position,
                                           RotationSpeed(unit.aim, unit.weapon));
    }

    unit.aim = CalcResultAim(selected_rule.keepAim, unit.aim, unit.weapon);

    const auto velocity =
            MaxSpeedVector(unit.position, unit.direction, selected_rule.moveDirection, CalcAimSpeedModifier(unit))
                    .LimitLength(selected_rule.speedLimit);
    UnitOrder order;
    order.targetDirection = unit.direction;
    order.targetVelocity = velocity;
    unit.velocity = ResultSpeedVector(unit.velocity, velocity);
    updateForCollision(unit.position, unit.velocity);
    return order;
}

#endif //AI_CUP_22_SIMULATION_HPP