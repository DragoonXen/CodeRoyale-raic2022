//
// Created by dragoon on 10.07.2022.
//

#include <tuple>
#include "model/Unit.hpp"
#include "model/Projectile.hpp."
#include "model/Game.hpp"
#include "model/Constants.hpp"
#include "Movement.hpp"
#include "DebugInterface.hpp"

#ifndef AI_CUP_22_SIMULATION_HPP
#define AI_CUP_22_SIMULATION_HPP

namespace {
    using namespace model;
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

#endif //AI_CUP_22_SIMULATION_HPP
