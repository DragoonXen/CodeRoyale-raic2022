//
// Created by dragoon on 10.07.2022.
//

#ifndef AI_CUP_22_TICKSTARTUPDATE_HPP
#define AI_CUP_22_TICKSTARTUPDATE_HPP

#include <unordered_map>
#include "model/Projectile.hpp"
#include "model/Game.hpp"
#include "Util.hpp"
#include "DebugInterface.hpp"
#include "Visible.hpp"

namespace {
    using namespace model;

    int kLootExpiration = 30 * 30;
    int kUnitExpiration = 30 * 10;
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

struct ProjectileUnitsProposals {
    int tick;
    int unitId;
    int playerId;
    Vec2 position;
    Vec2 direction;
    int weaponType;
};

std::unordered_map<int, ProjectileUnitsProposals>
UpdateProjectiles(Game &game, std::optional<Game> &last_tick, const std::vector<Unit *> &units,
                  const std::unordered_map<int, VisibleFilter> &filters) {
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
    std::unordered_map<int, ProjectileUnitsProposals> knownUnitsInfo;
    for (auto &projectile: game.projectiles) {
        if (from_prev_tick.count(projectile.id)) {
            projectile.lifeTime = from_prev_tick[projectile.id]->lifeTime;
            from_prev_tick.erase(projectile.id);
        } else {
            const double timeInFly =
                    constants.weapons[projectile.weaponTypeIndex].projectileLifeTime - projectile.lifeTime;
            const int tickToShootFrom = game.currentTick - (int) (timeInFly / constants.tickTime + 0.1);
            auto iter = knownUnitsInfo.find(projectile.shooterId);
            if (iter == knownUnitsInfo.end() || iter->second.tick < tickToShootFrom) {
                const auto projectileDir = projectile.velocity.clone().toLen(1.);
                const auto shootPoint = projectile.position - projectile.velocity * timeInFly -
                                        projectileDir * constants.unitRadius;

                knownUnitsInfo[projectile.shooterId] =
                        ProjectileUnitsProposals{tickToShootFrom, projectile.shooterId, projectile.shooterPlayerId,
                                                 shootPoint, projectileDir, projectile.weaponTypeIndex};
            }

            UpdateLifetime(projectile);
        }
    }

    if (last_tick) {
        for (auto &projectile: last_tick->projectiles) {
            if (!from_prev_tick.count(projectile.id)) {
                continue;
            }
            projectile.position += projectile.velocity * constants.tickTime;
            if (!IsVisible<kVisibilityFilter>(projectile.position, units, filters)) {
                game.projectiles.push_back(projectile);
            } else {
                DRAW({
                         double angle = projectile.velocity.toRadians();
                         debugInterface->addPie(projectile.position, .5, angle - M_PI_4, angle + M_PI_4,
                                                debugging::Color(1., 0., 0., .3));
                     });
            }
        }
    }
    DRAW(
            for (auto &curr_shot: game.projectiles) {
                debugInterface->addPlacedText(curr_shot.position + model::Vec2{0.5, 0.5},
                                              "p " + std::to_string(curr_shot.id),
                                              {0., 1.}, 0.3,
                                              debugging::Color(0, 0, 0, 1));
                debugInterface->addGradientSegment(curr_shot.position, debugging::Color(1., 0., 0., .8),
                                                   curr_shot.position +
                                                   curr_shot.velocity * curr_shot.lifeTime,
                                                   debugging::Color(0., 1., 0., .8),
                                                   .1);
            }
    );
    return knownUnitsInfo;
}

void UpdateLoot(Game &game, std::optional<Game> &last_tick, const std::vector<Unit *> &units,
                const std::unordered_map<int, VisibleFilter> &filters) {
    std::unordered_map<int, Loot *> from_prev_tick;
     const Constants &constants = Constants::INSTANCE;
    if (last_tick) {
        for (auto &loot: last_tick->loot) {
            if (loot.lastSeenTick + kLootExpiration < game.currentTick) {
                continue;
            }
            from_prev_tick[loot.id] = &loot;
        }
    }

    for (auto &curr_loot: game.loot) {
        if (from_prev_tick.count(curr_loot.id)) {
            from_prev_tick.erase(curr_loot.id);
        }
        curr_loot.lastSeenTick = game.currentTick;
    }
    if (last_tick) {
        for (auto &loot: last_tick->loot) {
            if (!from_prev_tick.count(loot.id)) {
                continue;
            }
            if (!IsVisible<kVisibilityFilter>(loot.position, units, filters)) {
                DRAW(debugInterface->addCircle(loot.position, .7, debugging::Color(0., 1., 0., .3)););
                game.loot.push_back(loot);
            }
        }
    }

    DRAW(
            for (auto &curr_loot: game.loot) {
                debugInterface->addPlacedText(curr_loot.position + model::Vec2{0.5, 0.5},
                                              "l " + std::to_string(curr_loot.id),
                                              {0., 1.}, 0.3,
                                              debugging::Color(0, 0, 0, 1));
            }
    );
}

void UpdateUnitFromInfo(Unit& unit, ProjectileUnitsProposals& proposal) {
    if (unit.lastSeenTick > proposal.tick) {
        return;
    }
    unit.position = proposal.position;
    unit.direction = proposal.direction;
    unit.playerId = proposal.playerId;
    unit.id = proposal.unitId;
    unit.weapon = proposal.weaponType;
    if (unit.ammo.empty()) {
        unit.ammo = {10, 10, 10};
    } else {
        --unit.ammo[proposal.weaponType];
    }
    Constants& constants = Constants::INSTANCE;
    unit.nextShotTick =
            proposal.tick + (constants.ticksPerSecond / constants.weapons[proposal.weaponType].roundsPerSecond + 1e-3);
    unit.lastSeenTick = proposal.tick;
}

void UpdateUnits(Game &game, std::optional<Game> &lastTick, const std::vector<Unit *> &units,
                 const std::unordered_map<int, VisibleFilter> &filters,
                 std::unordered_map<int, ProjectileUnitsProposals> projectilesUnitInfo) {
    DRAW(for (auto &[id, info]: projectilesUnitInfo) {
        debugInterface->addCircle(info.position, Constants::INSTANCE.unitRadius,
                                  debugging::Color(1., 0., 0., .3));
        debugInterface->addSegment(info.position, info.position + info.direction, .1, debugging::Color(0., 0., 0., .7));
        debugInterface->addPlacedText(info.position + model::Vec2{0., 1},
                                      "su " + std::to_string(info.unitId) + "\n p " + std::to_string(info.playerId) +
                                      "\nt " + std::to_string(info.tick),
                                      {0., 1.}, 0.3,
                                      debugging::Color(0, 0, 0, 1));
    });
    std::unordered_map<int, Unit *> from_prev_tick;
    const Constants &constants = Constants::INSTANCE;
    if (lastTick) {
        for (auto &unit: lastTick->units) {
            if (unit.lastSeenTick + kUnitExpiration < game.currentTick) {
                continue;
            }
            from_prev_tick[unit.id] = &unit;
        }
    }

    for (auto &unit: game.units) {
        if (from_prev_tick.count(unit.id)) {
            from_prev_tick.erase(unit.id);
        }
        projectilesUnitInfo.erase(unit.id);
        unit.lastSeenTick = game.currentTick;
    }

    if (lastTick) {
        std::vector<Unit*> unitsToAdd;
        for (auto &unit: lastTick->units) {
            if (!from_prev_tick.count(unit.id)) {
                continue;
            }
            if (auto iter = projectilesUnitInfo.find(unit.id); iter != projectilesUnitInfo.end()) {
                UpdateUnitFromInfo(unit, iter->second);
                projectilesUnitInfo.erase(iter);
            }
            if (!IsVisible<kVisibilityFilter>(unit.position, units, filters)) {
                DRAW(debugInterface->addCircle(unit.position, 1.3, debugging::Color(0., 1., 0., .3)););
                unitsToAdd.push_back(&unit);
            }
        }
        for (auto unit: unitsToAdd) {
            game.units.push_back(*unit);
        }
    }
    for (auto &[id, info]: projectilesUnitInfo) {
        Unit unit;
        unit.id = id;
        unit.lastSeenTick = 0;
        UpdateUnitFromInfo(unit, info);
        game.units.push_back(unit);
    }
}

#endif //AI_CUP_22_TICKSTARTUPDATE_HPP
