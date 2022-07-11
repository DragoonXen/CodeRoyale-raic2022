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

void UpdateProjectiles(Game &game, std::optional<Game> &last_tick, const std::vector<Unit *> &units,
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

    for (auto &curr_shot: game.projectiles) {
        if (from_prev_tick.count(curr_shot.id)) {
            curr_shot.lifeTime = from_prev_tick[curr_shot.id]->lifeTime;
            from_prev_tick.erase(curr_shot.id);
        } else {
            UpdateLifetime(curr_shot);
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
}

void UpdateLoot(Game &game, std::optional<Game> &last_tick, const std::vector<Unit *> &units,
                const std::unordered_map<int, VisibleFilter> &filters) {
    std::unordered_map<int, Loot *> from_prev_tick;
    const Constants &constants = Constants::INSTANCE;
    if (last_tick) {
        for (auto &loot: last_tick->loot) {
            if (loot.lastSeenTick + kLootExpiration > game.currentTick) {
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

void UpdateUnits(Game &game, std::optional<Game> &last_tick, const std::vector<Unit *> &units,
                 const std::unordered_map<int, VisibleFilter> &filters) {
    std::unordered_map<int, Unit *> from_prev_tick;
    const Constants &constants = Constants::INSTANCE;
    if (last_tick) {
        for (auto &unit: last_tick->units) {
            if (unit.lastSeenTick + kUnitExpiration > game.currentTick) {
                continue;
            }
            from_prev_tick[unit.id] = &unit;
        }
    }

    for (auto &unit: game.units) {
        if (from_prev_tick.count(unit.id)) {
            from_prev_tick.erase(unit.id);
        }
        unit.lastSeenTick = game.currentTick;
    }
    if (last_tick) {
        std::vector<Unit*> unitsToAdd;
        for (auto &unit: last_tick->units) {
            if (!from_prev_tick.count(unit.id)) {
                continue;
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
}

#endif //AI_CUP_22_TICKSTARTUPDATE_HPP
