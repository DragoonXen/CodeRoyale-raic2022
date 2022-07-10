//
// Created by dragoon on 10.07.2022.
//

#ifndef AI_CUP_22_TICKSTARTUPDATE_H
#define AI_CUP_22_TICKSTARTUPDATE_H

#include <unordered_map>
#include "model/Projectile.hpp"
#include "model/Game.hpp"
#include "Util.hpp"
#include "DebugInterface.hpp"

namespace {
    using namespace model;
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

void UpdateProjectiles(Game &game, std::optional<Game> &last_tick) {
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

void UpdateLoot(Game &game, std::optional<Game> &last_tick) {
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

#endif //AI_CUP_22_TICKSTARTUPDATE_H
