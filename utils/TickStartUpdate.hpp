//
// Created by dragoon on 10.07.2022.
//

#ifndef AI_CUP_22_TICKSTARTUPDATE_HPP
#define AI_CUP_22_TICKSTARTUPDATE_HPP

#include <unordered_map>
#include <list>
#include "model/Projectile.hpp"
#include "model/Game.hpp"
#include "Util.hpp"
#include "DebugInterface.hpp"
#include "Visible.hpp"

namespace {
    using namespace model;

    int kLootExpiration = 30 * 30;
    int kUnitExpiration = 30 * 10;
    int kSoundUnitExpiration = 30 * 2;
}

inline int GetNextUnitId() {
    static int currId = 50000;
    return ++currId;
}

inline Unit CreateEmptyUnit(int id) {
    Unit unit;
    unit.id = id;
    unit.lastSeenTick = 0;
    unit.health = 100.;
    unit.shield = 100.;
    unit.ammo = {40, 100, 10};
    return unit;
}

inline void UpdateLifetime(Projectile &projectile) {
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

inline std::unordered_map<int, ProjectileUnitsProposals>
UpdateProjectiles(Game &game, std::optional<Game> &last_tick, const std::vector<Unit *> &units,
                  const std::vector<Unit> &allUnits,
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
            if (projectile.shooterPlayerId != game.myId) {
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
            if (!IsVisible<kVisibilityFilter>(projectile.position, units, allUnits, filters)) {
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

struct ResultSound {
    Vec2 position;
    double radius;
    int soundType;
};

inline void UpdatePositionFromSound(Unit &unit, const ResultSound &sound, int currentTick,
                                    std::unordered_map<int, double> &lastUpdatePrecision) {
    auto &obstacles = Constants::INSTANCE.Get(sound.position);
    unit.lastSeenTick = currentTick;
    for (auto &obstacle: obstacles) {
        if ((obstacle->position - sound.position).sqrNorm() < sqr(Constants::INSTANCE.unitRadius + obstacle->radius)) {
            Vec2 direction = sound.position - obstacle->position;
            if (direction.sqrNorm() < 1e-5) {
                direction = {1., 0};
            }
            lastUpdatePrecision[unit.id] = sound.radius + obstacle->radius;
            unit.position = obstacle->position + direction.toLen(obstacle->radius + Constants::INSTANCE.unitRadius);
            return;
        }
    }
    lastUpdatePrecision[unit.id] = sound.radius;
    unit.position = sound.position;
}

inline std::unordered_map<int, std::vector<ResultSound>>
MergeSounds(const Game &game, const std::unordered_map<int, Unit> &unitById) {
    std::unordered_map<int, std::vector<ResultSound>> resultSound;
    std::unordered_map<int, std::unordered_map<int, std::vector<const model::Sound *>>> soundsMap;
    for (const auto &sound: game.sounds) {
        auto &insideMap = soundsMap.emplace(sound.typeIndex, 0).first->second;
        insideMap.emplace(sound.unitId, 0).first->second.push_back(&sound);
    }
    for (auto &[soundType, map]: soundsMap) {
        while (!map.empty()) {
            const model::Sound startingSound = *map.begin()->second.back();
            map.begin()->second.pop_back();
            ResultSound newSound = {
                    startingSound.position,
                    MaxSoundRadius(startingSound, unitById.at(startingSound.unitId).position),
                    soundType
            };
            auto iterator = map.begin();
            while (++iterator != map.end()) {
                double minDistance = sqr(newSound.radius * 2);
                auto &vector = iterator->second;
                size_t soundIndex = vector.size();
                for (auto &sound: vector) {
                    const double dist = (sound->position - newSound.position).sqrNorm();
                    if (dist < minDistance) {
                        minDistance = dist;
                        soundIndex = std::distance(vector.data(), &sound);
                    }
                }
                if (soundIndex != vector.size()) {
                    Vec2 newPosition = (vector[soundIndex]->position + newSound.position) * 0.5;

                    double addingMaxRadius = MaxSoundRadius(*vector[soundIndex],
                                                            unitById.at(vector[soundIndex]->unitId).position);
                    double distance = (newPosition - newSound.position).norm();
                    newSound.radius = std::max(newSound.radius - distance, addingMaxRadius - distance);
                    newSound.position = newPosition;
                    vector[soundIndex] = vector.back();
                    vector.pop_back();
                }
            }
            resultSound.emplace(soundType, 0).first->second.push_back(newSound);
            while (!map.empty() && map.begin()->second.empty()) {
                map.erase(map.begin()->first);
            }
        }
    }
    return resultSound;
}

inline void ApplySoundGroup(std::optional<int> weapon, std::vector<ResultSound> &soundsVec, double maxSqrDistanceToApply,
                            std::unordered_map<int, double> &lastUpdatePrecision, Game &game) {
    if (weapon.has_value()) {
        for (auto &sound: soundsVec) {
            sound.radius += Constants::INSTANCE.unitRadius;
        }
    }
    const auto weaponMatch = [&weapon](const Unit& unit) {
        if (!weapon.has_value()) {
            return true;
        }
        return unit.weapon.value_or(-1) == *weapon;
    };
    std::unordered_set<int> matchedUnits;
    for (size_t i = 0; i != soundsVec.size(); ++i) {
        auto &stepSound = soundsVec[i];
        if ([&game, &stepSound, &matchedUnits, &maxSqrDistanceToApply, &weaponMatch]() {
            for (auto &unit: game.units) {
                if (game.myId == unit.playerId || game.currentTick != unit.lastSeenTick || !weaponMatch(unit) ||
                    (unit.position - stepSound.position).sqrNorm() > maxSqrDistanceToApply) {
                    continue;
                }
                const double sqrDist = (unit.position - stepSound.position).sqrNorm();
                if (sqrDist + 1e-5 < sqr(stepSound.radius)) {
                    matchedUnits.insert(unit.id);
                    return true;
                }
            }
            return false;
        }()) {
            soundsVec[i--] = soundsVec.back();
            soundsVec.pop_back();
            continue;
        }
    }

    const auto singleMatch = [&soundsVec, &game, &matchedUnits, &maxSqrDistanceToApply, &lastUpdatePrecision, &weaponMatch]() -> bool {
        bool singleMatches = false;
        for (size_t i = 0; i != soundsVec.size(); ++i) {
            auto &stepSound = soundsVec[i];
            int foundUnitId = -1;
            for (auto &unit: game.units) {
                if (game.myId == unit.playerId || !weaponMatch(unit) || matchedUnits.count(unit.id) ||
                    (unit.position - stepSound.position).sqrNorm() > maxSqrDistanceToApply) {
                    continue;
                }
                const int ticksNotSeen = game.currentTick - unit.lastSeenTick;
                const double maxSqrDistance =
                        sqr(ticksNotSeen * Constants::INSTANCE.maxUnitForwardSpeed *
                            Constants::INSTANCE.tickTime + stepSound.radius +
                            GetWithDef(lastUpdatePrecision, unit.id, 0.)) + 1e-5;
                const double sqrDist = (unit.position - stepSound.position).sqrNorm();

                if (sqrDist < maxSqrDistance) {
                    if (foundUnitId == -1) {
                        foundUnitId = (int) std::distance(game.units.data(), &unit);
                    } else {
                        foundUnitId = -1;
                        break;
                    }
                }
            }
            if (foundUnitId != -1) {
                auto &unit = game.units[foundUnitId];
                matchedUnits.insert(unit.id);
                DRAW({
                         debugInterface->addCircle(unit.position, Constants::INSTANCE.unitRadius,
                                                   debugging::Color(0.0, 0.5, 0.5, 0.8));
                     });
                UpdatePositionFromSound(unit, stepSound, game.currentTick, lastUpdatePrecision);
                soundsVec[i--] = soundsVec.back();
                soundsVec.pop_back();
                singleMatches = true;
            }
        }
        return singleMatches;
    };

    const auto minMatch = [&soundsVec, &game, &matchedUnits, &maxSqrDistanceToApply, &lastUpdatePrecision, &weaponMatch]() {
        for (size_t i = 0; i != soundsVec.size(); ++i) {
            auto &stepSound = soundsVec[i];
            double minTimeToReach = std::numeric_limits<double>::infinity();
            size_t unitId;
            for (auto &unit: game.units) {
                if (game.myId == unit.playerId || !weaponMatch(unit) || matchedUnits.count(unit.id) ||
                    (unit.position - stepSound.position).sqrNorm() > maxSqrDistanceToApply) {
                    continue;
                }
                const int ticksNotSeen = game.currentTick - unit.lastSeenTick;
                const double maxDistance =
                        ticksNotSeen * Constants::INSTANCE.maxUnitForwardSpeed *
                        Constants::INSTANCE.tickTime + GetWithDef(lastUpdatePrecision, unit.id, 0.);
                const double sqrDist = (unit.position - stepSound.position).sqrNorm();
                if (sqrDist > sqr(maxDistance) + 1e-5) {
                    continue;
                }
                const double ttr = std::sqrt(sqrDist) / Constants::INSTANCE.maxUnitForwardSpeed;
                if (ttr < minTimeToReach) {
                    minTimeToReach = ttr;
                    unitId = (int) std::distance(game.units.data(), &unit);
                }
            }
            if (minTimeToReach != std::numeric_limits<double>::infinity()) {
                auto &unit = game.units[unitId];
                matchedUnits.insert(unit.id);
                DRAW({
                         debugInterface->addCircle(unit.position, Constants::INSTANCE.unitRadius,
                                                   debugging::Color(0.5, 0.5, 0., 0.8));
                     });
                UpdatePositionFromSound(unit, stepSound, game.currentTick, lastUpdatePrecision);
                soundsVec[i--] = soundsVec.back();
                soundsVec.pop_back();
            }
        }
    };

    while (!soundsVec.empty()) {
        size_t size = soundsVec.size();
        while (singleMatch());
        minMatch();
        if (size == soundsVec.size()) {
            break;
        }
    }
    for (auto &sound: soundsVec) {
        double minDistance = std::numeric_limits<double>::infinity();
        Vec2 closestMyUnit;
        for (const auto &unit: game.units) {
            if (unit.playerId != game.myId) {
                continue;
            }
            const double currDist = (unit.position - sound.position).sqrNorm();
            if (currDist < minDistance) {
                minDistance = currDist;
                closestMyUnit = unit.position;
            }
        }
        auto unit = CreateEmptyUnit(GetNextUnitId());
        unit.lastSeenTick = game.currentTick;
        unit.direction = (closestMyUnit - sound.position).toLen(1.);
        unit.position = sound.position;
        unit.playerId = -1;
        unit.aim = 0.;
        unit.weapon = weapon.value_or(1);
        game.units.push_back(unit);
        DRAW({
                 debugInterface->addCircle(unit.position, Constants::INSTANCE.unitRadius,
                                           debugging::Color(0.0, 0.0, 0.8, 0.8));
             });
    }
}

inline void UpdateUnknownDangers(std::unordered_map<int, ProjectileUnitsProposals> &projectileUnitProposals,
                                 Game &game, std::unordered_map<int, double> &lastUpdatePrecision) {
    std::unordered_map<int, Unit> unitById;
    for (const auto &unit: game.units) {
        unitById[unit.id] = unit;
    }

    std::unordered_map<int, std::vector<ResultSound>> resultSound = MergeSounds(game, unitById);
    if (resultSound.find(0) != resultSound.end()) {
        ApplySoundGroup(std::nullopt, resultSound[0], sqr(5), lastUpdatePrecision, game);
    }
    constexpr std::array<double, 3> distanceToApply = {15, 20, 10};
    for (int i = 1; i != 4; ++i) {
        if (resultSound.find(i) != resultSound.end()) {
            ApplySoundGroup(i - 1, resultSound[i], sqr(distanceToApply[i - 1]), lastUpdatePrecision, game);
        }
    }
    DRAWK('S', {
        const auto color = debugging::Color(.8, 0., .8, .2);
        for (const auto &[soundType, vector]: resultSound) {
            for (const auto &sound: vector) {
                DrawCross(sound.position, 0.2, color, debugInterface);
                debugInterface->addCircle(sound.position, sound.radius, color);
            }
        }
    });
}

inline void UpdateLoot(Game &game, std::optional<Game> &last_tick, const std::vector<Unit *> &units,
                       const std::vector<Unit> &allUnits, const std::unordered_map<int, VisibleFilter> &filters) {
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
            if (!IsVisible<kVisibilityFilter>(loot.position, units, allUnits, filters)) {
                DRAWK('L', debugInterface->addCircle(loot.position, .7, debugging::Color(0., 1., 0., .3)););
                game.loot.push_back(loot);
            }
        }
    }

    DRAWK('L',
          for (auto &curr_loot: game.loot) {
              debugInterface->addPlacedText(curr_loot.position + model::Vec2{0.5, 0.5},
                                            "l " + std::to_string(curr_loot.id),
                                            {0., 1.}, 0.3,
                                            debugging::Color(0, 0, 0, 1));
          }
    );
}

inline void UpdateUnitFromInfo(Unit &unit, ProjectileUnitsProposals &proposal) {
    if (unit.lastSeenTick > proposal.tick) {
        return;
    }
    unit.position = proposal.position;
    unit.direction = proposal.direction;
    unit.playerId = proposal.playerId;
    unit.id = proposal.unitId;
    unit.aim = 1.;
    unit.weapon = proposal.weaponType;
    --unit.ammo[proposal.weaponType];
    unit.remainingSpawnTime.reset();
    Constants &constants = Constants::INSTANCE;
    unit.nextShotTick =
            proposal.tick + (constants.ticksPerSecond / constants.weapons[proposal.weaponType].roundsPerSecond + 1e-3);
    unit.lastSeenTick = proposal.tick;
}

struct MovementStat {
    MovementStat(int val) : lastUpdateTick(0), lastSuccessfulPrediction(0), mem(), totalTries(0),
                            successPrediction(0) {}

    int lastUpdateTick;
    int lastSuccessfulPrediction;
    std::list<Unit> mem;
    int totalTries;
    int successPrediction;

    static inline std::array<int, 4> moveVariant;
    static inline std::array<int, 4> successVariant;

    inline void UpdateValue(Unit newVal, int currentTick) {
        constexpr size_t kMaxResultCount = 3;
        if (lastUpdateTick + 1 < currentTick) {
            mem.clear();
        }
        lastUpdateTick = currentTick;
        mem.push_back(newVal);
        if (mem.size() > kMaxResultCount) {
            mem.pop_front();
        }

        if (mem.size() < 3) {
            return;
        }
        MoveRule supposedRule;
        const Unit &base = *(++(++mem.rbegin()));
        const Unit &prev = *(++mem.rbegin());
        supposedRule.keepAim = prev.aim > 1. - 1e-6 || prev.aim > base.aim;
        const double rotationSpeed = RotationSpeed(base.aim, base.weapon);
        const double aDiff = AngleDiff(prev.direction.toRadians(), base.direction.toRadians());
        if (std::abs(aDiff - rotationSpeed) < 1e-5) { // unknown look point but we know rotate direction
            double resultAngle = M_PI_2;
            if (resultAngle * aDiff < 0) {
                resultAngle *= -1.;
            }
            const double targetAngle = AddAngle(prev.direction.toRadians(), resultAngle);
            supposedRule.lookDirection = prev.position + Vec2(targetAngle) * 50.;
        } else {
            auto first = Lane(base.position, base.position + base.direction);
            auto second = Lane(prev.position, prev.position + prev.direction);
            if (!Parallel(first, second)) {
                Vec2 intersectPoint;
                if (Intersect(first, second, intersectPoint)) {
                    Vec2 newVec = intersectPoint - prev.position;
                    if (std::abs(AngleDiff(newVec.toRadians(), prev.direction.toRadians())) < M_PI_2) {
                        DRAW(debugInterface->addSegment(prev.position, intersectPoint, 0.1,
                                                        debugging::Color(1., 0., 0., 0.3)););
                        supposedRule.lookDirection = intersectPoint;
                    } else {
                        DRAW(debugInterface->addSegment(prev.position, intersectPoint, 0.1,
                                                        debugging::Color(0., 1., 0., 0.3)););
                    }
                }
            }
        }

        Vec2 velocityDiff = prev.velocity - base.velocity;
        supposedRule.speedLimit = std::numeric_limits<double>::infinity();
        int currVariant;
        const double velocityDiffNorm = velocityDiff.norm();
        if (velocityDiffNorm > Constants::INSTANCE.unitAccelerationPerTick + 1e-8) { // collision detected. just suppose we will move to
            supposedRule.moveDirection = base.position + base.velocity * 50.;
            currVariant = 0;
        } else if (std::abs(velocityDiffNorm - Constants::INSTANCE.unitAccelerationPerTick) < 1e-5) {
            // maximum change. need to retrieve direction
            if (base.aim < 1e-5 && !supposedRule.keepAim) {
                auto newVector = MaxSpeedVector(prev.position, prev.direction, prev.position + prev.velocity,
                                                prev.position + prev.velocity + velocityDiff);
                supposedRule.moveDirection = prev.position + newVector.toLen(50.);
                currVariant = 1;
            } else {
                supposedRule.moveDirection = prev.position + Vec2(0, 1e-5);
                currVariant = 2;
            }
        } else {
            supposedRule.moveDirection = prev.position + prev.velocity.clone().toLen(50.);
            currVariant = 3;
        }
        ++totalTries;
        Unit forPrediction = prev;
        ApplyAvoidRule(forPrediction, supposedRule);
        const Unit &toCompare = mem.back();
        ++moveVariant[currVariant];
        constexpr double kMaxAcceptableDiff = 1e-3;
        const double kMaxAcceptablePosDiff = Constants::INSTANCE.unitAccelerationPerTick * Constants::INSTANCE.tickTime * kMaxAcceptableDiff;

        const double sqrDiff = (forPrediction.position - toCompare.position).sqrNorm();
        if (sqrDiff < sqr(kMaxAcceptablePosDiff)) {
            ++successPrediction;
            ++successVariant[currVariant];
            lastSuccessfulPrediction = currentTick;
            DRAW({
                     debugInterface->addCircle(forPrediction.position, 0.3, debugging::Color(0., 1., 0., 0.9));
                     for (size_t i = 0; i != 10; ++i) {
                         ApplyAvoidRule(forPrediction, supposedRule);
                         debugInterface->addRing(forPrediction.position, Constants::INSTANCE.unitRadius, 0.05,
                                                 debugging::Color(1., 1., 0., 0.9));
                     }
                 });
        } else {
            DRAW(debugInterface->addCircle(forPrediction.position, 0.3, debugging::Color(1., 0., 0., 0.9)););
        }
    }
};

inline void
UpdateVisited(std::vector<std::vector<int>> &lastSeenArray, const std::vector<Unit *> &units, int currentTick) {
    for (size_t i = 0; i != lastSeenArray.size(); ++i) {
        for (size_t j = 0; j != lastSeenArray[0].size(); ++j) {
            auto pos = GetLastSeenCoord(i, j);
            for (auto unit: units) {
                Vec2 dirVector = pos - unit->position;
                if (dirVector.sqrNorm() > sqr(Constants::INSTANCE.viewDistance)) {
                    continue;
                }
//                if (std::abs(AngleDiff(unit->direction.toRadians(), dirVector.toRadians())) <=
//                    unit->currentFieldOfView * 0.5) {
                lastSeenArray[i][j] = currentTick;
                break;
//                }
            }
        }
    }
}

inline void UpdateUnits(Game &game, std::optional<Game> &lastTick, const std::vector<Unit *> &units,
                        const std::unordered_map<int, VisibleFilter> &filters,
                        std::unordered_map<int, ProjectileUnitsProposals> projectilesUnitInfo,
                        std::unordered_map<int, MovementStat> &unitMovementMem) {
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
            if (unit.lastSeenTick + kUnitExpiration < game.currentTick || unit.playerId == game.myId ||
                (unit.id >= 50000 && unit.lastSeenTick + kSoundUnitExpiration < game.currentTick)) {
                continue;
            }
            from_prev_tick[unit.id] = &unit;
        }
    }

    for (auto &unit: game.units) {
        if (!unit.remainingSpawnTime.has_value() && unit.playerId != game.myId) {
            auto [iter, _] = unitMovementMem.emplace(unit.id, 0);
            iter->second.UpdateValue(unit, game.currentTick);
        }

        if (from_prev_tick.count(unit.id)) {
            from_prev_tick.erase(unit.id);
        }
        projectilesUnitInfo.erase(unit.id);
        unit.lastSeenTick = game.currentTick;
    }
    DRAW({
             std::stringstream print;
             print << game.currentTick;
             for (size_t i = 0; i != 4; ++i) {
                 print << " " << MovementStat::successVariant[i] << "/" << MovementStat::moveVariant[i];
                 if (MovementStat::moveVariant[i] != 0) {
                     print << "("
                           << to_string_p(MovementStat::successVariant[i] / (double) MovementStat::moveVariant[i], 3)
                           << ")";
                 }
             }
             std::cerr << print.str() << std::endl;
         });

    if (lastTick) {
        std::vector<Unit *> unitsToAdd;
        for (auto &unit: lastTick->units) {
            if (!from_prev_tick.count(unit.id)) {
                continue;
            }
            if (auto iter = projectilesUnitInfo.find(unit.id); iter != projectilesUnitInfo.end()) {
                UpdateUnitFromInfo(unit, iter->second);
                projectilesUnitInfo.erase(iter);
            }
            if (!IsVisible<kVisibilityFilter>(unit.position, units, game.units, filters)) {
                DRAW(debugInterface->addCircle(unit.position, 1.3, debugging::Color(0., 1., 0., .3)););
                unitsToAdd.push_back(&unit);
            }
        }
        for (auto unit: unitsToAdd) {
            TickRespawnTime(*unit);
            game.units.push_back(*unit);
        }
    }
    for (auto &[id, info]: projectilesUnitInfo) {
        Unit unit = CreateEmptyUnit(id);
        UpdateUnitFromInfo(unit, info);
        game.units.push_back(unit);
    }
}

#endif //AI_CUP_22_TICKSTARTUPDATE_HPP
