//
// Created by dragoon on 09.07.2022.
//

#ifndef AI_CUP_22_UTIL_HPP
#define AI_CUP_22_UTIL_HPP

#include <tuple>
#include <optional>
#include "model/Obstacle.hpp"
#include "model/Vec2.hpp"
#include "model/Constants.hpp"

namespace {
    using namespace model;
}

constexpr int kLastSeenArrayStep = 30;

inline double RotationSpeed(double aim, const std::optional<int>& weapon) {
    const Constants &constants = Constants::INSTANCE;
    return aim > 0 ? constants.rotationSpeed -
                     (constants.rotationSpeed - constants.weapons[*weapon].aimRotationSpeed) * aim
                   : constants.rotationSpeed;
}

inline double RotationSpeed(const Unit& unit) {
    return RotationSpeed(unit.aim, unit.weapon);
}

template<typename T>
T sqr(T value) { return value * value; }

inline Vec2 SegmentClosestPoint(Vec2 point, Vec2 segmentStart, Vec2 segmentEnd) {
    Vec2 v = segmentEnd - segmentStart;
    Vec2 w = point - segmentStart;

    double c1 = v.dot(w);
    if ( c1 <= 0 ) {
        return segmentStart;
    }
    double c2 =v.dot(v);
    if (c2 <= c1) {
        return segmentEnd;
    }
    double b = c1 / c2;
    return segmentStart + v * b;
}

inline double SegmentPointSqrDist(Vec2 point, Vec2 segmentStart, Vec2 segmentEnd) {
    return (SegmentClosestPoint(point, segmentStart, segmentEnd) - point).sqrNorm();
}

inline double SegmentPointDist(Vec2 point, Vec2 segmentStart, Vec2 segmentEnd) {
    return (SegmentClosestPoint(point, segmentStart, segmentEnd) - point).norm();
}

template<bool shoot_passable = false>
std::tuple<std::optional<const model::Obstacle *>, Vec2>
GetClosestCollision(Vec2 start, Vec2 finish, const double addRadius, std::optional<int> ignoreId = std::nullopt) {
    const auto& obstacles = Constants::INSTANCE.Get(start);

    std::optional<const Obstacle *> closest_obstacle = std::nullopt;
    Vec2 point;
    double closest_distance = std::numeric_limits<double>::infinity();
    for (const auto &obstacle: obstacles) {
        if (ignoreId && obstacle->id == *ignoreId || (shoot_passable && obstacle->canShootThrough)) {
            continue;
        }
        const double radiusSqr = sqr(obstacle->radius + addRadius);
        auto closest_point = SegmentClosestPoint(obstacle->position, start, finish);
        if ((closest_point - obstacle->position).sqrNorm() > radiusSqr) {
            continue;
        }
        const double curr_distance = (closest_point - start).sqrNorm();
        if (curr_distance < closest_distance) {
            closest_distance = curr_distance;
            closest_obstacle = obstacle;
            point = closest_point;
        }
    }
    return {closest_obstacle, point};
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

inline double CalcResultAim(bool keep, double start, const std::optional<int>& weapon) {
    DRAW(
        if (keep && !weapon) {
            std::cerr << "wrong state" << std::endl;
            exit(0);
        }
    );
    if (keep) {
        return std::min(1., start + Constants::INSTANCE.weapons[*weapon].aimPerTick);
    } else {
        return std::max(0., start - Constants::INSTANCE.weapons[*weapon].aimPerTick);
    }
}

inline double AngleDiff(double first, double second) {
    double angleDiff = first - second;
    if (angleDiff > M_PI) {
        angleDiff -= M_PI * 2.;
    } else if (angleDiff <= -M_PI) {
        angleDiff += M_PI * 2.;
    }
    return angleDiff;
}

inline double IncreaseAngle(double angle, double until) {
    while (angle < until) {
        angle += M_PI * 2.;
    }
    return angle;
}

inline double AddAngle(double angle, double angleToAdd) {
    angle += angleToAdd;
    if (angle > M_PI) {
        angle -= M_PI * 2.;
    }
    return angle;
}

inline double SubstractAngle(double angle, double angleToSubstract) {
    angle -= angleToSubstract;
    if (angle <= -M_PI) {
        angle += M_PI * 2.;
    }
    return angle;
}

// check if angle between from and to counter clockwize
inline bool AngleBetween(double angle, double from, double to) {
    if (to < from) {
        to += M_PI * 2.;
    }
    if (angle < from) {
        angle += M_PI * 2.;
    }
    return angle <= to;
}

inline bool
IsCollide(Vec2 position, Vec2 velocity, Vec2 position2, Vec2 velocity2, const double time,
          const double collisionRadius) {
    velocity2 -= velocity;
    Vec2 position2End = position2 + velocity2 * time;
    return (SegmentClosestPoint(position, position2, position2End) - position).sqrNorm() < sqr(collisionRadius);
}

inline void ApplyDamage(Unit& unit, double incomingDamage, int tick) {
    if (unit.shield > 1e-5) {
        const double shieldMitigate = std::min(unit.shield, incomingDamage);
        unit.shield -= shieldMitigate;
        incomingDamage -= shieldMitigate;
    }
    if (incomingDamage > 1e-8) {
        unit.healthRegenerationStartTick = tick + Constants::INSTANCE.healthRegenerationDelayTicks;
        unit.health = std::max(unit.health - incomingDamage, 0.);
    }
}

struct ZoneMover {
    Zone zone;

    ZoneMover(const Zone &zone) : zone(zone) {};

    void nextTick() {
        if (zone.currentRadius <= zone.nextRadius) {
            zone.currentRadius -= Constants::INSTANCE.zoneSpeedPerTick;
            return;
        }
        size_t tickToReach = round((zone.currentRadius - zone.nextRadius) / Constants::INSTANCE.zoneSpeedPerTick);
        zone.currentCenter += (zone.nextCenter - zone.currentCenter) * (1. / tickToReach);
        zone.currentRadius -= Constants::INSTANCE.zoneSpeedPerTick;
    }

    inline double DistanceFromZone(const Vec2 position, size_t count) const {
        size_t tickToReach = round((zone.currentRadius - zone.nextRadius) / Constants::INSTANCE.zoneSpeedPerTick);
        size_t centerShiftTicks = std::min(tickToReach, count);
        const Vec2 center = zone.currentCenter + (zone.nextCenter - zone.currentCenter) *
                                                 ((double) centerShiftTicks / (double) tickToReach);
        const double radius = zone.currentRadius - Constants::INSTANCE.zoneSpeedPerTick * count;
        return radius - (position - center).norm();
    }

    inline bool IsTouchingZone(const Unit &unit) {
        return (unit.position - zone.currentCenter).norm() + Constants::INSTANCE.unitRadius > zone.currentRadius;
    }

    inline double DistanceFromZone(const Unit &unit) {
        if (zone.currentRadius < Constants::INSTANCE.unitRadius) {
            return 0.;
        }
        return zone.currentRadius - ((unit.position - zone.currentCenter).norm() + Constants::INSTANCE.unitRadius);
    }
};

inline bool ShootWhileSpawning(const Unit &fromUnit, const Unit &otherUnit, const double addDistance) {
    return otherUnit.remainingSpawnTime.has_value() &&
           ((otherUnit.position - fromUnit.position).norm() + addDistance) /
           Constants::INSTANCE.weapons[*fromUnit.weapon].projectileSpeed <
           *otherUnit.remainingSpawnTime;
}

inline std::pair<Vec2, Vec2> TangentialPoints(Vec2 from, Vec2 to, const double radius) {
    from = (to + from) * .5;
    const double firstRadiusSqr = (from - to).sqrNorm() + 1e-10;
    to -= from;

    const double a = -2 * to.x;
    const double b = -2 * to.y;
    const double c = sqr(to.x) + sqr(to.y) + firstRadiusSqr - sqr(radius);

    const double x0 = -a * c / (a * a + b * b);
    const double y0 = -b * c / (a * a + b * b);
    // every time there would be two intersection points
    double d = firstRadiusSqr - sqr(c) / (sqr(a) + sqr(b));
    double mult = sqrt(d / (a * a + b * b));
    return {Vec2{x0 + b * mult, y0 - a * mult} + from,
            Vec2{x0 - b * mult, y0 + a * mult} + from};
}

inline void TickRespawnTime(Unit& unit) {
    if (unit.remainingSpawnTime.has_value()) {
        *unit.remainingSpawnTime -= Constants::INSTANCE.tickTime;
        if (*unit.remainingSpawnTime < 1e-5) {
            unit.remainingSpawnTime.reset();
        }
    }
}

inline double CalculateDanger(Vec2 position, const Unit& enemyUnit) {
    double danger = 1.;
    const Vec2 distanceVec = position - enemyUnit.position;
    // angle danger
    const double angleDiff = std::abs(AngleDiff(enemyUnit.direction.toRadians(), distanceVec.toRadians()));
    // normalize from 0 to M_PI * 5 / 6
    const auto diff = std::min(std::max(angleDiff - (M_PI / 6), 0.), M_PI * 2 / 3);
    danger *= 1. - diff / (M_PI * 5 / 6);
    // distance danger
    constexpr double maxDangerDistance = 7. * 7.;
    const double distanceSqr = distanceVec.sqrNorm();
    const double dangerDistanceCheck = maxDangerDistance / std::max(distanceSqr, maxDangerDistance);
    danger *= dangerDistanceCheck;
    // weapon danger
    if (enemyUnit.weapon && enemyUnit.ammo[*enemyUnit.weapon] > 0) {
        constexpr double kWeaponDanger[] = {0.34, .6666, 1.};
        danger *= kWeaponDanger[*enemyUnit.weapon];
        if (*enemyUnit.weapon == 1 && distanceSqr < sqr(11.)) {
            danger *= 1.55;
        }
    } else {
        danger *= 1e-2;
    }
    return danger;
}

inline double EvaluateDanger(Vec2 pos, std::vector<std::vector<std::pair<int, double>>>& dangerMatrix, const model::Game& game) {
    const auto& constants = Constants::INSTANCE;
    Vec2 targetPos(constants.toI(pos.x), constants.toI(pos.y));
    const int posX = constants.toI(pos.x) - constants.minX;
    const int posY = constants.toI(pos.y) - constants.minY;

    auto& value = dangerMatrix[posX][posY];
    if (value.first == game.currentTick) {
        return value.second;
    }
    value.first = game.currentTick;

    // angle, danger, playerId
    std::vector<std::tuple<double, double, int >> unitsDanger;
    double sumDanger = 0.;
    const std::array<int, 4> weaponDangerRadius = {12, 11, 18, 4};
    for (auto& unit : game.units) {
        if (unit.playerId == game.myId) {
            continue;
        }
        auto positionDiff = unit.position - targetPos;
        // TODO: think about obstacles. Worth to hide behind non shootable obstacle with reasonable precision
        unitsDanger.emplace_back(positionDiff.toRadians(), CalculateDanger(targetPos, unit),
                                 unit.playerId);
        // distance of 5
        if (positionDiff.sqrNorm() < sqr(weaponDangerRadius[unit.weapon.value_or(3)])) {
            sumDanger += std::get<1>(unitsDanger.back());
        }
    }
    constexpr double kMinAngleDifference = M_PI / 3.6;  // 50 degrees
    constexpr double kMinCoeff = 0.;
    constexpr double kMaxAngleDifference = M_PI * (2. / 3.);  // 120 degrees
    constexpr double kMaxCoeff = 1.;
    constexpr double kDiffPerScore = (kMaxCoeff - kMinCoeff) / (kMaxAngleDifference - kMinAngleDifference);
    for (size_t i = 0; i != unitsDanger.size(); ++i) {
        auto& [angle, danger, playerId] = unitsDanger[i];
        for (size_t j = i + 1; j != unitsDanger.size(); ++j) {
            auto& [angle2, danger2, playerId2] = unitsDanger[j];
            const double angleDiff = std::abs(AngleDiff(angle, angle2));
            if (angleDiff <= kMinAngleDifference) {
                sumDanger += kMinCoeff * std::min(danger, danger2);
            } else if (angleDiff >= kMaxAngleDifference) {
                sumDanger += kMaxCoeff * std::min(danger, danger2);
            } else {
                sumDanger += kDiffPerScore * (angleDiff - kMinAngleDifference) * std::min(danger, danger2);
            }
        }
    }
    value.second = sumDanger;
    return value.second;
}

inline double
EvaluateDangerIncludeObstacles(int unitId, Vec2 pos, std::vector<std::vector<std::pair<int, double>>> &dangerMatrix,
                               const Game &game) {
    constexpr double maxDistance = 1.5;
    constexpr double maxValue = 1.;
    const double baseDanger = EvaluateDanger(pos, dangerMatrix, game);
    auto danger = baseDanger;
    const auto &obstacles = Constants::INSTANCE.GetL(pos);

    for (auto obstacle: obstacles) {
        double distance = (obstacle->position - pos).norm() - obstacle->radius - Constants::INSTANCE.unitRadius;
        if (distance >= maxDistance) {
            continue;
        }
        danger += ((maxDistance - distance) / maxDistance) * maxValue;
    }
    for (auto& unit : game.units) {
        if (unit.id == unitId) {
            continue;
        }
        double distance = (unit.position - pos).norm() - Constants::INSTANCE.unitRadius * 2;
        if (distance >= maxDistance) {
            continue;
        }
        danger += ((maxDistance - distance) / maxDistance) * maxValue;
    }
//    DRAW({
//             debugInterface->addRect(pos - Vec2(0.5, 0.5), {1., 1.}, debugging::Color(0., 0., 0., 0.5));
//             debugInterface->addPlacedText(pos,
//                                           to_string_p(baseDanger, 4) + " " + to_string_p(danger, 4), {0.5, 0.5},
//                                           0.05, debugging::Color(1., 1., 1., 0.9));
//         });
    return danger;
}

inline Vec2 GetLastSeenCoord(int posX, int posY) {
    return Vec2(Constants::INSTANCE.minX + posX * kLastSeenArrayStep,
                Constants::INSTANCE.minY + posY * kLastSeenArrayStep);
}

inline int TicksToShotAvailable(const Unit& unit, Vec2 position, int currentTick) {
    // at first - just timer
    int result = std::max(unit.nextShotTick - currentTick, 0);
    const double rotationSpeed = RotationSpeed(unit);
    const double angleDiff = std::abs(AngleDiff(unit.direction.toRadians(), (position - unit.position).toRadians()));
    result = std::max(result, (int) round(angleDiff / rotationSpeed + 0.4));
    int ticksToAim = (int) round((1. - unit.aim) / Constants::INSTANCE.weapons[*unit.weapon].aimPerTick);
    if (unit.action.has_value()) {
        ticksToAim += unit.action->finishTick - currentTick;
    }
    return std::max(result, ticksToAim);
}

inline double DamageCouldCause(const Unit& unit) {
    return unit.ammo[*unit.weapon] * Constants::INSTANCE.weapons[*unit.weapon].projectileDamage;
}

inline int TicksToKillUnit(const Unit& unit, const Unit& target, int currentTick) {
    if (!unit.weapon.has_value() || DamageCouldCause(unit) < target.shield + target.health) {
        return 999999;
    }
    int ticksKill = TicksToShotAvailable(unit, target.position, currentTick);
    double targetHealth = target.shield + target.health;
    while (targetHealth > 0.) {
        targetHealth -= Constants::INSTANCE.weapons[*unit.weapon].projectileDamage;
        ticksKill += Constants::INSTANCE.weapons[*unit.weapon].ticksBetweenRounds;
    }
    return ticksKill;
}

inline double MaxSoundRadius(const model::Sound& sound, Vec2 unitPos) {
    Constants &constants = Constants::INSTANCE;
    const double &sDistance = constants.sounds[sound.typeIndex].distance;
    const double &sOffset = constants.sounds[sound.typeIndex].offset;
    const double maxDistance = std::min(
            (sound.position - unitPos).norm() / (1. - sOffset), sDistance);
    return maxDistance * sOffset;
}

template<typename K, typename V>
V GetWithDef(const std::unordered_map<K, V> &m, const K &key, const V &defVal) {
    typename std::unordered_map<K, V>::const_iterator it = m.find(key);
    if (it == m.end()) {
        return defVal;
    } else {
        return it->second;
    }
}

struct Lane {
    double a,b,c;
    Lane(const Vec2& p, const Vec2& q) {
        a = p.y - q.y;
        b = q.x - p.x;
        c = -a * p.x - b * p.y;
    }
};

const double EPS = 1e-9;

inline double Det(double a, double b, double c, double d) {
    return a * d - b * c;
}

inline bool Intersect(Lane m, Lane n, Vec2 &res) {
    double zn = Det(m.a, m.b, n.a, n.b);
    if (abs(zn) < EPS)
        return false;
    res.x = -Det(m.c, m.b, n.c, n.b) / zn;
    res.y = -Det(m.a, m.c, n.a, n.c) / zn;
    return true;
}

inline bool Parallel(Lane m, Lane n) {
    return abs(Det(m.a, m.b, n.a, n.b)) < EPS;
}

inline bool Equivalent(Lane m, Lane n) {
    return abs(Det(m.a, m.b, n.a, n.b)) < EPS
           && abs(Det(m.a, m.c, n.a, n.c)) < EPS
           && abs(Det(m.b, m.c, n.b, n.c)) < EPS;
}

#ifdef DEBUG_INFO
#define VERIFY(a, b) {if (!(a)){std::cerr << (b) << std::endl; getchar();exit(-1);}}
#else
#define VERIFY(a, b) (1==1)
#endif

#endif //AI_CUP_22_UTIL_HPP
