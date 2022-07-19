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
    if (incomingDamage - 1e-6 > unit.shield) {
        incomingDamage -= unit.shield;
        unit.shield = 0;
        unit.healthRegenerationStartTick = tick + Constants::INSTANCE.healthRegenerationDelayTicks;
    }
    unit.health -= incomingDamage;
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

    inline bool IsTouchingZone(const Unit &unit) {
        return (unit.position - zone.currentCenter).norm() + Constants::INSTANCE.unitRadius > zone.currentRadius;
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

#ifdef DEBUG_INFO
#define VERIFY(a, b) {if (!(a)){std::cerr << (b) << std::endl; getchar();exit(-1);}}
#else
#define VERIFY(a, b) (1==1)
#endif

#endif //AI_CUP_22_UTIL_HPP
