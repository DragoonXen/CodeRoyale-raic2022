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
        const double radius = sqr(obstacle->radius + addRadius);
        auto closest_point = SegmentClosestPoint(obstacle->position, start, finish);
        if ((closest_point - obstacle->position).sqrNorm() > radius) {
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
    } else if (angleDiff < -M_PI) {
        angleDiff += M_PI * 2.;
    }
    return angleDiff;
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

#ifdef DEBUG_INFO
#define VERIFY(a, b) {if (!(a)){std::cerr << (b) << std::endl; getchar();exit(-1);}}
#else
#define VERIFY(a, b) ()
#endif

#endif //AI_CUP_22_UTIL_HPP
