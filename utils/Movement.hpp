//
// Created by dragoon on 09.07.2022.
//

#ifndef AI_CUP_22_MOVEMENT_HPP
#define AI_CUP_22_MOVEMENT_HPP

#include "model/Vec2.hpp"
#include "model/Constants.hpp"
#include <iostream>

//enum MovemendSide {
//    kNorth, KNorthWest, kWest, kSouthWest, kSouth, kSouthEast, kEast, kNorthEast
//};

//template<MovemendSide side>
//Vec2 MaxSpeedVector(Vec2 position, Vec2 currentSpeed);

using namespace model;

template<typename T>
T sqr(T value) { return value * value; }

const std::vector<Vec2> kMoveDirections{{1.,  0.},
                                        {1.,  1.},
                                        {0.,  1.},
                                        {-1., 1.},
                                        {-1., 0.},
                                        {-1., -1.},
                                        {0.,  -1.},
                                        {1.,  -1.}};

Vec2 firstCollision(Vec2 position, Vec2 velocity, const double unit_radius, const Obstacle &obstacle,
                    const double remaining_time) {
    Vec2 end_point = position + velocity * remaining_time;

    const Vec2& coordsShift = obstacle.position;
    position -= coordsShift;
    end_point -= coordsShift;

    const double a = end_point.y - position.y;
    const double b = position.x - end_point.x;
    const double c = -a * end_point.x - b * end_point.y;

    const double aabb = a * a + b * b;

    const double x0 = -a * c / aabb;
    const double y0 = -b * c / aabb;
    const double d = sqr(obstacle.radius + unit_radius) - c * c / aabb;
    const double mult = sqrt(d / aabb);


    const auto pt1 = Vec2{x0 + b * mult, y0 - a * mult};
    const auto pt2 = Vec2{x0 - b * mult, y0 + a * mult};
    const double len = (end_point - position).norm();
    const double len1 = (pt1 - position).norm();
    const double len2 = (pt2 - position).norm();
//    std::cerr << "obs " << obstacle.id << " len1 " << len1 << " len2 " << len2 << " total " << len << std::endl;

    return Vec2{x0 + b * mult, y0 - a * mult} + coordsShift;
}

Vec2
MaxSpeedVector(Vec2 position, Vec2 direction, Vec2 target, const Constants &constants, const double aimModifier = 1) {
    const Vec2 coordsShift = position + direction * constants.unitMovementCircleShift;
    position -= coordsShift;
    target -= coordsShift;

    const double a = target.y - position.y;
    const double b = position.x - target.x;
    const double c = -a * target.x - b * target.y;

    const double aabb = a * a + b * b;

    const double x0 = -a * c / aabb;
    const double y0 = -b * c / aabb;
    const double d = sqr(constants.unitMovementCircleRadius) - c * c / aabb;
    const double mult = sqrt(d / aabb);
    return Vec2{x0 - b * mult, y0 + a * mult} * aimModifier - position;
}

inline Vec2 ResultSpeedVector(Vec2 currentVector, Vec2 targetVector, const Constants &constants) {
    Vec2 change_vector = targetVector - currentVector;
    double norm = change_vector.norm();
    return norm > constants.unitAccelerationPerTick ? currentVector +
                                                      change_vector * (constants.unitAccelerationPerTick / norm)
                                                    : targetVector;
}

inline double rotationSpeed(double aim, const std::optional<int>& weapon, const Constants &constants) {
    return aim > 0 ? constants.rotationSpeed -
                     (constants.rotationSpeed - constants.weapons[*weapon].aimRotationSpeed) * aim
                   : constants.rotationSpeed;
}

inline Vec2
applyNewDirection(Vec2 currentDirection, Vec2 targetDirection, const double rotation_speed) {
    double curr_angle = currentDirection.toRadians();
    double angle_diff = targetDirection.toRadians() - curr_angle;
    if (angle_diff > M_PI) {
        angle_diff -= M_PI * 2.;
    } else if (angle_diff < -M_PI) {
        angle_diff += M_PI * 2.;
    }
    const double diff_abs = abs(angle_diff);
    if (diff_abs > rotation_speed) {
        angle_diff *= rotation_speed / diff_abs;
    }
    curr_angle += angle_diff;

    return {std::cos(curr_angle), std::sin(curr_angle)};
}

Vec2 segmentClosestPoint(Vec2 point, Vec2 segmentStart, Vec2 segmentEnd) {
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

inline void updateForCollision(Unit& unit, const Constants& constants, DebugInterface* debugInterface) {
    const auto& obstacles = constants.Get(unit.position);
    double time_remained = 1. / constants.ticksPerSecond;

    const auto getClosestCollision = [&obstacles, &constants, &unit, &time_remained](
            const Obstacle *ignore) -> std::tuple<std::optional<const Obstacle *>, Vec2, double> {
        const Vec2 end_position = unit.position + unit.velocity * time_remained;

        std::optional<const Obstacle *> closest_obstacle = std::nullopt;
        Vec2 point;
        double closest_distance = std::numeric_limits<double>::infinity();
        for (const auto &obstacle: obstacles) {
            if (ignore != nullptr && obstacle->id == ignore->id) {
                continue;
            }
            const double radius = sqr(obstacle->radius + constants.unitRadius);
            auto closest_point = segmentClosestPoint(obstacle->position, unit.position, end_position);
            if ((closest_point - obstacle->position).sqrNorm() > radius) {
                continue;
            }
            const double curr_distance = (closest_point - unit.position).sqrNorm();
            if (curr_distance < closest_distance) {
                closest_distance = curr_distance;
                closest_obstacle = obstacle;
                point = closest_point;
            }
        }
        return {closest_obstacle, point, closest_distance};
    };

    const auto nextMove = [&](const Obstacle *prev_collision) -> const Obstacle * {
        const auto &[obstacle, point, distance_sqr] = getClosestCollision(prev_collision);
        if (!obstacle) {
            DRAW(debugInterface->addGradientSegment(unit.position, debugging::Color(1., 0., 0., 1.),
                                                    unit.position + unit.velocity * time_remained,
                                                    debugging::Color(0., 1., 0., 1.), 0.03););

//            std::cerr << "new_move " << (unit.velocity * time_remained).toString() << " norm "
//                      << (unit.velocity * time_remained).norm() << " time_remained " << time_remained
//                      << std::endl;
            unit.position += unit.velocity * time_remained;
            return nullptr;
        }
        const Obstacle &obstacleRef = **obstacle;
        const auto collision_point = firstCollision(unit.position, unit.velocity, constants.unitRadius, obstacleRef,
                                                    time_remained);
        DRAW(
                auto norm = (obstacleRef.position - collision_point).toLen(constants.unitRadius);
                auto new_pt = collision_point + norm;
                norm.rotate90().toLen(2.);
                debugInterface->addSegment(new_pt - norm, new_pt + norm, 0.03, debugging::Color(0., 0., 1., 1.));
        );
        time_remained -=
                time_remained * (collision_point - unit.position).norm() / (unit.velocity * time_remained).norm();
        const Vec2 norm = (obstacleRef.position - collision_point).toLen(1.);
        unit.velocity -= norm * unit.velocity.dot(norm);
        DRAW(debugInterface->addGradientSegment(unit.position, debugging::Color(1., 0., 0., 1.),
                                                collision_point,
                                                debugging::Color(0., 1., 0., 1.), 0.03););
//        std::cerr << "first_move " << (collision_point - unit.position).toString() << " norm "
//                  << (collision_point - unit.position).norm() << " time_remained " << time_remained << std::endl;
        unit.position = collision_point;
        return *obstacle;
    };

    if (auto obstacle = nextMove(nullptr); obstacle != nullptr) {
        nextMove(obstacle);
    }
}

#endif //AI_CUP_22_MOVEMENT_HPP
