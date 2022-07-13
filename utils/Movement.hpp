//
// Created by dragoon on 09.07.2022.
//

#ifndef AI_CUP_22_MOVEMENT_HPP
#define AI_CUP_22_MOVEMENT_HPP

#include "model/Vec2.hpp"
#include "model/Constants.hpp"
#include "Util.hpp"
#include <iostream>

using namespace model;

const std::vector<Vec2> kMoveDirections = []() {
    constexpr double stepCount = 12.;
    std::vector<Vec2> result(stepCount);

    const double angleStep = M_PI * 2 / stepCount;
    for (size_t i = 0; i != stepCount; ++i) {
        const double currAngle = i * angleStep;
        result[i] = Vec2(currAngle);
    }
    return result;
}();

inline Vec2 FirstCollision(Vec2 position, Vec2 velocity, const double unit_radius, const Obstacle &obstacle,
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
    const double div = d / aabb;
    if (div >= 0.) {
        const double mult = sqrt(div);
        return Vec2{x0 + b * mult, y0 - a * mult} + coordsShift;
    } else {
        return Vec2{x0, y0} + coordsShift;
    }

//    const auto pt1 = Vec2{x0 + b * mult, y0 - a * mult};
//    const auto pt2 = Vec2{x0 - b * mult, y0 + a * mult};
//    const double len = (end_point - position).norm();
//    const double len1 = (pt1 - position).norm();
//    const double len2 = (pt2 - position).norm();
//    std::cerr << "obs " << obstacle.id << " len1 " << len1 << " len2 " << len2 << " total " << len << std::endl;
}


inline double CalcAimSpeedModifier(const Unit& unit) {
    return unit.aim == 0 ? 1. : (1. -
                                 (1. - Constants::INSTANCE.weapons[*unit.weapon].aimMovementSpeedModifier) * unit.aim);
}

inline Vec2 MaxSpeedVector(Vec2 position, Vec2 direction, Vec2 target, const double aimModifier = 1) {
    const Constants &constants = Constants::INSTANCE;
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
    return (Vec2{x0 - b * mult, y0 + a * mult} - position) * aimModifier;
}

inline Vec2 ResultSpeedVector(Vec2 currentVector, Vec2 targetVector) {
    const Constants &constants = Constants::INSTANCE;
    Vec2 change_vector = targetVector - currentVector;
    double norm = change_vector.norm();
    return norm > constants.unitAccelerationPerTick ? currentVector +
                                                      change_vector * (constants.unitAccelerationPerTick / norm)
                                                    : targetVector;
}

inline double RotationSpeed(double aim, const std::optional<int>& weapon) {
    const Constants &constants = Constants::INSTANCE;
    return aim > 0 ? constants.rotationSpeed -
                     (constants.rotationSpeed - constants.weapons[*weapon].aimRotationSpeed) * aim
                   : constants.rotationSpeed;
}

inline double RotationSpeed(const Unit& unit) {
    return RotationSpeed(unit.aim, unit.weapon);
}

inline Vec2 applyNewDirection(Vec2 currentDirection, Vec2 targetDirection, const double rotation_speed) {
    double curr_angle = currentDirection.toRadians();
    double angle_diff = AngleDiff(targetDirection.toRadians(), curr_angle);
    const double diff_abs = abs(angle_diff);
    if (diff_abs > rotation_speed) {
        angle_diff *= rotation_speed / diff_abs;
    }
    curr_angle += angle_diff;

    return {curr_angle};
}

inline void updateForCollision(Vec2& position, Vec2& velocity) {
    const Constants& constants = Constants::INSTANCE;
    const auto& obstacles = constants.Get(position);
    double time_remained = 1. / constants.ticksPerSecond;

    const auto nextMove = [&](const std::optional<int> prev_collision) -> std::optional<int> {
        const auto &[obstacle, point] = GetClosestCollision(position,
                                                            position + velocity * time_remained,
                                                            constants.unitRadius, prev_collision);
        if (!obstacle) {
            DRAWK('M',
                  debugInterface->addGradientSegment(position, debugging::Color(1., 0., 0., 1.),
                                                     position + velocity * time_remained,
                                                     debugging::Color(0., 1., 0., 1.), 0.03););
            position += velocity * time_remained;
            return std::nullopt;
        }
        const Obstacle &obstacleRef = **obstacle;
        const auto collision_point = FirstCollision(position, velocity, constants.unitRadius, obstacleRef,
                                                    time_remained);
        DRAWK('M', {
            auto norm = (obstacleRef.position - collision_point).toLen(constants.unitRadius);
            auto new_pt = collision_point + norm;
            norm.rotate90().toLen(2.);
            debugInterface->addSegment(new_pt - norm, new_pt + norm, 0.03,
                                       debugging::Color(0., 0., 1., 1.));
        });
        double timeChange = time_remained * (collision_point - position).norm() / (velocity * time_remained).norm();
        if (isnan(timeChange)) {
            std::cerr << "Ooops!" << std::endl;
        }
        time_remained -= timeChange;
        const Vec2 norm = (obstacleRef.position - collision_point).toLen(1.);
        velocity -= norm * velocity.dot(norm);
        DRAWK('M', debugInterface->addGradientSegment(position, debugging::Color(1., 0., 0., 1.),
                                                      collision_point,
                                                      debugging::Color(0., 1., 0., 1.), 0.03););
        position = collision_point;
        return obstacleRef.id;
    };

    if (auto obstacle = nextMove(std::nullopt); obstacle) {
        nextMove(obstacle);
    }
}

#endif //AI_CUP_22_MOVEMENT_HPP
