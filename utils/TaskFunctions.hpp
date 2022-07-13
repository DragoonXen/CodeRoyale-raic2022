//
// Created by dragoon on 12.07.2022.
//

#ifndef AI_CUP_22_TASKFUNCTIONS_H
#define AI_CUP_22_TASKFUNCTIONS_H

#include "model/UnitOrder.hpp"
#include "Util.hpp"
#include "Movement.hpp"
#include "Task.hpp"
#include "Visible.hpp"

namespace {
    using namespace model;

    constexpr int kStepsSize = 24 / 2;
    const std::vector<double> kAngleDiff = []() {
        std::vector<double> result;
        const double stepAngle = M_PI / kStepsSize;
        result.push_back(0);
        for (int i = 1; i + 1 != kStepsSize; ++i) {
            result.push_back(i * stepAngle);
            result.push_back(-i * stepAngle);
        }
        result.push_back(M_PI);
        return result;
    }();

    std::array<double, 3> kWeaponDistance = {12., 10., 14.};
    const double kAcceptableAdditionalDistance = 10.;
}

std::vector<OrderType> ApplyAttackTask(const Unit& unit, const Unit& target, const int currentTick, POrder& order) {
    const Constants& constants = Constants::INSTANCE;
    double velocity = target.velocity.norm();
    const double totalVelocity = velocity + std::max(0., velocity - 1.) + std::max(0., velocity - 2.);
    Vec2 aimTarget = target.position + target.velocity.toLen(totalVelocity) * constants.tickTime;

    const double angleDiff = AngleDiff(unit.direction.toRadians(), (aimTarget - unit.position).toRadians());
    const double angleDiffAbs = std::abs(angleDiff);
    const double rotateSpeed = RotationSpeed(unit);
    const double acceptableWeaponDistance = kWeaponDistance[*unit.weapon] + kAcceptableAdditionalDistance;
    const double currentDistance = (target.position - unit.position).norm();

    // how many ticks I need to rotate with current speed
    const double ticksToRotate = angleDiffAbs / rotateSpeed;

    // how many ticks until next shot
    int ticksToPossibleShot = unit.nextShotTick - currentTick;
    if (ticksToRotate < 1 && unit.aim + 1e-5 > 1 && ticksToPossibleShot <= 0 &&
        acceptableWeaponDistance > currentDistance) {
        order.lookPoint = aimTarget;
        order.action = std::make_shared<ActionOrder::Aim>(true);
        order.aim = true;
        return {OrderType::kRotate, OrderType::kAction};
    }

    std::vector<OrderType> result;
    // how many ticks I need to fully aim
    int ticksToAim = (int)((1. - unit.aim) / Constants::INSTANCE.weapons[*unit.weapon].aimPerTick + 1e-5);
    if (ticksToRotate + 4 > ticksToPossibleShot) {
        order.lookPoint = aimTarget;
        result.push_back(OrderType::kRotate);
    }
    if (ticksToPossibleShot <= ticksToAim + 1 && ticksToRotate <= ticksToAim + 1 &&
        acceptableWeaponDistance > currentDistance) {

        order.action = std::make_shared<ActionOrder::Aim>(false);
        order.aim = true;
        result.push_back(OrderType::kAction);
    }
    return result;
}

std::vector<OrderType> ApplyLookTo(const Vec2 point, POrder &order) {
    order.lookPoint = point;
    return {OrderType::kRotate};
}

std::vector<OrderType> ApplyMoveTo(const Unit &unit,
                                   const Vec2 newPosition,
                                   const VisibleFilter& filter,
                                   const double maxSpeed,
                                   POrder &order) {
    auto [obstacle, point] = ClosestIntersectionPoint(unit.position, newPosition, filter.closeObstacles);
    if (obstacle == nullptr) {
        order.movePoint = newPosition;
        order.maxMoveSpeed = maxSpeed;
        return {OrderType::kMove};
    }
    Vec2 dir = point - obstacle->position;
    if (dir.sqrNorm() < 1e-10) { // zero vector
        dir = (obstacle->position - unit.position).rotate90();
    }
    dir.toLen(obstacle->radius + Constants::INSTANCE.unitRadius);
    order.movePoint = obstacle->position + dir;
    order.maxMoveSpeed = maxSpeed;
    return {OrderType::kMove};
}

std::vector<OrderType>
ApplyMoveToUnitTask(const Unit &unit, const Unit &target, std::unordered_map<int, VisibleFilter> &visibilityFilters,
                    POrder &order) {
    const Constants &constants = Constants::INSTANCE;
    const auto &myFilter = visibilityFilters[unit.id];
    const double currentWeaponDistance = kWeaponDistance[*unit.weapon];
    const Vec2 direction = unit.position - target.position;
    const double startingAngle = direction.toRadians();
    for (double angleDiff : kAngleDiff) {
        const Vec2 newDirection = Vec2{startingAngle + angleDiff} * currentWeaponDistance;
        const Vec2 newPosition = target.position + newDirection;
        if (IsVisible<VisionFilter::kShootFilter>(newPosition, -newDirection, 100., target.position, myFilter)) {
            const double maxSpeed = std::max((target.position - unit.position).norm() - currentWeaponDistance, 0.) + 1;
            return ApplyMoveTo(unit, newPosition, myFilter, maxSpeed, order);
        }
    }
    return {};
}

#endif //AI_CUP_22_TASKFUNCTIONS_H
