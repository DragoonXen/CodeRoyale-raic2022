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
#include "Visualization.h"

namespace {
    using namespace model;

    using DestinationWithMaxSpeed = std::tuple<Vec2, double>;

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

std::vector<OrderType>
ApplyAttackTask(const Unit &unit,
                const std::vector<Unit*>& myUnits,
                const Unit &target,
                const int currentTick,
                std::unordered_map<int, VisibleFilter> &visibilityFilters,
                std::unordered_map<int, std::list<TickSpeedDirUpdate>> unitMovementMem,
                POrder &order) {
    const Constants& constants = Constants::INSTANCE;

    auto& movementList = unitMovementMem[target.id];
    const Vec2 aimTarget = [&unit, &movementList, &target]() -> Vec2 {
        // move fast enough
        if (movementList.back().first > 3. &&
            std::abs(AngleDiff(movementList.back().second, target.direction.toRadians())) < M_PI / 10.) {
            Unit copy = target;
            MoveRule rule;
            rule.moveDirection = target.position + Vec2(movementList.back().second) * 50;
            rule.lookDirection = rule.moveDirection;
            rule.keepAim = false;
            rule.speedLimit = std::numeric_limits<double>::infinity();
            const Constants &constants = Constants::INSTANCE;
            double bulletSpeed = constants.weapons[*unit.weapon].projectileSpeed * constants.tickTime;
            for (size_t tick = 1; tick <= 30; ++tick) {
                ApplyAvoidRule(copy, rule);
                DRAW({
                         debugInterface->addRing(copy.position, 1., .01, debugging::Color(0., 0., 1., .8));
                     });
                double distance = (copy.position - unit.position).norm() - constants.unitRadius * 2;
                if (bulletSpeed * tick >= distance) {
                    return copy.position;
                }
            }
        }
        double velocity = target.velocity.norm();
        const double totalVelocity = velocity + std::max(0., velocity - 1.) + std::max(0., velocity - 2.);
        return target.position + (velocity > 1e-5 ?
                                  target.velocity.toLen(totalVelocity) * constants.tickTime :
                                  target.direction * 0.3);
    }();

    DRAW(DrawCross(aimTarget, 0.3, debugging::Color(1., 0., 0., 0.5), debugInterface););

    const double angleDiff = AngleDiff(unit.direction.toRadians(), (aimTarget - unit.position).toRadians());
    const double angleDiffAbs = std::abs(angleDiff);
    const double rotateSpeed = RotationSpeed(unit);
    const double acceptableWeaponDistance = kWeaponDistance[*unit.weapon] + kAcceptableAdditionalDistance;
    const double currentDistance = (target.position - unit.position).norm();

    // how many ticks I need to rotate with current speed
    const double ticksToRotate = angleDiffAbs / rotateSpeed;

    // how many ticks until next shot
    int ticksToPossibleShot = unit.nextShotTick - currentTick;
    if (unit.action) {
        ticksToPossibleShot = std::max(ticksToPossibleShot, unit.action->finishTick - currentTick);
    }
    if (unit.remainingSpawnTime) {
        ticksToPossibleShot = std::max(ticksToPossibleShot,
                                       (int) (*unit.remainingSpawnTime / constants.tickTime + 1e-2));
    }
    if (ticksToRotate < 1 && unit.aim + 1e-5 > 1 && ticksToPossibleShot <= 0 &&
        acceptableWeaponDistance > currentDistance) {
        order.lookPoint = aimTarget;
        const auto &myFilter = visibilityFilters[unit.id];
        bool shoot = unit.lastSeenTick == target.lastSeenTick &&
                     IsVisible<VisionFilter::kShootFilter>(unit.position, unit.direction, 100., aimTarget, myFilter);
        if (shoot) {
            for (const auto& myUnit : myUnits) {
                if (myUnit->id == unit.id) {
                    continue;
                }
                const double timeToCoverDistance =
                        ((myUnit->position - unit.position).norm() - constants.unitRadius * 2) /
                        constants.weapons[*unit.weapon].projectileSpeed;
                auto myUnitPosition = myUnit->position + (myUnit->velocity * timeToCoverDistance);
                const double shootToUnitSqrDistance =
                        SegmentPointSqrDist(myUnitPosition, unit.position, aimTarget);
                if (shootToUnitSqrDistance < sqr(Constants::INSTANCE.unitRadius + 0.5)) {
                    shoot = false;
                    break;
                }
            }
        }
        order.action = std::make_shared<ActionOrder::Aim>(shoot);
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
    DRAW(DrawCross(point, 0.3, debugging::Color(0., 0., 1., 0.5), debugInterface););
    return {OrderType::kRotate};
}

std::vector<OrderType> ApplyPickUp(const Unit& unit, const Loot& loot, POrder &order) {
    if ((loot.position - unit.position).sqrNorm() > sqr(Constants::INSTANCE.unitRadius)) {
        return {};
    }
    order.action = std::make_shared<ActionOrder::Pickup>(loot.id);
    return {OrderType::kAction};
}

std::vector<OrderType> ApplyMoveTo(const Unit &unit,
                                   const Vec2 newPosition,
                                   const VisibleFilter &filter,
                                   const double maxSpeed,
                                   POrder &order) {
    DRAW(DrawCross(newPosition, 0.5, debugging::Color(1., 0., 1., 0.5), debugInterface););
    if (unit.remainingSpawnTime.has_value()) {
        order.movePoint = newPosition;
        order.maxMoveSpeed = std::min(maxSpeed, (newPosition - unit.position).sqrNorm());
        return {OrderType::kMove};
    }
    auto [obstacle, point] = ClosestIntersectionPoint(unit.position, newPosition, filter.closeObstacles);
    if (obstacle == nullptr) {
        order.movePoint = newPosition;
        order.maxMoveSpeed = maxSpeed;
        return {OrderType::kMove};
    }
    auto points = TangentialPoints(unit.position, obstacle->position, obstacle->radius + Constants::INSTANCE.unitRadius);
    order.movePoint = (newPosition - points.second).sqrNorm() > (newPosition - points.first).sqrNorm() ? points.first
                                                                                                       : points.second;

    order.maxMoveSpeed = maxSpeed;
    return {OrderType::kMove};
}

std::any
ApplyMoveToUnitTask(const Unit &unit, const std::vector<Unit *> &myUnits, const Unit &target, VisibleFilter &myFilter) {
    const Constants &constants = Constants::INSTANCE;
    const double currentWeaponDistance = kWeaponDistance[*unit.weapon];
    const Vec2 direction = unit.position - target.position;
    const double startingAngle = direction.toRadians();
    const double distance = (target.position - unit.position).norm();
    for (double angleDiff : kAngleDiff) {
        const Vec2 newDirection = Vec2{startingAngle + angleDiff} * currentWeaponDistance;
        const Vec2 newPosition = target.position + newDirection;
        const bool ignorePosition = [&unit, &myUnits, &newPosition]() {
            for (const auto myOtherUnit: myUnits) {
                if (myOtherUnit->id == unit.id || myOtherUnit->remainingSpawnTime.has_value()) {
                    continue;
                }
                const double shootToUnitSqrDistance =
                        SegmentPointSqrDist(myOtherUnit->position, unit.position, newPosition);
                if (shootToUnitSqrDistance < sqr(Constants::INSTANCE.unitRadius * 2 + 0.5)) {
                    return true;
                }
            }
            return false;
        }();
        if (ignorePosition) {
            continue;
        }
        if (unit.lastSeenTick == target.lastSeenTick) {
            if (IsVisible<VisionFilter::kShootFilter>(newPosition, -newDirection, 100., target.position, myFilter)) {
                const double maxSpeed = currentWeaponDistance > distance ? std::numeric_limits<double>::infinity() :
                                        std::max(distance - currentWeaponDistance, 0.) + 1;
                return DestinationWithMaxSpeed{newPosition, maxSpeed};
            }
        } else {
            if (IsVisible<VisionFilter::kVisibilityFilter>(newPosition, -newDirection, 100., target.position, myFilter)) {
                const double maxSpeed = currentWeaponDistance > distance ? std::numeric_limits<double>::infinity() :
                                        std::max(distance - currentWeaponDistance, 0.) + 1;
                return DestinationWithMaxSpeed{newPosition, maxSpeed};
            }
        }
    }
    return {};
}

std::vector<OrderType>
ApplyBattleMovement(const Unit &unit, std::vector<std::vector<std::pair<int, double>>> &dangerMatrix,
                    const model::Game &game, const VisibleFilter &filter, POrder &order) {
    constexpr int kSearchRange = 5;
    constexpr int kSearchRangeSqr = kSearchRange * kSearchRange;
    Vec2 basePosition(Constants::toI(unit.position.x), Constants::toI(unit.position.y));
    Vec2 bestPosition = unit.position;
    double minDanger = std::numeric_limits<double>::infinity();
    for (int i = -kSearchRange; i <= kSearchRange; ++i) {
        for (int j = -kSearchRange; j <= kSearchRange; ++j) {
            if (i * i + j * j > kSearchRangeSqr) {
                continue;
            }
            auto pos = basePosition + Vec2(i, j);
            auto danger = EvaluateDangerIncludeObstacles(unit.id, pos, dangerMatrix, game);
            if (danger < minDanger) {
                minDanger = danger;
                bestPosition = pos;
            }
        }
    }
    if ((bestPosition - unit.position).sqrNorm() < 1e-7) {
        bestPosition = unit.position + Vec2(0.1, 0.1);
    }
    return ApplyMoveTo(unit, bestPosition, filter, std::numeric_limits<double>::infinity(), order);
}

#endif //AI_CUP_22_TASKFUNCTIONS_H
