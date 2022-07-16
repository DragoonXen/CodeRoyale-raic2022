//
// Created by dragoon on 11.07.2022.
//

#ifndef AI_CUP_22_VISIBLE_HPP
#define AI_CUP_22_VISIBLE_HPP

#include <unordered_map>
#include "model/Vec2.hpp"
#include "model/Constants.hpp"
#include "Util.hpp"
#include "model/Unit.hpp"

namespace {
    using namespace model;
}

struct VisibleFilter {
    std::vector<const Obstacle *> visionBlockingObstacles;
    std::vector<const Obstacle *> closeObstacles;
    std::vector<const Obstacle *> shootBlockingObstacles;
};

double FieldOfView(const Unit& unit) {
    const double fieldOfView = Constants::INSTANCE.fieldOfView;
    if (unit.aim == 0.) {
        return fieldOfView;
    }
    return fieldOfView - (fieldOfView - Constants::INSTANCE.weapons[*unit.weapon].aimFieldOfView) * unit.aim;
}

VisibleFilter FilterObstacles(Vec2 position, Vec2 direction, double fieldOfView) {
    const auto &constants = Constants::INSTANCE;
    double maxShootDistance = 0.;
    for (size_t i = 0; i != constants.weapons.size(); ++i) {
        maxShootDistance = std::max(maxShootDistance,
                                    constants.weapons[i].projectileSpeed * constants.weapons[i].projectileLifeTime);
    }
    Vec2 leftPt = position + Vec2(direction.toRadians() + fieldOfView * .5).toLen(constants.viewDistance);
    Vec2 rightPt = position + Vec2(direction.toRadians() - fieldOfView * .5).toLen(constants.viewDistance);
    DRAW(
            debugInterface->addSegment(position, leftPt, 0.1, debugging::Color(0., 0., 0., 0.5));
            debugInterface->addSegment(position, rightPt, 0.1, debugging::Color(0., 0., 0., 0.5));
    );
    VisibleFilter visibleFilter;
    for (const auto &obstacle: constants.obstacles) {
        const double distanceToObstacle = (obstacle.position - position).norm() - obstacle.radius;
        if (distanceToObstacle >= constants.viewDistance) {
            continue;
        }
        visibleFilter.closeObstacles.push_back(&obstacle);
        bool visible = [&]() {
            if (SegmentPointDist(obstacle.position, position, leftPt) < obstacle.radius ||
                SegmentPointDist(obstacle.position, position, rightPt) < obstacle.radius) {
                return true;
            }
            double angleDiff = abs(AngleDiff(direction.toRadians(), (obstacle.position - position).toRadians()));
            return angleDiff * 2. < fieldOfView;
        }();
        if (!visible) {
            continue;
        }
        if (constants.viewBlocking && !obstacle.canSeeThrough) {
            visibleFilter.visionBlockingObstacles.push_back(&obstacle);
        }
        if (!obstacle.canShootThrough && (distanceToObstacle < maxShootDistance)) {
            visibleFilter.shootBlockingObstacles.push_back(&obstacle);
        }
    }
    return visibleFilter;
}

enum VisionFilter {
    kShootFilter,
    kVisibilityFilter
};

bool IsReachable(Vec2 position, Vec2 point, const VisibleFilter &visibleFilter) {
    const auto &constants = Constants::INSTANCE;
    if (sqr(constants.viewDistance) < (point - position).sqrNorm()) {
        return false;
    }
    for (const auto &obstacle: visibleFilter.closeObstacles) {
        if (SegmentPointDist(obstacle->position, position, point) + 1e-8 <= obstacle->radius) {
            return false;
        }
    }
    return true;
}

template<VisionFilter filter>
bool IsVisible(Vec2 position, Vec2 direction, double fieldOfView, Vec2 point,
               const std::vector<const Obstacle *> &filteredObstacles) {
    const auto &constants = Constants::INSTANCE;
    if (sqr(constants.viewDistance) < (point - position).sqrNorm()) {
        return false;
    }
    const Vec2 vectorToPoint = point - position;
    double angleDiff = abs(AngleDiff(direction.toRadians(), vectorToPoint.toRadians()));
    if (angleDiff * 2. > fieldOfView) {
        return false;
    }
    for (const auto &obstacle: filteredObstacles) {
        if constexpr (filter == VisionFilter::kVisibilityFilter) {
            if (obstacle->canSeeThrough || !constants.viewBlocking) {
                continue;
            }
        } else {
            if (obstacle->canShootThrough) {
                continue;
            }
        }
        if (SegmentPointDist(obstacle->position, position, point) + 1e-8 <= obstacle->radius) {
            return false;
        }
    }
    return true;
}

std::pair<const Obstacle *, Vec2>
ClosestIntersectionPoint(Vec2 position, Vec2 point, const std::vector<const Obstacle *> &obstacleVector) {
    double minDistance = std::numeric_limits<double>::infinity();
    const Obstacle *closestObstacle = nullptr;
    Vec2 closestPoint = {};
    for (auto obstacle: obstacleVector) {
        Vec2 currentPoint = SegmentClosestPoint(obstacle->position, position, point);
        if ((currentPoint - obstacle->position).norm() + 0.1 > obstacle->radius + Constants::INSTANCE.unitRadius) {
            continue;
        }
        const double distanceToPoint = (currentPoint - position).sqrNorm();
        if (distanceToPoint < minDistance) {
            minDistance = distanceToPoint;
            closestObstacle = obstacle;
            closestPoint = currentPoint;
        }
    }
    return {closestObstacle, closestPoint};
}

template<VisionFilter filter>
bool
IsVisible(Vec2 position, Vec2 direction, double fieldOfView, Vec2 point, const VisibleFilter &visibleFilter) {
    const auto &constants = Constants::INSTANCE;
    if (sqr(constants.viewDistance) < (point - position).sqrNorm()) {
        return false;
    }
    const Vec2 vectorToPoint = point - position;
    double angleDiff = abs(AngleDiff(direction.toRadians(), vectorToPoint.toRadians()));
    if (angleDiff * 2. > fieldOfView) {
        return false;
    }
    const auto &obstacleVector = (filter == VisionFilter::kVisibilityFilter ?
                                      visibleFilter.visionBlockingObstacles : visibleFilter.shootBlockingObstacles);
    for (const auto &obstacle: obstacleVector) {
        if (SegmentPointDist(obstacle->position, position, point) + 1e-8 <= obstacle->radius) {
            return false;
        }
    }
    return true;
}

template<VisionFilter filter>
bool IsVisible(Vec2 position, Vec2 point, const std::vector<const Obstacle *> &obstacleVector) {
    const auto &constants = Constants::INSTANCE;
    if (sqr(constants.viewDistance) < (point - position).sqrNorm()) {
        return false;
    }
    for (const auto &obstacle: obstacleVector) {
        if constexpr (filter == VisionFilter::kVisibilityFilter) {
            if (obstacle->canSeeThrough || !constants.viewBlocking) {
                continue;
            }
        } else {
            if (obstacle->canShootThrough) {
                continue;
            }
        }
        if (SegmentPointDist(obstacle->position, position, point) + 1e-8 <= obstacle->radius) {
            return false;
        }
    }
    return true;
}

template<VisionFilter filter>
bool IsVisible(Vec2 position, double lessAngle, double moreAngle, Vec2 point,
               const std::vector<const Obstacle *> &obstacleVector) {
    const auto &constants = Constants::INSTANCE;
    if (sqr(constants.viewDistance) < (point - position).sqrNorm()) {
        return false;
    }
    if (!AngleBetween((point - position).toRadians(), lessAngle, moreAngle)) {
        return false;
    }

    for (const auto &obstacle: obstacleVector) {
        if constexpr (filter == VisionFilter::kVisibilityFilter) {
            if (obstacle->canSeeThrough || !constants.viewBlocking) {
                continue;
            }
        } else {
            if (obstacle->canShootThrough) {
                continue;
            }
        }
        if (SegmentPointDist(obstacle->position, position, point) + 1e-8 <= obstacle->radius) {
            return false;
        }
    }
    return true;
}

template<VisionFilter filter>
bool
IsVisible(Vec2 point, const std::vector<Unit *> &units, const std::unordered_map<int, VisibleFilter> &filters) {
    for (const auto unit: units) {
        if (IsVisible<filter>(unit->position, unit->direction, unit->currentFieldOfView, point, filters.at(unit->id))) {
            return true;
        }
    }
    return false;
}

#endif //AI_CUP_22_VISIBLE_HPP
