//
// Created by dragoon on 11.07.2022.
//

#ifndef AI_CUP_22_TASK_HPP
#define AI_CUP_22_TASK_HPP

#include <string>
#include <optional>
#include <functional>
#include "model/Vec2.hpp"
#include <any>

namespace {
    using namespace model;
}

enum class OrderType {
    kMove,
    kRotate,
    kAction,
    kItemsCount
};

struct POrder;

struct Task {
    std::string description;
    int type;
    int unitId;
    std::vector<OrderType> actionTypes;
    std::any taskData;
    std::any evalData;
    double score;
    double penalty;

    explicit Task(int type, int unitId, std::string description, std::vector<OrderType> actionTypes) :
            type(type),
            description(std::move(description)),
            unitId(unitId),
            actionTypes(std::move(actionTypes)),
            taskData(),
            penalty(0) {}

    std::function<std::tuple<double, std::any>()> preEval;
    std::function<std::vector<OrderType>(const std::any& evalData, POrder &)> func;

    bool operator<(const Task &other) const {
        return this->score < other.score;
    }
};

struct POrder {
    model::Vec2 movePoint;
    double maxMoveSpeed;
    model::Vec2 lookPoint;
    bool aim;
    std::optional<std::shared_ptr<model::ActionOrder>> action;
    std::array<int, (int) OrderType::kItemsCount> picked;
    std::array<std::string, (int) OrderType::kItemsCount> description;

    POrder() : aim(false) {
        picked.fill(-1);
    }

    bool IsAbleToAcceptTask(const std::vector<OrderType> &orderTypes) {
        return std::all_of(orderTypes.begin(), orderTypes.end(),
                           [&picked = this->picked](const auto &type) { return picked[(int) type] < 0; });
    }

    inline bool Accept(const Task &task) {
        std::vector<OrderType> orderTypes = task.func(task.evalData, *this);
        for (auto &type: orderTypes) {
            picked[(int) type] = task.type;
            DRAW({
                     this->description[(int) type] =
                             to_string_p(task.score, 2) + "|" + to_string_p(task.penalty, 2) + "| " + task.description;
                 });
        }
        return !orderTypes.empty();
    }

    inline MoveRule toMoveRule() const {
        MoveRule result;
        VERIFY(picked[(int) OrderType::kMove] >= 0, "Move not set");
        result.moveDirection = movePoint;
        result.speedLimit = maxMoveSpeed;
        if (picked[(int) OrderType::kRotate] >= 0) {
            result.lookDirection = lookPoint;
        }
        result.keepAim = picked[(int) OrderType::kAction] >= 0 && aim;
        return result;
    };
};

/*
 * class Pickup;
    // Use shield potion
    class UseShieldPotion;
    // Drop shield potions on the ground
    class DropShieldPotions;
    // Drop current weapon
    class DropWeapon;
    // Drop ammo
    class DropAmmo;
    // Start/continue aiming
    class Aim;
 */

struct MoveTask {
    std::string description;
    Vec2 targetPoint;
};

struct AttackTask {
    Vec2 targetPoint;
};

struct PickTask {

};

struct DrinkTask {

};

#endif //AI_CUP_22_TASK_HPP
