//
// Created by dragoon on 11.07.2022.
//

#ifndef AI_CUP_22_TASK_H
#define AI_CUP_22_TASK_H

#include <string>
#include <optional>
#include "model/Vec2.hpp"

namespace {
    using namespace model;
}

enum class TaskType {
    kMove,
    kAttack,
    kPickUp,
    kDrink,
    kDrop
};

struct Task {
    std::string description;

    Vec2 targetPoint;
    int id;
    int type;
    int amount;
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

#endif //AI_CUP_22_TASK_H
