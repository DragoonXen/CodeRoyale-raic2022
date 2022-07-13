#ifndef __MODEL_CONSTANTS_HPP__
#define __MODEL_CONSTANTS_HPP__

#include "Stream.hpp"
#include "model/Obstacle.hpp"
#include "model/SoundProperties.hpp"
#include "model/Vec2.hpp"
#include "model/WeaponProperties.hpp"
#include <optional>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#define _USE_MATH_DEFINES
#include <math.h>

namespace model {

// Non changing game state
class Constants {
public:
    static model::Constants INSTANCE;

    // Number of ticks per game second
    double ticksPerSecond;
    double tickTime;
    // Starting number of units in each team
    int teamSize;
    // Initial zone radius
    double initialZoneRadius;
    // Speed of zone radius
    double zoneSpeed;
    double zoneSpeedPerTick;
    // Damage dealt to units outside of the zone per second
    double zoneDamagePerSecond;
    double zoneDamagePerTick;
    // Unit spawning time
    double spawnTime;
    // Damage dealt to units trying to spawn in incorrect position per second
    double spawnCollisionDamagePerSecond;
    // Time required to perform looting actions (in seconds)
    double lootingTime;
    // Number of bot players (teams)
    int botPlayers;
    // Units' radius
    double unitRadius;
    // Max units' health
    double unitHealth;
    // Health automatically restored per second
    double healthRegenerationPerSecond;
    // Time until automatic health regeneration since last health damage (in seconds)
    double healthRegenerationDelay;

    int healthRegenerationDelayTicks;
    // Max value of unit's shield
    double maxShield;
    // Initial value of unit's shield
    double spawnShield;
    // Initial number of extra lives for units
    int extraLives;
    // Zone radius after which respawning is disabled
    double lastRespawnZoneRadius;
    // Units' field of view without aiming (in degrees)
    double fieldOfView;
    // Units' view distance
    double viewDistance;
    // Whether units' view is blocked by obstacles
    bool viewBlocking;
    // Unit rotation speed without aiming (degrees per second)
    double rotationSpeed;
    // Units' movement speed while spawning
    double spawnMovementSpeed;
    // Max unit speed when walking forward
    double maxUnitForwardSpeed;
    // Max unit speed when walking backward
    double maxUnitBackwardSpeed;
    double unitMovementCircleShift;
    double unitMovementCircleRadius;
    // Max unit acceleration
    double unitAcceleration;
    double unitAccelerationPerTick;
    // Whether a unit can damage units of the same team
    bool friendlyFire;
    // Score given for killing enemy unit
    double killScore;
    // Score multiplier for damaging enemy units
    double damageScoreMultiplier;
    // Score given for every team killed before you
    double scorePerPlace;
    // List of properties of every weapon type
    std::vector<model::WeaponProperties> weapons;
    // Starting weapon with which units spawn, or None
    std::optional<int> startingWeapon;
    // Ammo for starting weapon given when unit spawns
    int startingWeaponAmmo;
    // Max number of shield potions in unit's inventory
    int maxShieldPotionsInInventory;
    // Amount of shield restored using one potion
    double shieldPerPotion;
    // Time required to perform action of using shield potion
    double shieldPotionUseTime;
    // List of properties of every sound type
    std::vector<model::SoundProperties> sounds;
    // Sound type index when moving (starting with 0), or None
    std::optional<int> stepsSoundTypeIndex;
    // Distance when steps sound will be 100% probability
    double stepsSoundTravelDistance;
    // List of obstacles on the map
    std::vector<model::Obstacle> obstacles;

    static constexpr int collision_zone_offset = 30;

    std::vector<std::vector<std::vector<const Obstacle*>>> obstacle_matrix;
    int minX, minY;

    Constants() = default;

    Constants(double ticksPerSecond, int teamSize, double initialZoneRadius, double zoneSpeed, double zoneDamagePerSecond, double spawnTime, double spawnCollisionDamagePerSecond, double lootingTime, int botPlayers, double unitRadius, double unitHealth, double healthRegenerationPerSecond, double healthRegenerationDelay, double maxShield, double spawnShield, int extraLives, double lastRespawnZoneRadius, double fieldOfView, double viewDistance, bool viewBlocking, double rotationSpeed, double spawnMovementSpeed, double maxUnitForwardSpeed, double maxUnitBackwardSpeed, double unitAcceleration, bool friendlyFire, double killScore, double damageScoreMultiplier, double scorePerPlace, std::vector<model::WeaponProperties> weapons, std::optional<int> startingWeapon, int startingWeaponAmmo, int maxShieldPotionsInInventory, double shieldPerPotion, double shieldPotionUseTime, std::vector<model::SoundProperties> sounds, std::optional<int> stepsSoundTypeIndex, double stepsSoundTravelDistance, std::vector<model::Obstacle> obstacles);

    // Read Constants from input stream
    static Constants readFrom(InputStream& stream);

    // Write Constants to output stream
    void writeTo(OutputStream& stream) const;

    inline int toI(double val) const {
        return val + 0.5;
    }

    void Update() {
        tickTime = 1. / ticksPerSecond;
        zoneSpeedPerTick = zoneSpeed * tickTime;
        zoneDamagePerTick = zoneDamagePerSecond * tickTime;
        healthRegenerationDelayTicks = (int)std::round(healthRegenerationDelay / tickTime);
        unitAccelerationPerTick = unitAcceleration / ticksPerSecond;
        unitMovementCircleShift = (maxUnitForwardSpeed - maxUnitBackwardSpeed) * .5;
        unitMovementCircleRadius = (maxUnitForwardSpeed + maxUnitBackwardSpeed) * .5;
        rotationSpeed *= M_PI / 180. / ticksPerSecond;
        fieldOfView *= M_PI / 180.;
        for (auto& weapon : weapons) {
            weapon.aimRotationSpeed *= M_PI / 180. / ticksPerSecond;
            weapon.aimFieldOfView *= M_PI / 180.;
            weapon.aimPerTick = tickTime / weapon.aimTime;
        }

        double minX, minY;
        minX = minY = std::numeric_limits<double>::infinity();
        double maxX, maxY;
        maxX = maxY = -std::numeric_limits<double>::infinity();
        for (const auto& obstacle : obstacles) {
            minX = std::min(minX, obstacle.position.x - obstacle.radius);
            minY = std::min(minY, obstacle.position.y - obstacle.radius);
            maxX = std::max(maxX, obstacle.position.x + obstacle.radius);
            maxY = std::max(maxY, obstacle.position.y - obstacle.radius);
        }
        minX -= collision_zone_offset;
        minY -= collision_zone_offset;
        maxX += collision_zone_offset;
        maxY += collision_zone_offset;
        this->minX = toI(minX);
        this->minY = toI(minY);
        std::vector<std::vector<const Obstacle *>> base((int) (maxY - minY));
        obstacle_matrix.resize((int) (maxX - minX), base);
        constexpr double addition_radius = 2.;
        const double addition =
                std::max(unitRadius, /*weapons*/ weapons[2].projectileSpeed / ticksPerSecond) + addition_radius;
        for (const auto &obstacle: obstacles) {
            const int x = toI(obstacle.position.x) - minX;
            const int y = toI(obstacle.position.y) - minY;
            const int diff = obstacle.radius + addition;
            const int toX = x + diff + 1;
            const int toY = y + diff + 1;
            for (int cx = x - diff; cx != toX; ++cx) {
                for (int cy = y - diff; cy != toY; ++cy) {
                    obstacle_matrix[cx][cy].push_back(&obstacle);
                }
            }
        }
    }

    inline const std::vector<const Obstacle *> &Get(double x, double y) const {
        const int posX = toI(x) - minX;
        const int posY = toI(y) - minY;
        return obstacle_matrix[posX][posY];
    }

    inline const std::vector<const Obstacle *> &Get(Vec2 pos) const {
        return Get(pos.x, pos.y);
    }

    // Get string representation of Constants
    std::string toString() const;
};

}

#endif