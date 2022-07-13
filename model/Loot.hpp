#ifndef __MODEL_LOOT_HPP__
#define __MODEL_LOOT_HPP__

#include "Stream.hpp"
#include "model/Vec2.hpp"
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace model {

    enum LootType {
        Weapon,
        ShieldPotions,
        Ammo
    };

// Loot lying on the ground
class Loot {
public:
    // Unique id
    int id;
    // Position
    model::Vec2 position;

    LootType tag;
    // Weapon type index (starting with 0)
    int weaponTypeIndex;
    // Amount of ammo
    int amount;

    int lastSeenTick;

    Loot(int id, model::Vec2 position, LootType tag, int weaponTypeIndex, int amount);

    // Read Loot from input stream
    static Loot readFrom(InputStream& stream);

    // Write Loot to output stream
    void writeTo(OutputStream& stream) const;

    // Get string representation of Loot
    std::string toString() const;
};

}

#endif