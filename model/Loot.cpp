#include "Loot.hpp"

namespace model {

    namespace {
        std::string WriteTagString(LootType tag, int weaponType, int amount) {
            std::stringstream ss;
            switch (tag) {
                case LootType::Weapon:
                    ss << "Item::Weapon { ";
                    ss << "typeIndex: ";
                    ss << weaponType;
                    ss << " }";
                    break;
                case LootType::ShieldPotions:
                    ss << "Item::ShieldPotions { ";
                    ss << "amount: ";
                    ss << amount;
                    ss << " }";
                    break;
                case LootType::Ammo:
                    ss << "Item::Ammo { ";
                    ss << "weaponTypeIndex: ";
                    ss << weaponType;
                    ss << ", ";
                    ss << "amount: ";
                    ss << amount;
                    ss << " }";
                    break;
            }
            return ss.str();
        }

        std::tuple<LootType, int, int> readIFrom(InputStream &stream) {
            switch (stream.readInt()) {
                case 0:
                    return {LootType::Weapon, stream.readInt(), 0};
                case 1:
                    return {LootType::ShieldPotions, 0, stream.readInt()};
                case 2:
                    return {LootType::Ammo, stream.readInt(), stream.readInt()};
                default:
                    throw std::runtime_error("Unexpected tag value");
            }
        }
    }

    Loot::Loot(int id, model::Vec2 position, LootType tag, int weaponTypeIndex, int amount) : id(id),
                                                                                              position(position),
                                                                                              tag(tag), weaponTypeIndex(
                    weaponTypeIndex), amount(amount) {}

// Read Loot from input stream
    Loot Loot::readFrom(InputStream &stream) {
        int id = stream.readInt();
        model::Vec2 position = model::Vec2::readFrom(stream);
        const auto [tag, weaponTypeIndex, amount] = readIFrom(stream);
        return Loot(id, position, tag, weaponTypeIndex, amount);
    }

// Write Loot to output stream
    void Loot::writeTo(OutputStream &stream) const {
        stream.write(id);
        position.writeTo(stream);
        stream.write(tag);
        if (tag != 1) {
            stream.write(weaponTypeIndex);
        }
        if (tag != 0) {
            stream.write(amount);
        }
    }

// Get string representation of Loot
    std::string Loot::toString() const {
        std::stringstream ss;
        ss << "Loot { ";
        ss << "id: ";
        ss << id;
        ss << ", ";
        ss << "position: ";
        ss << position.toString();
        ss << ", ";
        ss << "item: ";
        ss << WriteTagString(tag, weaponTypeIndex, amount);
        ss << " }";
        return ss.str();
    }

}