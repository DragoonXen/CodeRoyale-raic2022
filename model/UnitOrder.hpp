#ifndef __MODEL_UNIT_ORDER_HPP__
#define __MODEL_UNIT_ORDER_HPP__

#include "Stream.hpp"
#include "model/ActionOrder.hpp"
#include "model/Vec2.hpp"
#include "Unit.hpp"
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace model {

// Order for specific unit
class UnitOrder {
public:
    // Target moving velocity
    model::Vec2 targetVelocity;
    // Target view direction (vector length doesn't matter)
    model::Vec2 targetDirection;
    // Order to perform an action, or None
    std::optional<std::shared_ptr<model::ActionOrder>> action;

    UnitOrder() = default;

    explicit UnitOrder(const Unit &unit) : targetVelocity(unit.velocity), targetDirection(unit.direction), action() {};

    UnitOrder(model::Vec2 targetVelocity, model::Vec2 targetDirection, std::optional<std::shared_ptr<model::ActionOrder>> action);

    // Read UnitOrder from input stream
    static UnitOrder readFrom(InputStream& stream);

    // Write UnitOrder to output stream
    void writeTo(OutputStream& stream) const;

    // Get string representation of UnitOrder
    std::string toString() const;
};

}

#endif