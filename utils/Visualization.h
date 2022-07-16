//
// Created by dragoon on 16.07.2022.
//

#ifndef AI_CUP_22_VISUALIZATION_H
#define AI_CUP_22_VISUALIZATION_H


#include "DebugInterface.hpp"

inline void
DrawCross(const model::Vec2 point, const double crossSize, debugging::Color color, DebugInterface *debugInterface) {
    debugInterface->addSegment(point + model::Vec2{crossSize, crossSize}, point - model::Vec2{crossSize, crossSize},
                               0.1, color);
    debugInterface->addSegment(point + model::Vec2{crossSize, -crossSize}, point - model::Vec2{crossSize, -crossSize},
                               0.1, color);
}

#endif //AI_CUP_22_VISUALIZATION_H
