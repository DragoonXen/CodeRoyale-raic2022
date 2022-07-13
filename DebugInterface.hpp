#ifndef _DEBUG_INTERFACE_HPP_
#define _DEBUG_INTERFACE_HPP_

#include "TcpStream.hpp"
#include "debugging/DebugCommand.hpp"
#include "debugging/DebugState.hpp"
#include <memory>

class DebugInterface {
public:
    static DebugInterface* INSTANCE;

    DebugInterface(TcpStream* stream);

    void addPlacedText(model::Vec2 position, std::string text, model::Vec2 alignment, double size, debugging::Color color);
    void addCircle(model::Vec2 position, double radius, debugging::Color color);
    void addGradientCircle(model::Vec2 position, double radius, debugging::Color innerColor, debugging::Color outerColor);
    void addRing(model::Vec2 position, double radius, double width, debugging::Color color);
    void addPie(model::Vec2 position, double radius, double startAngle, double endAngle, debugging::Color color);
    void addArc(model::Vec2 position, double radius, double width, double startAngle, double endAngle, debugging::Color color);
    void addRect(model::Vec2 bottomLeft, model::Vec2 size, debugging::Color color);
    void addPolygon(std::vector<model::Vec2> vertices, debugging::Color color);
    void addGradientPolygon(std::vector<debugging::ColoredVertex> vertices);
    void addSegment(model::Vec2 firstEnd, model::Vec2 secondEnd, double width, debugging::Color color);
    void addGradientSegment(model::Vec2 firstEnd, debugging::Color firstColor, model::Vec2 secondEnd, debugging::Color secondColor, double width);
    void addPolyLine(std::vector<model::Vec2> vertices, double width, debugging::Color color);
    void addGradientPolyLine(std::vector<debugging::ColoredVertex> vertices, double width);
    void add(std::shared_ptr<debugging::DebugData> debugData);
    void clear();
    void setAutoFlush(bool enable);
    void flush();
    void send(std::shared_ptr<debugging::DebugCommand> command);
    debugging::DebugState getState();

private:
    TcpStream* stream;
};

#include <sstream>


template <typename T>
std::string to_string_p(const T a_value, const int n = 6)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

#ifdef DEBUG_INFO

template<typename T>
void Draw(char key, const T function) {
    if (!debugging::DebugState::Enabled(key)) return;
    function();
}
template<typename T>
void Draw(const T function) {
    function();
}
#define DRAW(a) Draw(([&](){auto debugInterface = DebugInterface::INSTANCE;if (debugInterface == nullptr) return;a}))
#define DRAWK(key, lambda) Draw(key,([&](){auto debugInterface = DebugInterface::INSTANCE;if (debugInterface == nullptr) return;lambda}))
#else
inline void Draw() {
}
#define DRAW(a) Draw()
#define DRAWK(key, lambda) Draw()
#endif
#endif