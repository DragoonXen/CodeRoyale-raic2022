#ifndef __MODEL_VEC2_HPP__
#define __MODEL_VEC2_HPP__

#include "Stream.hpp"
#include <sstream>
#include <string>
#include <cmath>

namespace model {

// 2 dimensional vector.
class Vec2 {
public:
    // `x` coordinate of the vector
    double x;
    // `y` coordinate of the vector
    double y;

    Vec2() : x(0), y(0) {}

    Vec2(double x, double y);

    // Read Vec2 from input stream
    static Vec2 readFrom(InputStream& stream);

    // Write Vec2 to output stream
    void writeTo(OutputStream& stream) const;

    // Get string representation of Vec2
    std::string toString() const;

    Vec2 operator+(const Vec2 &second) const {
        return {this->x + second.x, this->y + second.y};
    }

    Vec2 operator-(const Vec2 &second) const {
        return {this->x - second.x, this->y - second.y};
    }

    Vec2 operator*(const double val) const {
        return {this->x * val, this->y * val};
    }

    Vec2& operator*=(const double val) {
        this->x *= val;
        this->y *= val;
        return *this;
    }

    Vec2 &toLen(const double val) {
        const double norm = this->norm();
        this->x *= val / norm;
        this->y *= val / norm;
        return *this;
    }

    Vec2 toLen(const double val) const {
        const double norm = this->norm();
        return {this->x * val / norm, this->y * val / norm};
    }

    double norm() const {
        return std::sqrt(sqrNorm());
    }

    double sqrNorm() const {
        return x * x + y * y;
    }

};

}



#endif