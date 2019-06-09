#ifndef Vec3_h_
#define Vec3_h_

#include "RGB.h"
#include <cmath>

// 3D-vector
template <typename T>
struct Vec3 {
    // Members.
    union {
        struct {
            T x, y, z;
        };
        T D[3];
    };
    RGB color;

    inline Vec3() :
        x(0), y(0), z(0) {}
    inline Vec3(T _x, T _y, T _z) :
        x(_x), y(_y), z(_z) {}
    inline Vec3(T _x, T _y, T _z, RGB _color) :
        x(_x), y(_y), z(_z), color(_color) {}

    inline bool nonNull()
        {return x != 0 || y != 0 || z != 0;}

    inline T& operator[](unsigned int i)
        {return D[i];}
    inline const T& operator[](unsigned int i) const
        {return D[i];}

     inline Vec3 operator+(const Vec3& p) const
        {return Vec3(x + p.x, y + p.y, z + p.z);}
    inline Vec3 operator-(const Vec3& p) const
        {return Vec3(x - p.x, y - p.y, z - p.z);}

    inline Vec3 cmul(const Vec3& p) const
        {return Vec3(x * p.x, y * p.y, z * p.z);}
    inline Vec3 cdiv(const Vec3& p) const
        {return Vec3(x / p.x, y / p.y, z / p.z);}

    inline Vec3 operator*(T t) const
        {return Vec3(x * t, y * t, z * t);}
    inline Vec3 operator/(T t) const
        {return Vec3(x / t, y / t, z / t);}

    inline Vec3& operator+=(const Vec3& r)
        {x += r.x; y += r.y; z += r.z; return *this;}
    inline Vec3& operator-=(const Vec3& r)
        {x -= r.x; y -= r.y; z -= r.z; return *this;}

    inline Vec3& operator*=(T t)
        {x *= t; y *= t; z *= t; return *this;}
    inline Vec3& operator/=(T t)
        {x /= t; y /= t; z /= t; return *this;}

    inline T operator*(const Vec3& p) const
        {return x * p.x + y * p.y + z * p.z;}

    inline T norm() const
        {return std::sqrt(x*x + y*y + z*z);}
    inline T normSquared() const
        {return x*x + y*y + z*z;}

    inline void normalize()
        {*this /= norm();}
    inline Vec3 normalized() const
        {return *this / norm();}

    inline T distance(const Vec3& r) const
        {return (*this - r).norm();}
    inline T squareDistance(const Vec3& r) const
        {return (*this - r).normSquared();}

    Vec3 operator^(const Vec3& r) const
    {
        return Vec3(
                y * r.z - z * r.y,
                z * r.x - x * r.z,
                x * r.y - y * r.x
                );
    }

    void max(const Vec3& p)
    {
        x = p.x > x ? p.x : x;
        y = p.y > y ? p.y : y;
        z = p.z > z ? p.z : z;
    }

    void min(const Vec3& p)
    {
        x = p.x < x ? p.x : x;
        y = p.y < y ? p.y : y;
        z = p.z < z ? p.z : z;
    }
};

template <typename T>
inline Vec3<T> operator*(T t, const Vec3<T>& v) {
    return Vec3<T>(v.x * t, v.y * t, v.z * t);
}

typedef Vec3<double> Vec3d;

#endif
