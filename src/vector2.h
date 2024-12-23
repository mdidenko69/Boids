#ifndef _BOIDS_VECTOR_H_
#define _BOIDS_VECTOR_H_

#include <cmath>
#include <cassert>
// The Pvector class implements Euclidian vectors -- that is, each vector has
// both a magnitude and a direction. We use Pvectors for implementing movement
// and the three Boid rules -- cohesion, separation, and matching velocity
// through the use of acceleration, force, and velocity vectors.

namespace boids {

template<typename T>
class Vector2 {
public:
  T x{};
  T y{};

  Vector2() = default;
  
  Vector2(T _x, T _y) : x(_x), y(_y) {}

  Vector2<T>& operator+= (const Vector2<T>& rh) {
    x += rh.x;
    y += rh.y;
    return *this;
  }
  Vector2<T>& operator+= (T rh) {
    x += rh;
    y += rh;
    return *this;
  }
  Vector2<T>& operator-= (const Vector2<T>& rh) {
    x -= rh.x;
    y -= rh.y;
    return *this;
  }
  Vector2<T>& operator-= (T rh) {
    x -= rh;
    y -= rh;
    return *this;
  }
  Vector2<T>& operator*= (const Vector2<T>& rh) {
    x *= rh.x;
    y *= rh.y;
    return *this;
  }
  Vector2<T>& operator*= (T rh) {
    x *= rh;
    y *= rh;
    return *this;
  }
  Vector2<T>& operator/= (const Vector2<T>& rh) {
    assert(rh.x != 0 && "rh.x operator/= cannot divide by 0");
    assert(rh.y != 0 && "rh.y operator/= cannot divide by 0");

    x /= rh.x;
    y /= rh.y;
    return *this;
  }
  Vector2<T>& operator/= (T rh) {
    assert(rh != 0 && "rh operator/= cannot divide by 0");
    x /= rh;
    y /= rh;
    return *this;
  }

  Vector2<T>& limit(T max) {
    T size = magnitude();
    if (size > max) {
      x *= max / size;
      y *= max / size;
    }
    return *this;
  }

  Vector2<T>& normalize() {
    T m = magnitude();

    if (m > 0) {
      x /= m;
      y /= m;
    }
    return *this;
  }

  T distance(const Vector2<T>& v) const {
    T dx = x - v.x;
    T dy = y - v.y;
    return std::sqrt(dx*dx + dy*dy);
  }

  T dot(const Vector2<T>& v) const {
    return x * v.x + y * v.y;
  }

  T magnitude() const { return std::sqrt(x*x + y*y); }

};

template<typename T>
Vector2<T> operator-(const Vector2<T>& rh) {
  return Vector2<T>(-rh.x, -rh.y);
}

template<typename T>
Vector2<T> operator+ (const Vector2<T>& lh, const Vector2<T>& rh) {
  return Vector2<T>{lh.x + rh.x, lh.y + rh.y};
}

template<typename T>
Vector2<T> operator- (const Vector2<T>& lh, const Vector2<T>& rh) {
  return Vector2<T>{lh.x - rh.x, lh.y - rh.y};
}

using Vector2f = Vector2<float>;

} // namespace boids


#endif // _BOIDS_VECTOR_H_
