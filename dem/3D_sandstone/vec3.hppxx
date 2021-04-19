#ifndef VEC3_HPP
#define VEC3_HPP

/// @file
/// @brief Template class for vectors with 3 components
/// @author Vincent Richefeu <Vincent.Richefeu@3sr-grenoble.fr>
/// @author Lab 3SR, Grenoble University
/// @date 2009-2010; 2015

#include <cmath>
#include <iostream>

/// @brief Vector with 3 components
template <class T> class vec3 {
public:
  T x, y, z;

  static const vec3 zero;
  static const vec3 unit_x;
  static const vec3 unit_y;
  static const vec3 unit_z;
  static const vec3 one;

  vec3() : x(0), y(0), z(0) {}
  vec3(T X, T Y, T Z) : x(X), y(Y), z(Z) {}
  vec3(const vec3 &v) : x(v.x), y(v.y), z(v.z) {}

  void reset() { x = y = z = 0; }
  void set(T X, T Y, T Z) {
    x = X;
    y = Y;
    z = Z;
  }
  void set(T val) { x = y = z = val; }

  bool isnull(const T tol = 1e-20) const { return (fabs(x) < tol && fabs(y) < tol && fabs(z) < tol); }

  T *c_vec() { return &x; }

  T &operator[](int i) { return (&x)[i]; }
  const T &operator[](int i) const { return (&x)[i]; }

  // For local frames, the notation n,t and s is more appropriate than x,y and z
  const T n() const { return x; }
  const T t() const { return y; }
  const T s() const { return z; }

  // Arithmetic operations
  vec3 &operator+=(const vec3 &a) {
    x += a.x;
    y += a.y;
    z += a.z;
    return *this;
  }
  vec3 &operator-=(const vec3 &a) {
    x -= a.x;
    y -= a.y;
    z -= a.z;
    return *this;
  }
  vec3 &operator*=(T k) {
    x *= k;
    y *= k;
    z *= k;
    return *this;
  }
  vec3 &operator/=(T k) {
    T invk = 1.0 / k;
    x *= invk;
    y *= invk;
    z *= invk;
    return *this;
  }

  friend vec3 operator+(const vec3 &a, const vec3 &b) { return vec3(a.x + b.x, a.y + b.y, a.z + b.z); }
  friend vec3 operator-(const vec3 &a, const vec3 &b) { return vec3(a.x - b.x, a.y - b.y, a.z - b.z); }
  friend vec3 operator-(const vec3 &a) { return vec3(-a.x, -a.y, -a.z); }
  friend vec3 operator*(const vec3 &a, T k) { return vec3(a.x * k, a.y * k, a.z * k); }
  friend vec3 operator*(T k, const vec3 &a) { return vec3(a.x * k, a.y * k, a.z * k); }
  friend vec3 operator/(const vec3 &a, T k) {
    T invk = 1.0 / k;
    return vec3(a.x * invk, a.y * invk, a.z * invk);
  }

  // --- Specific external operations ---

  /// Dot product
  friend T operator*(const vec3 &a, const vec3 &b) { return (a.x * b.x + a.y * b.y + a.z * b.z); }

  /// Multiply each component one another
  friend vec3<T> component_product(const vec3<T> &a, const vec3<T> &b) {
    return vec3<T>(a.x * b.x, a.y * b.y, a.z * b.z);
  }

  /// Find the smallest components
  friend vec3<T> component_min(const vec3<T> &a, const vec3<T> &b) {
    return vec3<T>((a.x < b.x) ? a.x : b.x, (a.y < b.y) ? a.y : b.y, (a.z < b.z) ? a.z : b.z);
  }

  /// Find the largest components
  friend vec3<T> component_max(const vec3<T> &a, const vec3<T> &b) {
    return vec3<T>((a.x > b.x) ? a.x : b.x, (a.y > b.y) ? a.y : b.y, (a.z > b.z) ? a.z : b.z);
  }

  /// Absolut value of the components
  friend vec3<T> component_abs(const vec3<T> &a) { return vec3<T>(fabs(a.x), fabs(a.y), fabs(a.z)); }

  /// Cross product
  friend vec3<T> cross(const vec3<T> &a, const vec3<T> &b) {
    return vec3<T>(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
  }

  /// Squared length of the vector
  friend T norm2(const vec3 &a) { return a * a; }

  /// Length of the vector
  friend T norm(const vec3 &a) { return sqrt(a * a); }

  T length() const { return norm(*this); }

  /// Normalize and return length (before being normalized)
  T normalize() {
    T n = norm2(*this);
    if (n > 0.0) {
      n = sqrt(n);
      *this *= (1.0 / n);
    }

    return n;
  }

  /// Normalize and return the normalized vector
  vec3 normalized() {
    this->normalize();
    return *this;
  }

  // input/output
  friend std::ostream &operator<<(std::ostream &pStr, const vec3 &pV) {
    return (pStr << pV.x << ' ' << pV.y << ' ' << pV.z);
  }

  friend std::istream &operator>>(std::istream &pStr, vec3 &pV) { return (pStr >> pV.x >> pV.y >> pV.z); }
};

typedef vec3<double> vec3r;
typedef vec3<int> vec3i;
typedef vec3<size_t> vec3ui;
typedef vec3<bool> vec3b;

namespace std {
template <class T> struct less<vec3<T>> {
  bool operator()(const vec3<T> &lhs, const vec3<T> &rhs) const {
    if (rhs.x < lhs.x)
      return true;
    else if (lhs.x == rhs.x && rhs.y < lhs.y)
      return true;
    else if (lhs.x == rhs.x && lhs.y == rhs.y && rhs.z < lhs.z)
      return true;
    return false;
  }
};
} // namespace std

#endif /* end of include guard: VEC3_HPP */
