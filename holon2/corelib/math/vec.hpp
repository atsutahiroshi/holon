/* vec - Vector class
 *
 * Copyright (c) 2018 Hiroshi Atsuta <atsuta.hiroshi@gmail.com>
 *
 * This file is part of holon.
 *
 * Holon is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Holon is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with holon.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef HOLON_MATH_VEC_HPP_
#define HOLON_MATH_VEC_HPP_

#include <limits>
#include <sstream>
#include <string>
#include "holon2/corelib/common/random.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#include "third_party/eigen/Eigen/Dense"
#pragma GCC diagnostic pop
#include "third_party/catch/catch.hpp"

namespace holon {

using Vec = Eigen::VectorXd;
using Vec3d = Eigen::Vector3d;

extern const Vec3d kVec3dZero;
extern const Vec3d kVec3dX;
extern const Vec3d kVec3dY;
extern const Vec3d kVec3dZ;

// ref: https://stackoverflow.com/a/15052131
template <typename DerivedA, typename DerivedB>
bool allclose(
    const Eigen::DenseBase<DerivedA>& a, const Eigen::DenseBase<DerivedB>& b,
    const typename DerivedA::RealScalar& rtol =
        Eigen::NumTraits<typename DerivedA::RealScalar>::dummy_precision(),
    const typename DerivedA::RealScalar& atol =
        Eigen::NumTraits<typename DerivedA::RealScalar>::epsilon()) {
  return ((a.derived() - b.derived()).array().abs() <=
          (atol + rtol * b.derived().array().abs()))
      .all();
}

// for random number generator
template <typename Engine, typename Distribution>
struct RandomAdapter<Vec3d, Engine, Distribution> {
  using type = Vec3d;
  type get(Engine& eng, Distribution& dist) {
    return Vec3d(dist(eng), dist(eng), dist(eng));
  }
};

}  // namespace holon

namespace Catch {
namespace Matchers {
namespace Vec {

using holon::Vec3d;

class ApproxEqualsMatcher : public MatcherBase<Vec3d> {
 public:
  explicit ApproxEqualsMatcher(const Vec3d& comparator, const double tol)
      : m_comparator(comparator), m_tol(tol) {}
  bool match(const Vec3d& v) const override {
    return ::holon::allclose(
        m_comparator, v, Eigen::NumTraits<double>::dummy_precision(), m_tol);
  }
  std::string describe() const override {
    std::ostringstream ss;
    ss << m_comparator;
    return "\nApprox. equals:\n" + ss.str();
  }

 private:
  const Vec3d& m_comparator;
  const double m_tol;
};

}  // namespace Vec

inline Vec::ApproxEqualsMatcher ApproxEquals(
    const holon::Vec3d& comparator,
    const double tol = std::numeric_limits<double>::epsilon()) {
  return Vec::ApproxEqualsMatcher(comparator, tol);
}

}  // namespace Matchers
}  // namespace Catch

#endif  // HOLON_MATH_VEC_HPP_
