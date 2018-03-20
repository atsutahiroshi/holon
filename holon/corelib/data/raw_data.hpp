/* raw_data - Raw data class
 *
 * Copyright (c) 2018 Hiroshi Atsuta <atsuta.hiroshi@gmail.com>
 *
 * This file is part of the holon.
 *
 * The holon is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The holon is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the holon.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HOLON_DATA_RAW_DATA_HPP_
#define HOLON_DATA_RAW_DATA_HPP_

#include <memory>
#include "holon/corelib/math/vec3d.hpp"

namespace holon {

struct ComZmpModelRawData {
  static const double default_mass;
  static const Vec3D default_com_position;

  double mass;
  Vec3D com_position;
  Vec3D com_velocity;
  Vec3D com_acceleration;
  Vec3D zmp_position;
  Vec3D react_force;
  Vec3D ext_froce;
  Vec3D total_force;

  ComZmpModelRawData()
      : mass(default_mass),
        com_position(default_com_position),
        react_force({0, 0, mass * 9.8}) {}
  ComZmpModelRawData(const Vec3D& t_initial_com_position)
      : mass(default_mass),
        com_position(t_initial_com_position),
        react_force({0, 0, mass * 9.8}) {}
  ComZmpModelRawData(const Vec3D& t_initial_com_position, double t_mass)
      : mass(t_mass),
        com_position(t_initial_com_position),
        react_force({0, 0, mass * 9.8}) {}
};
const double ComZmpModelRawData::default_mass = 1.0;
const Vec3D ComZmpModelRawData::default_com_position = {0, 0, 1};

struct PointMassModelRawData {
  static const double default_mass;

  double mass;
  Vec3D position;
  Vec3D velocity;
  Vec3D acceleration;
  Vec3D force;

  PointMassModelRawData() : mass(default_mass) {}
  PointMassModelRawData(const Vec3D& t_initial_position)
      : mass(default_mass), position(t_initial_position) {}
  PointMassModelRawData(const Vec3D& t_initial_position, double t_mass)
      : mass(t_mass), position(t_initial_position) {}
};

const double PointMassModelRawData::default_mass = 1.0;

template <typename T, typename... Args>
std::shared_ptr<T> alloc_data(Args... args) {
  return std::make_shared<T>(args...);
}

template <typename Data>
class SharedData {
  using Self = SharedData<Data>;
  using DataPtr = std::shared_ptr<Data>;

 public:
  SharedData() : m_ptr(alloc_data<Data>()) {}
  SharedData(DataPtr t_ptr) : m_ptr(t_ptr) {}
  virtual ~SharedData() = default;
  // Deleted copy constructor / assignment operator
  SharedData(const Self&) = delete;
  Self& operator=(const Self&) = delete;
  // Defaulted move constructor / assignment operator
  SharedData(Self&&) = default;
  Self& operator=(Self&&) = default;

  Data& get() { return *m_ptr; }
  const Data& get() const { return *m_ptr; }
  DataPtr ptr() { return m_ptr; }
  const DataPtr ptr() const { return m_ptr; }
  void copy(const Self& src) { *m_ptr = src.get(); }
  void share(const Self& target) { m_ptr = target.ptr(); }

  Self clone() const {
    Self tmp;
    tmp.copy(*this);
    return tmp;
  }

 private:
  DataPtr m_ptr;
};

class ComZmpModelData {
  using Self = ComZmpModelData;
  using RawData = ComZmpModelRawData;

 public:
  ComZmpModelData() : m_data(alloc_data<RawData>()) {}
  ComZmpModelData(const Vec3D& t_initial_com_position)
      : m_data(alloc_data<RawData>(t_initial_com_position)) {}
  ComZmpModelData(const Vec3D& t_initial_com_position, double t_mass)
      : m_data(alloc_data<RawData>(t_initial_com_position, t_mass)) {}
  RawData& operator()() { return m_data.get(); }

 private:
  SharedData<RawData> m_data;
};

class PointMassModelData {
 public:
  PointMassModelData() : m_data(alloc_data<PointMassModelRawData>()) {}
  PointMassModelData(const Vec3D& t_initial_position)
      : m_data(alloc_data<PointMassModelRawData>(t_initial_position)) {}
  PointMassModelData(const Vec3D& t_initial_position, double t_mass)
      : m_data(alloc_data<PointMassModelRawData>(t_initial_position, t_mass)) {}
  PointMassModelRawData& operator()() { return m_data.get(); }

 private:
  SharedData<PointMassModelRawData> m_data;
};

struct BipedModelRawData {
  ComZmpModelRawData com_data;
  PointMassModelRawData lf_data;
  PointMassModelRawData rf_data;

  BipedModelRawData() : com_data(), lf_data(), rf_data() {}
  BipedModelRawData(const Vec3D& t_initial_com_position, double t_mass,
                    double t_foot_dist)
      : com_data(t_initial_com_position, t_mass), lf_data(), rf_data() {}
  BipedModelRawData(const Vec3D& t_initial_com_position, double t_mass,
                    const Vec3D& t_initial_lf_position,
                    const Vec3D& t_initial_rf_position)
      : com_data(t_initial_com_position, t_mass),
        lf_data(t_initial_lf_position),
        rf_data(t_initial_rf_position) {}

  ComZmpModelRawData& com() { return com_data; }
  PointMassModelRawData& lf() { return lf_data; }
  PointMassModelRawData& rf() { return rf_data; }
};

class BipedModelData2 {
 public:
  BipedModelData2() : m_data(alloc_data<BipedModelRawData>()) {}
  BipedModelData2(const Vec3D& t_initial_com_position, double t_mass,
                  double t_foot_dist)
      : m_data(alloc_data<BipedModelRawData>(t_initial_com_position, t_mass,
                                             t_foot_dist)) {}
  BipedModelData2(const Vec3D& t_initial_com_position, double t_mass,
                  const Vec3D& t_initial_lf_position,
                  const Vec3D& t_initial_rf_position)
      : m_data(alloc_data<BipedModelRawData>(t_initial_com_position, t_mass,
                                             t_initial_lf_position,
                                             t_initial_rf_position)) {}
  BipedModelRawData& operator()() { return m_data.get(); }

 private:
  SharedData<BipedModelRawData> m_data;
};

class BipedModelData {
 public:
  BipedModelData()
      : m_com_data(alloc_data<ComZmpModelRawData>()),
        m_lf_data(alloc_data<PointMassModelRawData>()),
        m_rf_data(alloc_data<PointMassModelRawData>()) {}

  BipedModelData(const Vec3D& t_initial_com_position, double t_mass,
                 double t_foot_dist)
      : m_com_data(
            alloc_data<ComZmpModelRawData>(t_initial_com_position, t_mass)),
        m_lf_data(alloc_data<PointMassModelRawData>()),
        m_rf_data(alloc_data<PointMassModelRawData>()) {}

  BipedModelData(const Vec3D& t_initial_com_position, double t_mass,
                 const Vec3D& t_initial_lf_position,
                 const Vec3D& t_initial_rf_position)
      : m_com_data(
            alloc_data<ComZmpModelRawData>(t_initial_com_position, t_mass)),
        m_lf_data(alloc_data<PointMassModelRawData>(t_initial_lf_position)),
        m_rf_data(alloc_data<PointMassModelRawData>(t_initial_rf_position)) {}

  BipedModelData& operator()() { return *this; }
  ComZmpModelRawData& com() { return m_com_data.get(); }
  PointMassModelRawData& lf() { return m_lf_data.get(); }
  PointMassModelRawData& rf() { return m_rf_data.get(); }

  BipedModelData clone() {
    BipedModelData tmp;
    tmp.com() = m_com_data.get();
    tmp.lf() = m_lf_data.get();
    tmp.rf() = m_rf_data.get();
    return tmp;
  }

 private:
  SharedData<ComZmpModelRawData> m_com_data;
  SharedData<PointMassModelRawData> m_lf_data;
  SharedData<PointMassModelRawData> m_rf_data;
};

}  // namespace holon

#endif  // HOLON_DATA_RAW_DATA_HPP_
