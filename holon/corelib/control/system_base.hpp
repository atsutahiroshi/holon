/* system_base - Base class for System
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

#ifndef HOLON_CONTROL_SYSTEM_BASE_HPP_
#define HOLON_CONTROL_SYSTEM_BASE_HPP_

#include <array>
#include <functional>
#include <memory>
#include <utility>

namespace holon {

template <template <typename, typename> class System, typename State,
          typename Data>
System<State, Data> make_system(Data data) {
  return System<State, Data>(data);
}

template <template <typename, typename> class System, typename State,
          template <typename> class Data>
System<State, Data<State>> make_system(Data<State> data) {
  return System<State, Data<State>>(data);
}

template <typename State, typename Data>
class SystemBase {
 protected:
  using Self = SystemBase<State, Data>;

 public:
  using StateArray = std::array<typename std::decay<State>::type, 2>;
  using Function = std::function<StateArray(const StateArray&, const double)>;

 public:
  SystemBase() = delete;
  SystemBase(Data t_data) : m_data(t_data) {}
  virtual ~SystemBase() = default;

  // virtual function
  virtual StateArray operator()(const StateArray& state,
                                const double t) const = 0;

  // accessors
  const Data& data() const noexcept { return m_data; }

  // mutators
  Self& set_data(Data t_data) {
    m_data = t_data;
    return *this;
  }

 private:
  Data m_data;
};

}  // namespace holon

#endif  // HOLON_CONTROL_SYSTEM_BASE_HPP_
