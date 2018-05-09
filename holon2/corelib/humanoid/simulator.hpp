/* simulator - Base class of simulator
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

#ifndef HOLON_HUMANOID_SIMULATOR_HPP_
#define HOLON_HUMANOID_SIMULATOR_HPP_

namespace holon {

class Simulator {
  static constexpr double default_time_step = 0.001;

 public:
  Simulator() : m_time(0.0), m_time_step(default_time_step) {}
  virtual ~Simulator() = default;

  // accessors
  double time() const { return m_time; }
  double time_step() const { return m_time_step; }

  double getTime() const { return time(); }
  double getTimeStep() const { return time_step(); }

  // mutators
  Simulator& setTimeStep(double t_time_step) {
    m_time_step = t_time_step;
    return *this;
  }

  // reset
  virtual Simulator& reset() = 0;

  // update
  virtual bool update() = 0;
  virtual bool update(double t_time_step) = 0;

 protected:
  void resetTime() { m_time = 0.0; }
  void updateTime(double t_time_step) { m_time += t_time_step; }

 private:
  double m_time;
  double m_time_step;
};

}  // namespace holon

#endif  // HOLON_HUMANOID_SIMULATOR_HPP_
