/* macro - Macro header
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

#ifndef HOLON_COMMON_MACRO_HPP_
#define HOLON_COMMON_MACRO_HPP_

#define EXPAND(x) x
#define HOLON_VA_ARGS_N(_1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12,     \
                        _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, \
                        _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, \
                        _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, _45, \
                        _46, _47, _48, _49, _50, _51, _52, _53, _54, _55, _56, \
                        _57, _58, _59, _60, _61, _62, _63, _64, _65, _66, _67, \
                        _68, _69, _70, _71, _72, _73, _74, _75, N, ...)        \
  N
#define HOLON_RSEQ_N()                                                        \
  75, 74, 73, 72, 71, 70, 69, 68, 67, 66, 65, 64, 63, 62, 61, 60, 59, 58, 57, \
      56, 55, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, \
      38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, \
      20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0
#define HOLON_VA_ARGS_ONE_OR_MORE(...)                                         \
  EXPAND(HOLON_VA_ARGS_N(                                                      \
      __VA_ARGS__, MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE, \
      MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE,  \
      MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE,  \
      MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE,  \
      MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE,  \
      MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE, MORE,  \
      MORE, MORE, MORE, MORE, ONE, rest))

#define HOLON_VA_ARGS_NUM(...) \
  EXPAND(HOLON_VA_ARGS_NUM_(__VA_ARGS__, HOLON_RSEQ_N()))
#define HOLON_VA_ARGS_NUM_(...) HOLON_VA_ARGS_N(__VA_ARGS__)

#define HOLON_VA_ARGS_GET_FIRST(...) \
  EXPAND(HOLON_VA_ARGS_GET_FIRST_(__VA_ARGS__, rest))
#define HOLON_VA_ARGS_GET_FIRST_(first, ...) first

#define HOLON_VA_ARGS_GET_REST(...) \
  HOLON_VA_ARGS_GET_REST_(HOLON_VA_ARGS_ONE_OR_MORE(__VA_ARGS__), __VA_ARGS__)
#define HOLON_VA_ARGS_GET_REST_(qty, ...) \
  HOLON_VA_ARGS_GET_REST__(qty, __VA_ARGS__)
#define HOLON_VA_ARGS_GET_REST__(qty, ...) \
  EXPAND(HOLON_VA_ARGS_GET_REST_##qty(__VA_ARGS__))
#define HOLON_VA_ARGS_GET_REST_ONE(first)
#define HOLON_VA_ARGS_GET_REST_MORE(first, ...) , __VA_ARGS__

#endif  // HOLON_COMMON_MACRO_HPP_
