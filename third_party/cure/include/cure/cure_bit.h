/* CURE - Commonly-Used Routine Elements
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 */
/*! \file cure_bit.h
 * \brief bit operations.
 * \author Zhidao
 */

#ifndef __CURE_BIT_H__
#define __CURE_BIT_H__

#include <cure/cure_misc.h>

__BEGIN_DECLS

/* ********************************************************** */
/*! \defgroup bit_op bit operations.
 * \{ *//* ************************************************** */

/*! \brief convert 16-bit little/big endian to big/little endian.
 *
 * endian_reverse16() converts a little endian 16-bit value
 * to a bit endian value, and vice versa.
 * \return a value converted.
 * \note
 * PDP endian is not dealt with.
 */
__EXPORT uword endian_reverse16(uword val);

/*! \brief convert 32-bit little/big endian to big/little endian.
 *
 * endian_reverse32() converts a little endian 32-bit value
 * to a bit endian value, and vice versa.
 * \return a value converted.
 * \note
 * PDP endian is not dealt with.
 */
__EXPORT ulong endian_reverse32(ulong val);

/*! \brief convert 64-bit little/big endian to big/little endian.
 *
 * endian_reverse64() converts a little endian 64-bit value
 * to a bit endian value, and vice versa.
 * \return a value converted.
 * \note
 * PDP endian is not dealt with.
 */
__EXPORT unsigned long long endian_reverse64(unsigned long long val);

/*! \brief rotate a bit sequence.
 *
 * bit_rotate() rotates a bit sequence of a value \a val
 * with the bit width \a width.
 * for bit width. \a d is a rotation displacement.
 * \return the value converted.
 */
__EXPORT ulong bit_rotate(ulong val, int width, int d);

/*! \brief reverse a bit sequence.
 *
 * bit_reverse() reverses a bit sequence \a val with
 * the bit width \a width.
 * \return the value converted.
 */
__EXPORT ulong bit_reverse(ulong val, int width);

/*! \brief convert a bit sequence to a string in binary format.
 *
 * sprintf_bit() converts a given value \a val to
 * a string in binary format. The string is stored
 * where a pointer \a str points.
 * \a width is the applied bit width.
 * For example, sprintf_bit( str, 0xf, 6 ) makes
 * \a str "001111".
 * \sa fprintf_bit
 */
__EXPORT void sprintf_bit(char *str, ulong val, int width);

#ifndef __KERNEL__
/*! \brief output a bit sequence to a file in binary format.
 *
 * fprintf_bit() outputs a given value \a val to the
 * current position of a file pointed by \a fp.
 * The output format is the same with that of sprintf_bit().
 * \note
 * fprintf_bit() is not available in the kernel space.
 */
__EXPORT void fprintf_bit(FILE *fp, ulong val, int width);
/*! \brief output a bit sequence to the standard output. */
#define printf_bit(v,w) fprintf_bit( stdout, (v), (w) )
#else
void printk_bit(ulong val, int width);
#endif /* __KERNEL__ */

/*! \} */

__END_DECLS

#endif /* __CURE_BIT_H__ */
