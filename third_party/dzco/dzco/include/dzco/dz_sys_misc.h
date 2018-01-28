/* DZco - digital control library
 * Copyright (C) 2000 Tomomichi Sugihara (Zhidao)
 *
 * dz_sys_misc - miscellanies
 */

#ifndef __DZ_SYS_MISC_H__
#define __DZ_SYS_MISC_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

/* ********************************************************** */
/* value confluenter
 * ********************************************************** */

/*! \brief create and connect inputs of confluenter.
 *
 * 'dzSysCreateAdder()' and 'dzSysCreateSubtr()'
 * creates an adder and a subtractor, respectively.
 * The system created is stored into 'c'.
 * [RETURN VALUE]
 * 'dzSysCreateAdder()' and 'dzSysCreateSubtr()'
 * return the true value without exception.
 */
__EXPORT bool dzSysCreateAdder(dzSys *c, int n);

extern dzSysMethod dz_sys_adder_met;

__EXPORT bool dzSysCreateSubtr(dzSys *c, int n);

extern dzSysMethod dz_sys_subtr_met;

/* ********************************************************** */
/* saturater
 * ********************************************************** */

/*! \brief create saturater.
 *
 * 'dzSysCreateLimit()' creates a saturater 'c'.
 * 'min' and 'max' are the minimum and maximum border,
 * respectively. The output of 'c' is saturated within
 * the range from 'min' to 'max'.
 * [RETURN VALUE]
 * 'dzSysCreateLimit()' returns the false value if
 * failing the internal work space. Otherwise, the
 * true value is returned.
 * [NOTES]
 * When 'max' is less than 'min', the border is
 * automatically corrected, swapping the two values.
 */
__EXPORT bool dzSysCreateLimit(dzSys *c, double max, double min);

extern dzSysMethod dz_sys_limit_met;

__END_DECLS

#endif /* __DZ_SYS_MISC_H__ */
