/* DZco - digital control library
 * Copyright (C) 2000 Tomomichi Sugihara (Zhidao)
 *
 * dz_sys_pid - PID controller
 */

#ifndef __DZ_SYS_PID_H__
#define __DZ_SYS_PID_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

/* ********************************************************** */
/* amplifier
 * ********************************************************** */

/* value map: [gain] */

/*! \brief create proportional amplifier.
 *
 * 'dzSysCreateP()' creates a proportional amplifier 'c'.
 * 'gain' is the proportional gain.
 *
 * \retval returns the false value if it fails
 * to allocate the internal working memory. Otherwise,
 * the true value is returned.
 */
__EXPORT bool dzSysCreateP(dzSys *sys, double gain);

__EXPORT void dzSysPSetGain(dzSys *sys, double gain);

extern dzSysMethod dz_sys_p_met;

/* ********************************************************** */
/* integrator
 * ********************************************************** */

/* value map: [prev][gain][fgt] */

/*! \brief create an integrator.
 *
 * 'dzSysCreateI()' creates an integrator 'c'.
 * 'dt' is the sampling time for descrete integration.
 * 'gain' is the integral gain.
 *
 * 'dzSysCreateI()' returns the false value if 'dt'
 * is an invalid value - too short or negative.
 * Also, the false value is returned when it fails to
 * allocate the internal working memory.
 * Otherwise, the true value is returned.
 */
__EXPORT bool dzSysCreateI(dzSys *sys, double gain, double fgt);

__EXPORT void dzSysISetGain(dzSys *sys, double gain);
__EXPORT void dzSysISetFgt(dzSys *sys, double fgt);

extern dzSysMethod dz_sys_i_met;

/* ********************************************************** */
/* differentiator
 * ********************************************************** */

/* value map: [prev][gain][tc] */

/* METHOD:
 * dzSysCreateD - create differentiator.
 *
 * 'dzSysCreateD()' creates a differentiator 'c'.
 * 'dt' is the sampling time for descrete differentiation.
 * 'gain' is the differential gain.
 * 't' is the time constant to filter the signal. A smaller
 * 't' provides fine outputs, while it tends to suffer from
 * signal noise. The feasible, i.e. mathematically valid,
 * smallest amount for 't' is 'dt'. A smaller 't' than 'dt',
 * 0 for example, is automatically replaced for 'dt'.
 * [RETURN VALUE]
 * 'dzSysCreateD()' returns the false value if 'dt'
 * is an invalid value - too short or negative.
 * Also, the false value is returned when it fails to
 * allocate the internal working memory.
 * Otherwise, the true value is returned.
 */
__EXPORT bool dzSysCreateD(dzSys *sys, double gain, double tc);

__EXPORT void dzSysDSetGain(dzSys *sys, double gain);
__EXPORT void dzSysDSetTC(dzSys *sys, double t);

extern dzSysMethod dz_sys_d_met;

/* ********************************************************** */
/* PID(Proportional, Integral and Differential) controller
 * ********************************************************** */

/* value map: [pgain][intg][prev][fgt][igain][dgain][tc] */

/*! \brief create PID controller.
 *
 * 'dzSysCreatePID()' creates a PID controller 'c'.
 * 'dt' is the sampling time for descrete control.
 * 'p' is the proportional gain.
 * 'i' is the integral gain.
 * 'd' is the differential gain.
 *
 * 'dzSysCreatePID()' returns the false value in cases
 * that 'dt' is a too short or negative value, or that
 * it fails to allocate the internal work space.
 * Otherwise, the true value is returned.
 */
__EXPORT bool dzSysCreatePID(dzSys *sys, double kp, double ki, double kd, double tc, double fgt);

__EXPORT void dzSysPIDSetPGain(dzSys *sys, double kp);
__EXPORT void dzSysPIDSetIGain(dzSys *sys, double ki);
__EXPORT void dzSysPIDSetDGain(dzSys *sys, double kd);
__EXPORT void dzSysPIDSetTC(dzSys *sys, double tc);
__EXPORT void dzSysPIDSetFgt(dzSys *sys, double fgt);

extern dzSysMethod dz_sys_pid_met;

/* ********************************************************** */
/* QPD(Quadratic Proportional and Differential) controller
 * ********************************************************** */

/* value map: [kq1=2kp(1-eps)][kq2=(3-2eps)/2(1-eps)][goal][init][dgain][prev] */

/*! \brief create QPD controller.
 *
 * 'dzSysCreateQPD()' creates a QPD controller 'c'.
 * 'dt' is the sampling time for descrete control.
 * 'p' is the quadratic proportional gain.
 * 'd' is the differential gain.
 * [RETURN VALUE]
 * 'dzSysCreateQPD()' returns the false value in cases
 * that 'dt' is a too short or negative value, or that
 * it fails to allocate the internal work space.
 * Otherwise, the true value is returned.
 */
__EXPORT bool dzSysCreateQPD(dzSys *sys, double kp, double kd, double eps);

__EXPORT void dzSysQPDSetGoal(dzSys *sys, double goal);

extern dzSysMethod dz_sys_qpd_met;

__END_DECLS

#endif /* __DZ_SYS_PID_H__ */
