/* LIW - Linux Wizard
 * Copyright (C) 2000 Tomomichi Sugihara (Zhidao)
 */
/*! \file liw_time.h
 * \brief time management.
 * \author Zhidao
 */

#ifndef __LIW_TIME_H__
#define __LIW_TIME_H__

#include <sys/time.h>
#include <cure/cure.h>

__BEGIN_DECLS

/* time precision depends on macro HZ(defined in asm/param.h) */

#define sec2msec(s)   ( (s) * 1000LL )
#define sec2usec(s)   ( (s) * 1000000LL )
#define sec2nsec(s)   ( (s) * 1000000000LL )
#define msec2sec(s)   ( (s) * 0.001 )
#define usec2sec(s)   ( (s) * 0.000001 )
#define nsec2sec(s)   ( (s) * 0.000000001 )

#define tv_usec( tv ) ( sec2usec((tv)->tv_sec) + (tv)->tv_usec )
#define tv_sec( tv )  ( (tv)->tv_sec + usec2sec((tv)->tv_usec) )

#define ts_nsec( ts ) ( sec2nsec((ts)->tv_sec) + (ts)->tv_nsec )
#define ts_sec( ts )  ( (ts)->tv_sec + nsec2sec((ts)->tv_nsec) )

/* fine sleep functions */

int liwSleep(long int sec, long int nsec);

/* CPU clock using RDTSC */
#if ((__GNUC__ < 4) || (__GNUC__ == 4)) && !defined(__clang__)
extern inline long long liwClock(void){
#else
inline long long liwClock(void){
#endif
  unsigned long long ret;
  __asm__ volatile ( "rdtsc" : "=A" (ret) );
  return ret;
}

int liwCPUFreq(void);
double liwCPURate(void);

/* get the current time stamp */
long long liwNow(void);
double liwNowF(void);

__END_DECLS

#endif /* __LIW_TIME_H__ */
