/* Zeo - Z/Geometry and optics computation library.
 * Copyright (C) 2005 Tomomichi Sugihara (Zhidao)
 *
 * zeo_vec6d - 6D spatial vector.
 */

#ifndef __ZEO_VEC6D_H__
#define __ZEO_VEC6D_H__

#include <zeo/zeo_vec3d.h>

__BEGIN_DECLS

/* ********************************************************** */
/* CLASS: zVec6D
 * spatial 6D vector class = a pair of linear/angular vectors
 * ********************************************************** */

typedef union {
  double e[6];
  zVec3D v[2];
} zVec6D;

/* for backward compatibility */
#define zVec6DElem(u,i)      ( (u)->e[i] )
#define zVec6DSetElem(u,i,e) ( zVec6DElem(u,i) = (e) )

#define zVec6DLin(u)         ( &(u)->v[0] )
#define zVec6DAng(u)         ( &(u)->v[1] )
#define zVec6DSetLin(u,l)    zVec3DCopy( l, zVec6DLin(u) )
#define zVec6DSetAng(u,r)    zVec3DCopy( r, zVec6DAng(u) )

/*! \brief 6D zero vector and unit vectors */
extern const zVec6D zvec6Dzero;
extern const zVec6D zvec6Dlinx;
extern const zVec6D zvec6Dliny;
extern const zVec6D zvec6Dlinz;
extern const zVec6D zvec6Dangx;
extern const zVec6D zvec6Dangy;
extern const zVec6D zvec6Dangz;
#define ZVEC6DZERO ( (zVec6D *)&zvec6Dzero )
#define ZVEC6DLINX ( (zVec6D *)&zvec6Dlinx )
#define ZVEC6DLINY ( (zVec6D *)&zvec6Dliny )
#define ZVEC6DLINZ ( (zVec6D *)&zvec6Dlinz )
#define ZVEC6DANGX ( (zVec6D *)&zvec6Dangx )
#define ZVEC6DANGY ( (zVec6D *)&zvec6Dangy )
#define ZVEC6DANGZ ( (zVec6D *)&zvec6Dangz )

/*! \brief create, copy and cleanup a 6D vector.
 *
 * zVec6DCreate() creates a 6D vector \a v which consists of \a x, \a y,
 * \a z, \a xa, \a ya and \a za.
 *
 * zVec3DToVec6D() creates a 6D vector \a v from two 3D vectors \a vlin
 * and \a vang.
 *
 * zVec6DCopy() copies a 6D vector \a src to the other \a dest.
 *
 * zVec6DClear() sets all the components of a 6D vector \a v for zero.
 * \return
 * zVec6DCreate(), zVec3DToVec6D() and zVec6DClear() return a pointer \a v.
 *
 * zVec6DCopy() returns a pointer \a dest.
 */
__EXPORT zVec6D *zVec6DCreate(zVec6D *v, double x, double y, double z, double xa, double ya, double za);
__EXPORT zVec6D *zVec3DToVec6D(zVec6D *v, zVec3D *v1, zVec3D *v2);
#define zVec6DCopy(s,d) zCopy( zVec6D, s, d )
#define zVec6DClear(v)  zVec6DCopy( ZVEC6DZERO, v )

/*! \brief check if the two 6D vectors are equal.
 *
 * zVec6DMatch() and zVec6DEqual() check if two 6D vectors \a v1 and \a v2
 * are equal. They return a boolean value as a result.
 *
 * zVec6DMatch() strictly compares two vectors \a v1 and \a v2 while
 * zVec6DEqual() checks if the error between \a v1 and \a v2 are
 * sufficiently small.
 * \return
 * zVec6DMatch() and zVec6DEqual() return the true value if \a v1 and \a v2
 * are equal. Otherwise, the false value is returned.
 */
__EXPORT bool zVec6DMatch(zVec6D *v1, zVec6D *v2);
__EXPORT bool zVec6DEqual(zVec6D *v1, zVec6D *v2);

/*! \brief check if a 6D vector is tiny.
 *
 * zVec6DIsTol() checks if the absolute values of every components of
 * a 6D vector \a v are smaller than \a tol.
 *
 * zVec6DIsTiny() applies zTOL (defined in "zeo_misc.h") to the tolerance
 * of zVec6DIsTol().
 * \return
 * zVec6DIsTol() and zVec6DIsTiny() return the true value when the absolute
 * values of every components of \a v are smaller than \a tol and zTOL,
 * respectively. Otherwise, the false value is returned.
 * \notes
 * \a tol must be positive.
 * \sa
 * zIsTol, zIsTiny
 */
__EXPORT bool zVec6DIsTol(zVec6D *v, double tol);
#define zVec6DIsTiny(v) zVec6DIsTol( v, zTOL )

/* ********************************************************** */
/* arithmetics
 * ********************************************************** */

/*! \brief the four rules of the arithmetics for 6D vector.
 *
 * zVec6DAdd() adds two 6D vectors \a v1 and \a v2 and puts it into \a v.
 *
 * zVec6DSub() subtracts a 6D vector \a v2 from the other \a v1 and puts
 * it into \a v.
 *
 * zVec6DRev() reverses a 6D vector \a v and puts it into \a rv.
 *
 * zVec6DMul() multiplies a 6D vector \a v by a scalar value \a k and
 * puts it into \a mv.
 *
 * zVec6DDiv() divides a 6D vector \a v by a scalar value \a k and puts
 * it into \a dv.
 *
 * zVec6DCat() multiplies a 6D vector \a v2 by a scalar value \a k,
 * concatenates it to the other \a v1 and puts it into \a v.
 *
 * zVec6DAddDRC() directly adds \a v2 to \a v1.
 *
 * zVec6DSubDRC() directly subtracts \a v2 from \a v1.
 *
 * zVec6DRevDRC() directly reverses \a v.
 *
 * zVec6DMulDRC() directly multiplies \a v by \a k.
 *
 * zVec6DDivDRC() directly divides \a v by \a k.
 *
 * zVec6DCat() directly concatenates \a v2 multiplied \a v2 by \a k to \a v1.
 * \return
 * Each function returns a pointer to the result vector.
 *
 * zVec6DDiv() and zVec6DDivDRC() return the null pointer if \a k is zero.
 */
__EXPORT zVec6D *zVec6DAdd(zVec6D *v1, zVec6D *v2, zVec6D *v);
__EXPORT zVec6D *zVec6DSub(zVec6D *v1, zVec6D *v2, zVec6D *v);
__EXPORT zVec6D *zVec6DRev(zVec6D *v, zVec6D *rv);
__EXPORT zVec6D *zVec6DMul(zVec6D *v, double k, zVec6D *mv);
__EXPORT zVec6D *zVec6DDiv(zVec6D *v, double k, zVec6D *dv);
__EXPORT zVec6D *zVec6DCat(zVec6D *v1, double k, zVec6D *v2, zVec6D *v);

#define zVec6DAddDRC(v1,v2)   zVec6DAdd(v1,v2,v1)
#define zVec6DSubDRC(v1,v2)   zVec6DSub(v1,v2,v1)
#define zVec6DRevDRC(v)       zVec6DRev(v,v)
#define zVec6DMulDRC(v,k)     zVec6DMul(v,k,v)
#define zVec6DDivDRC(v,k)     zVec6DDiv(v,k,v)
#define zVec6DCatDRC(v1,k,v2) zVec6DCat(v1,k,v2,v1)

/*! \brief inner product of two 6D vectors.
 *
 * zVec6DInnerProd() returns the inner product of two 6D vectors
 * \a v1 and \a v2.
 * \retval \a v1 ^T \a v2
 */
__EXPORT double zVec6DInnerProd(zVec6D *v1, zVec6D *v2);

/*! \brief shift a 6D vector in 3D space.
 *
 * zVec6DLinShift() shifts the velocity type of a 6D vector \a src at
 * the original point to the equivalent 6D vector \a dest at the point
 * \a pos, namely:
 *  v1 = v0 + w0 x \a pos
 *  w1 = w0
 * where \a src=[v0 w0]^T and \a dest=[v1 w1]^T.
 *
 * zVec6DLinShiftDRC() directly shifts the velocity type of a 6D vector
 * \a vec at the original point to the equivalent 6D vector at the point
 * \a pos.
 *
 * zVec6DAngShift() shifts the force type of a 6D vector \a src which
 * works at \a pos to the equivalent 6D vector \a dest which works at
 * the original point, namely:
 *  f1 = f0
 *  n1 = n0 + \a pos x f0
 * where \a src=[f0 n0]^T and \a dest=[f1 n1]^T.
 *
 * zVec6DAngShiftDRC() directly shifts the force type of a 6D vector \a vec
 * which works at \a pos to the equivalent 6D vector which works at the
 * original point.
 * \return
 * zVec6DLinShift() and zVec6DAngShift() return a pointer \a dest.
 * zVec6DLinShiftDRC() and zVec6DAngShift() return a pointer \a vec.
 */
__EXPORT zVec6D *zVec6DLinShift(zVec6D *src, zVec3D *pos, zVec6D *dest);
__EXPORT zVec6D *zVec6DLinShiftDRC(zVec6D *vec, zVec3D *pos);
__EXPORT zVec6D *zVec6DAngShift(zVec6D *src, zVec3D *pos, zVec6D *dest);
__EXPORT zVec6D *zVec6DAngShiftDRC(zVec6D *vec, zVec3D *pos);

/* ********************************************************** */
/* differential kinematics
 * ********************************************************** */

__EXPORT zVec6D *zVec6DDif(zVec6D *v, zVec6D *vnew, double dt, zVec6D *vel);

/* ********************************************************** */
/* I/O
 * ********************************************************** */

/*! \brief input and output of a 6D vector.
 *
 * zVec6DFRead() reads six values from the current position of a file
 * \a fp and creates a 6D vector \a v from them.
 * zVec6DRead() reads six values from the standard input.
 *
 * zVec6DFWrite() outputs a 6D vector \a v to the current position of
 * a file \a fp in the following format:
 *  ( x, y, z, xa, ya, za )
 * When the null pointer is given, it outputs the following string.
 *  (null 6D vector)
 * zVec6DWrite() outputs \a v to the standard output.
 *
 * zVec6DDataFWrite() outputs a 6D vector \a v to the current position
 * of a file \a fp in the following format:
 *  x y z xa ya za
 * When the null pointer is given, it outputs nothing.
 * zVec6DDataWrite() outputs \a v to the standard output in the same
 * format with zVec6DDataFWrite().
 * \return
 * zVec6DFRead() and zVec6DRead() return a pointer \a v.
 *
 * zVec6DFWrite(), zVec6DWrite(), zVec6DDataFWrite() and zVec6DDataWrite()
 * return no value.
 */
__EXPORT zVec6D *zVec6DFRead(FILE *fp, zVec6D *v);
#define zVec6DRead(v) zVec6DFRead( stdin, (v) )
__EXPORT zVec6D *zVec6DDataFWrite(FILE *fp, zVec6D *v);
#define zVec6DDataWrite(v) zVec6DDataFWrite( stdout, (v) )
__EXPORT zVec6D *zVec6DDataNLFWrite(FILE *fp, zVec6D *v);
#define zVec6DDataNLWrite(v) zVec6DDataNLFWrite( stdout, (v) )
__EXPORT zVec6D *zVec6DFWrite(FILE *fp, zVec6D *v);
#define zVec6DWrite(v) zVec6DFWrite( stdout, (v) )

/* METHOD:
 * zVec6DFWriteXML - xml output.
 * ... still testing.
 */
__EXPORT void zVec6DFWriteXML(FILE *fp, zVec6D *v);

__END_DECLS

#endif /* __ZEO_VEC6D_H__ */
