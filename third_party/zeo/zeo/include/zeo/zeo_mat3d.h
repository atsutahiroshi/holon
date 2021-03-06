/* Zeo - Z/Geometry and optics computation library.
 * Copyright (C) 2005 Tomomichi Sugihara (Zhidao)
 *
 * zeo_mat3d - 3x3 matrix.
 */

#ifndef __ZEO_MAT3D_H__
#define __ZEO_MAT3D_H__

#include <zeo/zeo_vec6d.h>

__BEGIN_DECLS

/* ********************************************************** */
/* CLASS: zMat3D
 * 3x3 matrix class
 * ********************************************************** */

typedef union{
  double e[3][3]; /*!< 3x3 matrix */
  zVec3D v[3];    /*!< 3 column vectors */
  double c[9];    /*!< 9 components */
} zMat3D;

/* for backward compatibility */
#define zMat3DElem(m,r,c)      (m)->e[(c)][(r)]
#define zMat3DSetElem(m,r,c,x) ( zMat3DElem(m,r,c) = (x) )
#define zMat3DElem9(m,i)       ( (m)->c[i] )
#define zMat3DSetElem9(m,i,c)  ( zMat3DElem9(m,i) = (c) )
#define zMat3DVec(m,i)         ( &(m)->v[(i)] )
#define zMat3DSetVec(m,i,v)    zVec3DCopy(v,zMat3DVec(m,i))

/*! \brief 3D zero matrix and identity matrix */
extern const zMat3D zmat3Dzero;
extern const zMat3D zmat3Dident;
#define ZMAT3DZERO  ( (zMat3D *)&zmat3Dzero )
#define ZMAT3DIDENT ( (zMat3D *)&zmat3Dident )

/*! \brief create, copy and cleanup a 3x3 matrix.
 *
 * zMat3DCreate() creates a 3x3 matrix from nine values as follows.
 *  | \a a11  \a a12  \a a13 |
 *  | \a a21  \a a22  \a a23 |
 *  | \a a31  \a a32  \a a33 |
 *
 * zMat3DCopy() copies a 3x3 matrix \a src to the other \a dest.
 *
 * zMat3DClear() sets all components of a 3x3 matrix \a m for zero.
 *
 * zMat3DIdent() makes a 3x3 matrix \a m the identity matrix.
 * \return
 * zMat3DCreate(), zMat3DClear() and zMat3DIdent() return a pointer \a m.
 *
 * zMat3DCopy() returns a pointer \a m.
 */
__EXPORT zMat3D *zMat3DCreate(zMat3D *m,
  double a11, double a12, double a13,
  double a21, double a22, double a23,
  double a31, double a32, double a33);
#define zMat3DCopy(s,d) zCopy( zMat3D, s, d )
#define zMat3DClear(m)  zMat3DCopy( ZMAT3DZERO, m )
#define zMat3DIdent(m)  zMat3DCopy( ZMAT3DIDENT, m )

/*! \brief transpose of a 3x3 matrix.
 *
 * zMat3DT() transposes a 3x3 matrix \a m and puts it into \a tm.
 * \return
 * zMat3DT() returns a pointer \a tm.
 * \notes
 * It is not allowed to let \a tm point to the same address with \a m.
 * When \a tm is the same with \a m, anything might happen.
 */
__EXPORT zMat3D *zMat3DT(zMat3D *m, zMat3D *tm);

/*! \brief abstract row/column vectors from a 3x3 matrix.
 *
 * zMat3DRow() abstracts the \a i'th row from a 3x3 matrix \a m and puts
 * it into \a v.
 *
 * zMat3DCol() abstracts the \a i'th column from a 3x3 matrix \a m and
 * puts it into \a v.
 * \return
 * zMat3DRow() and zMat3DCol() return a pointer \a v.
 */
__EXPORT zVec3D *zMat3DRow(zMat3D *m, int i, zVec3D *v);
__EXPORT zVec3D *zMat3DCol(zMat3D *m, int i, zVec3D *v);

/* ********************************************************** */
/* arithmetics
 * ********************************************************** */

/*! \brief the four rules of the arithmetics for 3x3 matrix.
 *
 * zMat3DAdd() adds two 3D matrices \a m1 and \a m2 and puts it into \a m.
 *
 * zMat3DSub() subtracts a 3x3 matrix \a m2 from the other \a m1 and puts
 * it into \a m.
 *
 * zMat3DRev() reverses a 3x3 matrix \a m and puts it into \a rm.
 *
 * zMat3DMul() multiplies a 3x3 matrix \a m by a scalar value \a k and
 * puts it into \a mm.
 *
 * zMat3DDiv() divides a 3x3 matrix \a m by a scalar value \a k and puts
 * it into \a dm.
 *
 * zMat3DCat() multiplies a 3x3 matrix \a m2 by a scalar value \a k,
 * concatenates it to \a m1 and puts it into \a m.
 *
 * zMat3DAddDRC() adds a 3x3 matrix \a m2 directly to the other \a m1.
 *
 * zMat3DSubDRC() subtracts a 3x3 matrix \a m2 directly from the other \a m1.
 *
 * zMat3DRevDRC() reverses a 3x3 matrix \a m directly.
 *
 * zMat3DMulDRC() multiplies a 3x3 matrix \a m directly by a scalar value \a k.
 *
 * zMat3DDivDRC() divides a 3x3 matrix \a m directly by a scalar value \a k.
 *
 * zMat3DCatDRC() multiplies a 3x3 matrix \a m2 by a scalar value \a k and
 * adds it directly to \a m1.
 * \return
 * Each function returns a pointer to the result matrix.
 *
 * zMat3DDiv() and zMat3DDivDRC() return the null pointer if \a k is zero.
 */
__EXPORT zMat3D *zMat3DAdd(zMat3D *m1, zMat3D *m2, zMat3D *m);
__EXPORT zMat3D *zMat3DSub(zMat3D *m1, zMat3D *m2, zMat3D *m);
__EXPORT zMat3D *zMat3DRev(zMat3D *m, zMat3D *rm);
__EXPORT zMat3D *zMat3DMul(zMat3D *m, double k, zMat3D *mm);
__EXPORT zMat3D *zMat3DDiv(zMat3D *m, double k, zMat3D *dm);
__EXPORT zMat3D *zMat3DCat(zMat3D *m1, double k, zMat3D *m2, zMat3D *m);

#define zMat3DAddDRC(m1,m2)   zMat3DAdd(m1,m2,m1)
#define zMat3DSubDRC(m1,m2)   zMat3DSub(m1,m2,m1)
#define zMat3DRevDRC(m)       zMat3DRev(m,m)
#define zMat3DMulDRC(m,k)     zMat3DMul(m,k,m)
#define zMat3DDivDRC(m,k)     zMat3DDiv(m,k,m)
#define zMat3DCatDRC(m1,k,m2) zMat3DCat(m1,k,m2,m1)

/*! \brief dyadic product.
 *
 * zMat3DDyad() calculates a dyadic product of two vectors \a v1 and \a v2
 * and puts it into \a dyad. Namely,
 *   \a dyad = \a v1 \a v2 ^T.
 * \return
 * zMat3DDyad() returns a pointer \a dyad.
 */
__EXPORT zMat3D *zMat3DDyad(zVec3D *v1, zVec3D *v2, zMat3D *dyad);

/*! \brief create a matrix equivalent to the outer product of a 3D vector.
 *
 * zVec3DOuterProdMat3D() computes the outer-product skew-symmetric matrix
 * of a 3D vector \a v and puts it into \a m. Namely, \a m a is equivalent
 * with \a v x a with respect to an arbitrary 3D vector a.
 *
 * zVec3DOuterProd2Mat3D() computes the twice-outer-product matrix of a 3D
 * vector \a v1 and \a v2 and puts it into \a m. Namely, \a m a is equivalent
 * with \a v1 x ( \a v2 x a ) ) with respect to an arbitrary 3D vector a.
 * \return
 * zVec3DOuterProdMat3D() and zVec3DOuterProd2Mat3D() return a pointer \a m.
 */
__EXPORT zMat3D *zVec3DOuterProdMat3D(zVec3D *v, zMat3D *m);
__EXPORT zMat3D *zVec3DOuterProd2Mat3D(zVec3D *v1, zVec3D *v2, zMat3D *m);

/* ********************************************************** */
/* inverse of a 3x3 matrix
 * ********************************************************** */

/*! \brief determinant of a 3x3 matrix.
 *
 * zMat3DDet() computes the determinant of an arbitrary 3x3 matrix \a m.
 * \retval the determinant of \a m
 */
__EXPORT double zMat3DDet(zMat3D *m);

/*! \brief inverse of a 3x3 matrix.
 *
 * zMat3DInv() computes the inverse matrix of an arbitrary 3x3 matrix \a m
 * and puts it into \a im. It does not assume that \a m is an orthonormal
 * matrix.
 * \return
 * zMat3DInv() returns a pointer \a im, if it succeeds. If \a m is singular,
 * it returns the null pointer.
 * \notes
 * \a im has to point to a different address from \a m. If \a im is the
 * same with \a m, anything might happen.
 */
__EXPORT zMat3D *zMat3DInv(zMat3D *m, zMat3D *im);

/* ********************************************************** */
/* multiplication of a 3D vector by a 3x3 matrix
 * ********************************************************** */

/*! \brief multiply a 3D vector and a 3x3 matrix.
 *
 * zMulMatVec3D() multiplies a 3D vector \a v by a 3x3 matrix \a m and puts
 * it into \a mv.
 *
 * zMulMatTVec3D() multiplies a 3D vector \a v by the transpose matrix of
 * a 3x3 matrix \a m and puts it into \a mv.
 *
 * zMulMatVec3DDRC() directly multiplies a 3D vector \a v by a 3x3 matrix \a m.
 *
 * zMulMatTVec3DDRC() directly multiplies a 3D vector v by the transpose
 * of a 3x3 matrix \a m.
 *
 * zMulInvMatVec3D() multiplies a 3D vector \a v by the inverse of a 3x3
 * matrix \a m and puts it into \a miv.
 * \return
 * Each function returns a pointer to the result.
 */
__EXPORT zVec3D *zMulMatVec3D(zMat3D *m, zVec3D *v, zVec3D *mv);
__EXPORT zVec3D *zMulMatTVec3D(zMat3D *m, zVec3D *v, zVec3D *mv);

#define zMulMatVec3DDRC(m,v)  zMulMatVec3D(m,v,v)
#define zMulMatTVec3DDRC(m,v) zMulMatTVec3D(m,v,v)

__EXPORT zVec3D *zMulInvMatVec3D(zMat3D *m, zVec3D *v, zVec3D *miv);

/*! \brief inversely compute the concatenate ratio of a 3D vector.
 *
 * zVec3DCatRatio() calculates the concatenate ratio of a 3D vector \a v
 * for three bases \a v1, \a v2 and \a v3. Namely, an arbitrary vector \a v
 * is represented by a concatenation of three bases vectors \a v1, \a v2
 * and \a v3 as follows.
 *  \a v = \a r1 * \a v1 + \a r2 * \a v2 + \a r3 * \a v3
 * zVec3DCatRatio() computes \a r1, \a r2 and \a r3 in the above equation
 * and puts them into \a ratio.
 * \notes
 * Consequently, the array \a ratio must have three components at least.
 * If not, anything might happen.
 *
 * This function fails if \a v1, \a v2 and \a v3 are not independent with
 * each other.
 * \return
 * zVec3DCatRatio() returns a pointer \a ratio.
 */
__EXPORT double *zVec3DCatRatio(zVec3D *v1, zVec3D *v2, zVec3D *v3, zVec3D *v, double ratio[]);

/* ********************************************************** */
/* multiplication of a 3x3 matrix by another 3x3 matrix
 * ********************************************************** */

/*! \brief multiply 3x3 matrices.
 *
 * zMulMatMat3D() multiplies a 3x3 matrix \a m2 by the other \a m1 from
 * the leftside and puts it into \a m.
 *
 * zMulMatTMat3D() multiplies a 3x3 matrix m2 by the transpose of a 3x3 matrix
 * \a m1 from the leftside and puts it into \a m.
 *
 * zMulMatMatT3D() multiplies a 3x3 matrix \a m1 by the transpose of a 3D
 * matrix \a m2 from the rightside and puts it into \a m.
 *
 * zMulMatMat3DDRC() directly multiplies a 3x3 matrix \a m2 by the other
 * \a m1 from the leftside.
 *
 * zMulMatTMat3DDRC() directly multiplies a 3x3 matrix \a m2 by the transpose
 * of the other \a m1 from the leftside.
 *
 * zMulMatMatT3DDRC() directly multiplies a 3x3 matrix \a m1 by the transpose
 * of the other \a m2 from the rightside.
 *
 * zMulInvMatMat3D() multiplies \a m2 by the inverse of a 3x3 matrix \a m1
 * from the leftside and puts it into \a m.
 * \return
 * Each function returns a pointer to the result.
 */
__EXPORT zMat3D *zMulMatMat3D(zMat3D *m1, zMat3D *m2, zMat3D *m);
__EXPORT zMat3D *zMulMatTMat3D(zMat3D *m1, zMat3D *m2, zMat3D *m);
__EXPORT zMat3D *zMulMatMatT3D(zMat3D *m1, zMat3D *m2, zMat3D *m);

#define zMulMatMat3DDRC(m1,m2)  zMulMatMat3D(m1,m2,m2)
#define zMulMatTMat3DDRC(m1,m2) zMulMatTMat3D(m1,m2,m2)
#define zMulMatMatT3DDRC(m1,m2) zMulMatMatT3D(m1,m2,m1)

__EXPORT zMat3D *zMulInvMatMat3D(zMat3D *m1, zMat3D *m2, zMat3D *m);

/* ********************************************************** */
/* multiplication of a 6D spatial vector by a 3x3 matrix
 * ********************************************************** */

/*! \brief multiply a 6D vector by a 3x3 matrix.
 *
 * zMulMatVec6D() multiplies a 6D vector \a v by a 3x3 matrix \a m and puts
 * it into \a mv.
 *
 * zMulMatTVec6D() multiplies a 6D vector \a v by the transpose of a 3x3
 * matrix \a m and puts it into \a mv.
 *
 * zMulMatVec6DDRC() directly multiplies a 6D vector \a v by a 3x3 matrix \a m.
 *
 * zMulMatTVec6DDRC() directly multiplies a 6D vector \a v by the transpose
 * of a 3x3 matrix \a m.
 * \return
 * Each function returns the pointer to the result vector.
 */
__EXPORT zVec6D *zMulMatVec6D(zMat3D *m, zVec6D *v, zVec6D *mv);
__EXPORT zVec6D *zMulMatTVec6D(zMat3D *m, zVec6D *v, zVec6D *mv);

#define zMulMatVec6DDRC(m,v)  zMulMatVec6D(m,v,v)
#define zMulMatTVec6DDRC(m,v) zMulMatTVec6D(m,v,v)

/* ********************************************************** */
/* rotation
 * ********************************************************** */

/*! \brief rotate a 3x3 matrix about a base in the specified axis.
 *
 * zMat3DRotRoll(), zMat3DRotPitch() and zMat3DRotYaw() rotate a 3x3 matrix
 * \a m with an angle \a angle about x-axis, y-axis and z-axis, respectively,
 * and puts it into \a rm. \a angle is in radian.
 *
 * zMat3DRotRollSC(), zMat3DRotPitchSC() and zMat3DRotYawSC() rotate a 3x3
 * matrix \a m not with an angle but with its trigonometric values.
 * \a s and \a c are for sine and cosine values, respectively.
 *
 * zMat3DRotRollDRC(), zMat3DRotPitchDRC(), zMat3DRotYawDRC(),
 * zMat3DRotRollSCDRC(), zMat3DRotPitchSCDRC() and zMat3DRotYawSCDRC() are
 * destructive versions of the above functions, directly updating a given
 * 3x3 matrix \a m.
 * \return
 * zMat3DRotRoll(), zMat3DRotPitch(), zMat3DRotYaw(), zMat3DRotRollSC(),
 * zMat3DRotPitchSC() and zMat3DRotYawSC() return a pointer \a rm.
 *
 * zMat3DRotRollDRC(), zMat3DRotPitchDRC(), zMat3DRotYawDRC()
 * zMat3DRotRollSCDRC(), zMat3DRotPitchSCDRC() and zMat3DRotYawSCDRC()
 * return a pointer \a m.
 */
__EXPORT zMat3D *zMat3DRotRoll(zMat3D *m, double angle, zMat3D *rm);
__EXPORT zMat3D *zMat3DRotPitch(zMat3D *m, double angle, zMat3D *rm);
__EXPORT zMat3D *zMat3DRotYaw(zMat3D *m, double angle, zMat3D *rm);
__EXPORT zMat3D *zMat3DRotRollSC(zMat3D *m, double s, double c, zMat3D *rm);
__EXPORT zMat3D *zMat3DRotPitchSC(zMat3D *m, double s, double c, zMat3D *rm);
__EXPORT zMat3D *zMat3DRotYawSC(zMat3D *m, double s, double c, zMat3D *rm);
__EXPORT zMat3D *zMat3DRotRollDRC(zMat3D *m, double theta);
__EXPORT zMat3D *zMat3DRotPitchDRC(zMat3D *m, double theta);
__EXPORT zMat3D *zMat3DRotYawDRC(zMat3D *m, double theta);
__EXPORT zMat3D *zMat3DRotRollSCDRC(zMat3D *m, double s, double c);
__EXPORT zMat3D *zMat3DRotPitchSCDRC(zMat3D *m, double s, double c);
__EXPORT zMat3D *zMat3DRotYawSCDRC(zMat3D *m, double s, double c);

/*! \brief 3D attitude alternation with matrix.
 *
 * zMat3DZYX() calculates a 3D attitude matrix from z-y-x Eulerian angles.
 * The identity matrix is rotated firstly by \a azim about z-axis, secondly
 * by \a elev about y-axis, and finally by \a tilt about x-axis.
 * The result is put into \a m.
 *
 * zMat3DZYXSC() directly accepts trigonometric values of z-y-x Eulerian
 * angles. \a sa/\a ca, \a se/\a ce and \a st/\a ct are for azimuth,
 * elevation and tilt angles, respectively.
 *
 * zMat3DToZYX() is the inverse transformation of zMat3DZYX() from a 3D
 * attitude matrix to a quasi 3D vector which respresents z-y-x Eulerian
 * angles \a angle in the order of azimuth, elevation and tilt. The result
 * is put into \a angle.
 *
 * zMat3DZYZ() calculates a 3D attitude matrix from z-y-z Eulerian angles.
 * The identity matrix is rotated firstly by \a heading about z-axis,
 * secondly by \a pitch about y-axis, and finally by \a bank about z-axis.
 * The result is put into \a m.
 *
 * zMat3DZYZSC() directly accepts trigonometric values of z-y-x Eulerian
 * angles. \a sh/\a ch, \a sp/\a cp and \a sb/\a cb are for heading, pitch
 * and bank angles, respectively.
 *
 * zMat3DToZYZ() is the inverse transformation of zMat3DZYZ() from a 3D
 * attitude matrix to a quasi 3D vector which respresents z-y-z Eulerian
 * angles \a angle in the order of heading, pitch and bank. The result is
 * put into \a angle.
 *
 * zMat3DAA() calculates a 3D attitude matrix from the equivalent angle-axis
 * vector \a aa. The identity matrix is rotated about the direction of
 * \a aa with the angle equal to the norm of \a aa. The result is put into
 * \a m.
 *
 * zMat3DToAA() is the inverse transformation of zMat3DAA() from a 3D
 * attitude matrix to the equivalent angle-axis vector. The result is put
 * into \a aa.
 * \return
 * zMat3DZYX(), zMat3DZYXSC(), zMat3DZYZ(), zMat3DZYZSC() and zMat3DAA()
 * return a pointer \a m.
 *
 * zMat3DToZYX() and zMat3DToZYZ() return a pointer \a angle.
 *
 * zMat3DToAA() returns a pointer \a aa.
 */
__EXPORT zMat3D *zMat3DZYX(zMat3D *m, double azim, double elev, double tilt);
__EXPORT zMat3D *zMat3DZYXSC(zMat3D *m, double sa, double ca, double se, double ce, double st, double ct);
__EXPORT zVec3D *zMat3DToZYX(zMat3D *m, zVec3D *angle);
__EXPORT zMat3D *zMat3DZYZ(zMat3D *m, double heading, double pitch, double bank);
__EXPORT zMat3D *zMat3DZYZSC(zMat3D *m, double sh, double ch, double sp, double cp, double sb, double cb);
__EXPORT zVec3D *zMat3DToZYZ(zMat3D *m, zVec3D *angle);
__EXPORT zMat3D *zMat3DAA(zMat3D *m, zVec3D *aa);
__EXPORT zVec3D *zMat3DToAA(zMat3D *m, zVec3D *aa);

/*! \brief rotational multiplication of a 3D attitude matrix.
 *
 * zRotMat3D() multiplies a 3x3 matrix \a m by the other \a r from the
 * leftside, then does it by the transpose of \a r from the rightside,
 * and puts the result into \a rm. Namely,
 *   \a rm = \a r \a m \a r^T.
 *
 * zRotMat3DInv() is the opposite computation of zRotMat3D(). Namely,
 *   \a rm = \a r^T \a m \a r.
 *
 * zRotMat3DDRC() is the same computation with zRotMat3D() except that
 * it puts the result directly into \a m (destructive).
 * zRotMat3DInvDRC() is the same computation with zRotMat3DInv() except
 * that it puts the result directly into \a m (destructive).
 * \return
 * Each function returns the pointer to the result.
 */
__EXPORT zMat3D *zRotMat3D(zMat3D *r, zMat3D *m, zMat3D *rm);
__EXPORT zMat3D *zRotMat3DInv(zMat3D *r, zMat3D *m, zMat3D *rm);

#define zRotMat3DDRC(r,m)    zRotMat3D(r,m,m)
#define zRotMat3DInvDRC(r,m) zRotMat3DInv(r,m,m)

/*! \brief multiply cross product of a 3D vector and a 3x3 matrix.
 *
 * zMulVecOPMat3D() multiplies a 3x3 matrix \a m by the outer product of
 * a 3D vector \a ohm and puts it into \a mv. Namely,
 *   \a mv = \a ohm x \a m.
 *
 * zMulVecOPMat3DDRC() directly multiplies a 3x3 matrix \a m by the outer
 * product of a 3D vector \a ohm.
 * \return
 * zMulVecOPMat3D() returns a pointer \a mv.
 * zMulVecOPMat3DDRC() returns a pointer \a m.
 */
__EXPORT zMat3D *zMulVecOPMat3D(zVec3D *ohm, zMat3D *m, zMat3D *mv);
#define zMulVecOPMat3DDRC(o,m) zMulVecOPMat3D( o, m, m )

/*! \brief rotate a 3D attitude matrix about an arbitrary axis.
 *
 * zMat3DRot() rotates a 3D attitude matrix \a m by an angle-axis vector
 * \a aa. The rotation axis is in parallel to \a aa and the norm of \a aa
 * is the rotation angle in radian. The direction of rotation is according
 * to right-handed screw rule. The result is put into \a rm.
 * \return
 * zMat3DRot() returns a pointer \a rm.
 */
__EXPORT zMat3D *zMat3DRot(zMat3D *m, zVec3D *aa, zMat3D *rm);

__EXPORT zMat3D *zMat3DRotCat(zMat3D *m, zVec3D *omega, double dt, zMat3D *rm);

/*! \brief cascade an angle-axis vector to another.
 */
__EXPORT zVec3D *zAACascade(zVec3D *aa1, zVec3D *aa2, zVec3D *aa);

/*! \brief error vector between two 3D attitude matrices.
 *
 * zMat3DError() calculates the error vector, namely, the equivalent
 * angle-axis vector from a 3D attitude matrix \a m2 to the other \a m1
 * (note the order) and puts it into \a err.
 * \return
 * zMat3DError() returns a pointer \a err.
 */
__EXPORT zVec3D *zMat3DError(zMat3D *m1, zMat3D *m2, zVec3D *err);

/*! \brief error between two angle-axis vectors.
 *
 * zAAError() calculates the error vector, namely, the angle-axis vector
 * from an attitude represented by \a a2 to another \a a1 (note the order).
 * The result is put into \a err.
 * \return
 * zAAError() returns a pointer \a err.
 */
__EXPORT zVec3D *zAAError(zVec3D *a1, zVec3D *a2, zVec3D *err);

/* ********************************************************** */
/* differential kinematics
 * ********************************************************** */

__EXPORT zVec3D *zMat3DDif(zMat3D *m, zMat3D *mnew, double dt, zVec3D *omega);

/* ********************************************************** */
/* eigensystem
 * ********************************************************** */

/*! \brief eigenvalues of a symmetric 3x3 matrix by Jacobi's method.
 *
 * zMat3DSymEig() calculates eigenvalues and eigenvectors of a symmetric
 * 3x3 matrix \a m with Jacobi's method. Each eigenvalue and eigenvector
 * are stored in \a eval and \a evec in the corresponding order.
 * \return
 * zMat3DSymEig() returns no value.
 * \notes
 * \a m must be symmetric. Otherwise, the correct result will not be expected.
 */
__EXPORT void zMat3DSymEig(zMat3D *m, double eval[], zVec3D evec[]);

/* ********************************************************** */
/* I/O
 * ********************************************************** */

/*! \brief input and output of a 3x3 matrix.
 *
 * zMat3DFRead() reads nine values from the current position of a file
 * \a fp and creates a 3x3 matrix \a m from them.
 * zMat3DRead() reads nine values from the standard input.
 *
 * zMat3DFWrite() outputs a 3x3 matrix \a m to the current position of
 * a file \a fp in the following format.
 *  {
 *   a11, a12, a13
 *   a21, a22, a23
 *   a31, a32, a33
 *  }
 * When the null pointer is given, it outputs the following string.
 *  (null 3x3 matrix)
 * zMat3DWrite() outputs a 3x3 matrix \a m to the standard output.
 * \return
 * zMat3DFRead() and zMat3DRead() return a pointer \a m.
 *
 * zMat3DFWrite() and zMat3DWrite() return no value.
 */
__EXPORT zMat3D *zMat3DFRead(FILE *fp, zMat3D *m);
#define zMat3DRead(m) zMat3DFRead( stdin, (m) )
__EXPORT void zMat3DFWrite(FILE *fp, zMat3D *m);
#define zMat3DWrite(m) zMat3DFWrite( stdout, (m) )

/* zMat3DFWriteXML - xml output.
 * ... still testing.
 */
__EXPORT void zMat3DFWriteXML(FILE *fp, zMat3D *m);

__END_DECLS

#include <zeo/zeo_vec3d_pca.h> /* principal component analysis */

#endif /* __ZEO_MAT3D_H__ */
