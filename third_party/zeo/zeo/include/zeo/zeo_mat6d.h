/* Zeo - Z/Geometry and optics computation library.
 * Copyright (C) 2005 Tomomichi Sugihara (Zhidao)
 *
 * zeo_mat6d - 6x6 matrix.
 * This class was originally defined by N. Wakisaka in 2011.
 */

#ifndef __ZEO_MAT6D_H__
#define __ZEO_MAT6D_H__

#include <zeo/zeo_mat3d.h>

__BEGIN_DECLS

/*! \struct zMat6D
 * \brief 6x6 matrix, which has 4 3x3 matrices within.
 */
typedef struct{
  zMat3D m[4]; /*!< \brief four 3x3 matrices */
} zMat6D;

#define zMat6DElem(mat,i,j)  zMat3DElem(&(mat)->m[(i/3)*2+(j/3)], i%3, j%3 )
#define zMat6DMat3D(mat,i,j) ( &(mat)->m[(i)*2+(j)] )

/*! \brief create a 6x6 matrix. */
__EXPORT zMat6D *zMat6DCreate(zMat6D *m, zMat3D *m1, zMat3D *m2, zMat3D *m3, zMat3D *m4);
/*! \brief copy a 6x6 matrix to another */
#define zMat6DCopy(s,d) zCopy( zMat6D, s, d )
/*! \brief put a sub 3x3 matrix into a 6x6 matrix. */
__EXPORT zMat6D *zMat6DPutMat3D(zMat6D *m6d, int i, int j, zMat3D *m3d);
/*! \brief clear a 6x6 matrix to be zero. */
__EXPORT zMat6D *zMat6DClear(zMat6D *m);

/*! \brief transpose of a 6x6 matrix. */
__EXPORT zMat6D *zMat6DT(zMat6D *m, zMat6D *mout);
/*! \brief abstract row/column vectors from a 6x6 matrix.
 *
 * zMat6DRow() abstracts the \a i'th row from a 6x6 matrix \a m and puts
 * it into \a v.
 *
 * zMat6DCol() abstracts the \a i'th column from a 6x6 matrix \a m and
 * puts it into \a v.
 * \return
 * zMat6DRow() and zMat6DCol() return a pointer \a v.
 */
__EXPORT zVec6D *zMat6DRow(zMat6D *m, int i, zVec6D *v);
__EXPORT zVec6D *zMat6DCol(zMat6D *m, int i, zVec6D *v);

/*! \brief multiply a 6x1 vector by a 6x6 matrix from the left side.
 * \a vout = \a m \a vin
 */
__EXPORT zVec6D *zMulMat6DVec6D(zMat6D *m, zVec6D *vin, zVec6D *vout);

/*! \brief multiply a 6x1 vector by the transpose of a 6x6 matrix from the left side.
 * \a vout = \a m^T \a vin
 */
__EXPORT zVec6D *zMulMat6DTVec6D(zMat6D *m, zVec6D *vin, zVec6D *vout);

/*! \brief add two 6x6 matrices.
 * \a mout = \a m1 + \a m2
 */
__EXPORT zMat6D *zMat6DAdd(zMat6D *m1, zMat6D *m2, zMat6D *mout);

/*! \brief subtract a 6x6 matrix from another.
 * \a mout = \a m1 - \a m2
 */
__EXPORT zMat6D *zMat6DSub(zMat6D *m1, zMat6D *m2, zMat6D *mout);

/*! \brief multiply a 6x6 matrix by a scalar value.
 * \a mout = \a k \a m
 */
__EXPORT zMat6D *zMat6DMul(zMat6D *m, double k, zMat6D *mout);

/*! \brief divide a 6x6 matrix by a scalar value.
 * \a mout = 1/\a k \a m
 */
__EXPORT zMat6D *zMat6DDiv(zMat6D *m, double k, zMat6D *mout);

#define zMat6DAddDRC(m1,m2) zMat6DAdd(m1,m2,m1)
#define zMat6DSubDRC(m1,m2) zMat6DSub(m1,m2,m1)
#define zMat6DMulDRC(m,k)   zMat6DMul(m,k,m)
#define zMat6DDivDRC(m,k)   zMat6DDiv(m,k,m)

/*! \brief multiply a 6x6 matrix by another.
 * \a mout = \a m1 \a m2
 */
__EXPORT zMat6D *zMulMatMat6D(zMat6D *m1, zMat6D *m2, zMat6D *mout);

/*! \brief multiply a 6x6 matrix by the transpose of another from the left side.
 * \a mout = \a m1^T \a m2
 */
__EXPORT zMat6D *zMulMatTMat6D(zMat6D *m1, zMat6D *m2, zMat6D *mout);

/*! \brief multiply a 6x6 matrix by the transpose of another from the right side.
 * \a mout = \a m1 \a m2^T
 */
__EXPORT zMat6D *zMulMatMatT6D(zMat6D *m1, zMat6D *m2, zMat6D *mout);

/*! \brief dyadic product of two 6x1 vectors. */
__EXPORT zMat6D *zMat6DDyad(zVec6D *v1, zVec6D *v2, zMat6D *mout);

/*! \brief output a 6x6 matrix to the current position of a file. */
__EXPORT void zMat6DFWrite(FILE *fp, zMat6D *m);
#define zMat6DWrite(m) zMat6DFWrite( stdout, (m) )

__END_DECLS

#endif /* __ZEO_MAT6D_H__ */
