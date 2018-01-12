/* Zeo - Z/Geometry and optics computation library.
 * Copyright (C) 2005 Tomomichi Sugihara (Zhidao)
 *
 * zeo_mat6d - 6x6 matrix.
 * This class was originally defined by N. Wakisaka in 2011.
 */

#include <zeo/zeo_mat6d.h>

/* zMat6DCreate
 * - create a 6x6 matrix.
 */
zMat6D *zMat6DCreate(zMat6D *m, zMat3D *m1, zMat3D *m2, zMat3D *m3, zMat3D *m4)
{
  zMat3DCopy( m1, &m->m[0] );
  zMat3DCopy( m2, &m->m[1] );
  zMat3DCopy( m3, &m->m[2] );
  zMat3DCopy( m4, &m->m[3] );
  return m;
}

/* zMat6DPutMat3D
 * - put a sub 3x3 matrix into a 6x6 matrix.
 */
zMat6D *zMat6DPutMat3D(zMat6D *m6d, int i, int j, zMat3D *m3d)
{
  zMat3DCopy( m3d, zMat6DMat3D(m6d,i,j) );
  return m6d;
}

/* zMat6DClear
 * - clear a 6x6 matrix to be zero.
 */
zMat6D *zMat6DClear(zMat6D *m)
{
  zMat3DClear( &m->m[0] );
  zMat3DClear( &m->m[1] );
  zMat3DClear( &m->m[2] );
  zMat3DClear( &m->m[3] );
  return m;
}

/* zMat6DT
 * - transpose of a 6x6 matrix.
 */
zMat6D *zMat6DT(zMat6D *m, zMat6D *mout)
{
  zMat3DT( &m->m[0], &mout->m[0] );
  zMat3DT( &m->m[1], &mout->m[2] );
  zMat3DT( &m->m[2], &mout->m[1] );
  zMat3DT( &m->m[3], &mout->m[3] );
  return mout;
}

/* zMat6DRow
 * - abstract row vector from 6D matrix.
 */
zVec6D *zMat6DRow(zMat6D *m, int i, zVec6D *v)
{
  int a, b;
  a = (i/3)*2;
  b = i%3;
  v->e[0] = m->m[a  ].e[0][b];
  v->e[1] = m->m[a  ].e[1][b];
  v->e[2] = m->m[a  ].e[2][b];
  v->e[3] = m->m[a+1].e[0][b];
  v->e[4] = m->m[a+1].e[1][b];
  v->e[5] = m->m[a+1].e[2][b];
  return v;
}

/* zMat6DCol
 * - abstract column vector from 6D matrix.
 */
zVec6D *zMat6DCol(zMat6D *m, int i, zVec6D *v)
{
  int a, b;
  a = i/3;
  b = i%3;
  v->e[0] = m->m[  a].e[b][0];
  v->e[1] = m->m[  a].e[b][1];
  v->e[2] = m->m[  a].e[b][2];
  v->e[3] = m->m[2+a].e[b][0];
  v->e[4] = m->m[2+a].e[b][1];
  v->e[5] = m->m[2+a].e[b][2];
  return v;
}

#define __zVec6DCreate(v,x,y,z,xa,ya,za) do{\
    double __x, __xa, __y, __ya, __z, __za; \
  __x = x;\
  __y = y;\
  __z = z;\
  __xa = xa;\
  __ya = ya;\
  __za = za;\
  (v)->e[zX] = __x;\
  (v)->e[zY] = __y;\
  (v)->e[zZ] = __z;\
  (v)->e[zXA] = __xa;\
  (v)->e[zYA] = __ya;\
  (v)->e[zZA] = __za;\
} while(0)

#define __zMulMat6DVec6D(m,v,mv) __zVec6DCreate( mv,\
  (m)->m[0].e[0][0]*(v)->e[0] + (m)->m[0].e[1][0]*(v)->e[1] + (m)->m[0].e[2][0]*(v)->e[2] + (m)->m[1].e[0][0]*(v)->e[3] + (m)->m[1].e[1][0]*(v)->e[4] + (m)->m[1].e[2][0]*(v)->e[5],\
  (m)->m[0].e[0][1]*(v)->e[0] + (m)->m[0].e[1][1]*(v)->e[1] + (m)->m[0].e[2][1]*(v)->e[2] + (m)->m[1].e[0][1]*(v)->e[3] + (m)->m[1].e[1][1]*(v)->e[4] + (m)->m[1].e[2][1]*(v)->e[5],\
  (m)->m[0].e[0][2]*(v)->e[0] + (m)->m[0].e[1][2]*(v)->e[1] + (m)->m[0].e[2][2]*(v)->e[2] + (m)->m[1].e[0][2]*(v)->e[3] + (m)->m[1].e[1][2]*(v)->e[4] + (m)->m[1].e[2][2]*(v)->e[5],\
  (m)->m[2].e[0][0]*(v)->e[0] + (m)->m[2].e[1][0]*(v)->e[1] + (m)->m[2].e[2][0]*(v)->e[2] + (m)->m[3].e[0][0]*(v)->e[3] + (m)->m[3].e[1][0]*(v)->e[4] + (m)->m[3].e[2][0]*(v)->e[5],\
  (m)->m[2].e[0][1]*(v)->e[0] + (m)->m[2].e[1][1]*(v)->e[1] + (m)->m[2].e[2][1]*(v)->e[2] + (m)->m[3].e[0][1]*(v)->e[3] + (m)->m[3].e[1][1]*(v)->e[4] + (m)->m[3].e[2][1]*(v)->e[5],\
  (m)->m[2].e[0][2]*(v)->e[0] + (m)->m[2].e[1][2]*(v)->e[1] + (m)->m[2].e[2][2]*(v)->e[2] + (m)->m[3].e[0][2]*(v)->e[3] + (m)->m[3].e[1][2]*(v)->e[4] + (m)->m[3].e[2][2]*(v)->e[5])

#define __zMulMat6DTVec6D(m,v,mv) __zVec6DCreate( mv,\
  (m)->m[0].e[0][0]*(v)->e[0] + (m)->m[0].e[0][1]*(v)->e[1] + (m)->m[0].e[0][2]*(v)->e[2] + (m)->m[2].e[0][0]*(v)->e[0] + (m)->m[2].e[0][1]*(v)->e[1] + (m)->m[2].e[0][2]*(v)->e[2],\
  (m)->m[0].e[1][0]*(v)->e[0] + (m)->m[0].e[1][1]*(v)->e[1] + (m)->m[0].e[1][2]*(v)->e[2] + (m)->m[2].e[1][0]*(v)->e[0] + (m)->m[2].e[1][1]*(v)->e[1] + (m)->m[2].e[1][2]*(v)->e[2],\
  (m)->m[0].e[2][0]*(v)->e[0] + (m)->m[0].e[2][1]*(v)->e[1] + (m)->m[0].e[2][2]*(v)->e[2] + (m)->m[2].e[2][0]*(v)->e[0] + (m)->m[2].e[2][1]*(v)->e[1] + (m)->m[2].e[2][2]*(v)->e[2],\
  (m)->m[1].e[0][0]*(v)->e[0] + (m)->m[1].e[0][1]*(v)->e[1] + (m)->m[1].e[0][2]*(v)->e[2] + (m)->m[3].e[0][0]*(v)->e[0] + (m)->m[3].e[0][1]*(v)->e[1] + (m)->m[3].e[0][2]*(v)->e[2],\
  (m)->m[1].e[1][0]*(v)->e[0] + (m)->m[1].e[1][1]*(v)->e[1] + (m)->m[1].e[1][2]*(v)->e[2] + (m)->m[3].e[1][0]*(v)->e[0] + (m)->m[3].e[1][1]*(v)->e[1] + (m)->m[3].e[1][2]*(v)->e[2],\
  (m)->m[1].e[2][0]*(v)->e[0] + (m)->m[1].e[2][1]*(v)->e[1] + (m)->m[1].e[2][2]*(v)->e[2] + (m)->m[3].e[2][0]*(v)->e[0] + (m)->m[3].e[2][1]*(v)->e[1] + (m)->m[3].e[2][2]*(v)->e[2])
/* zMulMat6DVec6D
 * - multiply a 6x1 vector by a 6x6 matrix from the left side.
 */
zVec6D *zMulMat6DVec6D(zMat6D *m, zVec6D *vin, zVec6D *vout)
{
  __zMulMat6DVec6D(m, vin, vout);
  /* zVec3D v1, v2; */
  /* zMulMatVec3D( &m->m[0], zVec6DLin(vin), &v1 ); */
  /* zMulMatVec3D( &m->m[1], zVec6DAng(vin), &v2 ); */
  /* zVec3DAdd( &v1, &v2, zVec6DLin(vout) ); */
  /* zMulMatVec3D( &m->m[2], zVec6DLin(vin), &v1 ); */
  /* zMulMatVec3D( &m->m[3], zVec6DAng(vin), &v2 ); */
  /* zVec3DAdd( &v1, &v2, zVec6DAng(vout) ); */
  return vout;
}

/* zMulMat6DTVec6D
 * - multiply a 6x1 vector by the transpose of a 6x6 matrix from the left side.
 */
zVec6D *zMulMat6DTVec6D(zMat6D *m, zVec6D *vin, zVec6D *vout)
{
  __zMulMat6DTVec6D(m, vin, vout);
  /* zVec3D v1, v2; */
  /* zMulMatTVec3D( &m->m[0], zVec6DLin(vin), &v1 ); */
  /* zMulMatTVec3D( &m->m[2], zVec6DAng(vin), &v2 ); */
  /* zVec3DAdd( &v1, &v2, zVec6DLin(vout) ); */
  /* zMulMatTVec3D( &m->m[1], zVec6DLin(vin), &v1 ); */
  /* zMulMatTVec3D( &m->m[3], zVec6DAng(vin), &v2 ); */
  /* zVec3DAdd( &v1, &v2, zVec6DAng(vout) ); */
  return vout;
}

/* zMat6DAdd
 * - add two 6x6 matrices.
 */
zMat6D *zMat6DAdd(zMat6D *m1, zMat6D *m2, zMat6D *mout)
{
  mout->m[0].e[0][0] = m1->m[0].e[0][0] + m2->m[0].e[0][0];
  mout->m[0].e[1][0] = m1->m[0].e[1][0] + m2->m[0].e[1][0];
  mout->m[0].e[2][0] = m1->m[0].e[2][0] + m2->m[0].e[2][0];
  mout->m[0].e[0][1] = m1->m[0].e[0][1] + m2->m[0].e[0][1];
  mout->m[0].e[1][1] = m1->m[0].e[1][1] + m2->m[0].e[1][1];
  mout->m[0].e[2][1] = m1->m[0].e[2][1] + m2->m[0].e[2][1];
  mout->m[0].e[0][2] = m1->m[0].e[0][2] + m2->m[0].e[0][2];
  mout->m[0].e[1][2] = m1->m[0].e[1][2] + m2->m[0].e[1][2];
  mout->m[0].e[2][2] = m1->m[0].e[2][2] + m2->m[0].e[2][2];

  mout->m[1].e[0][0] = m1->m[1].e[0][0] + m2->m[1].e[0][0];
  mout->m[1].e[1][0] = m1->m[1].e[1][0] + m2->m[1].e[1][0];
  mout->m[1].e[2][0] = m1->m[1].e[2][0] + m2->m[1].e[2][0];
  mout->m[1].e[0][1] = m1->m[1].e[0][1] + m2->m[1].e[0][1];
  mout->m[1].e[1][1] = m1->m[1].e[1][1] + m2->m[1].e[1][1];
  mout->m[1].e[2][1] = m1->m[1].e[2][1] + m2->m[1].e[2][1];
  mout->m[1].e[0][2] = m1->m[1].e[0][2] + m2->m[1].e[0][2];
  mout->m[1].e[1][2] = m1->m[1].e[1][2] + m2->m[1].e[1][2];
  mout->m[1].e[2][2] = m1->m[1].e[2][2] + m2->m[1].e[2][2];

  mout->m[2].e[0][0] = m1->m[2].e[0][0] + m2->m[2].e[0][0];
  mout->m[2].e[1][0] = m1->m[2].e[1][0] + m2->m[2].e[1][0];
  mout->m[2].e[2][0] = m1->m[2].e[2][0] + m2->m[2].e[2][0];
  mout->m[2].e[0][1] = m1->m[2].e[0][1] + m2->m[2].e[0][1];
  mout->m[2].e[1][1] = m1->m[2].e[1][1] + m2->m[2].e[1][1];
  mout->m[2].e[2][1] = m1->m[2].e[2][1] + m2->m[2].e[2][1];
  mout->m[2].e[0][2] = m1->m[2].e[0][2] + m2->m[2].e[0][2];
  mout->m[2].e[1][2] = m1->m[2].e[1][2] + m2->m[2].e[1][2];
  mout->m[2].e[2][2] = m1->m[2].e[2][2] + m2->m[2].e[2][2];

  mout->m[3].e[0][0] = m1->m[3].e[0][0] + m2->m[3].e[0][0];
  mout->m[3].e[1][0] = m1->m[3].e[1][0] + m2->m[3].e[1][0];
  mout->m[3].e[2][0] = m1->m[3].e[2][0] + m2->m[3].e[2][0];
  mout->m[3].e[0][1] = m1->m[3].e[0][1] + m2->m[3].e[0][1];
  mout->m[3].e[1][1] = m1->m[3].e[1][1] + m2->m[3].e[1][1];
  mout->m[3].e[2][1] = m1->m[3].e[2][1] + m2->m[3].e[2][1];
  mout->m[3].e[0][2] = m1->m[3].e[0][2] + m2->m[3].e[0][2];
  mout->m[3].e[1][2] = m1->m[3].e[1][2] + m2->m[3].e[1][2];
  mout->m[3].e[2][2] = m1->m[3].e[2][2] + m2->m[3].e[2][2];

  /* zMat3DAdd( &m1->m[0], &m2->m[0], &mout->m[0] ); */
  /* zMat3DAdd( &m1->m[1], &m2->m[1], &mout->m[1] ); */
  /* zMat3DAdd( &m1->m[2], &m2->m[2], &mout->m[2] ); */
  /* zMat3DAdd( &m1->m[3], &m2->m[3], &mout->m[3] ); */
  return mout;
}

/* zMat6DSub
 * - subtract a 6x6 matrix from another.
 */
zMat6D *zMat6DSub(zMat6D *m1, zMat6D *m2, zMat6D *mout)
{
  zMat3DSub( &m1->m[0], &m2->m[0], &mout->m[0] );
  zMat3DSub( &m1->m[1], &m2->m[1], &mout->m[1] );
  zMat3DSub( &m1->m[2], &m2->m[2], &mout->m[2] );
  zMat3DSub( &m1->m[3], &m2->m[3], &mout->m[3] );
  return mout;
}

/* zMat6DMul
 * - multiply a 6x6 matrix by a scalar value.
 */
zMat6D *zMat6DMul(zMat6D *m, double k, zMat6D *mout)
{
  zMat3DMul( &m->m[0], k, &mout->m[0] );
  zMat3DMul( &m->m[1], k, &mout->m[1] );
  zMat3DMul( &m->m[2], k, &mout->m[2] );
  zMat3DMul( &m->m[3], k, &mout->m[3] );
  return mout;
}

/* zMat6DDiv
 * - divide a 6x6 matrix by a scalar value.
 */
zMat6D *zMat6DDiv(zMat6D *m, double k, zMat6D *mout)
{
  if( k == 0 ){
    ZRUNWARN( ZEO_ERR_ZERODIV );
    return NULL;
  }
  k = 1.0 / k;
  zMat3DMul( &m->m[0], k, &mout->m[0] );
  zMat3DMul( &m->m[1], k, &mout->m[1] );
  zMat3DMul( &m->m[2], k, &mout->m[2] );
  zMat3DMul( &m->m[3], k, &mout->m[3] );
  return mout;
}

/* zMulMatMat6D
 * - multiply a 6x6 matrix by another.
 */
zMat6D *zMulMatMat6D(zMat6D *m1, zMat6D *m2, zMat6D *mout)
{
  zMat3D tmp1, tmp2;

  zMulMatMat3D( &m1->m[0], &m2->m[0], &tmp1 );
  zMulMatMat3D( &m1->m[1], &m2->m[2], &tmp2 );
  zMat3DAdd( &tmp1, &tmp2, &mout->m[0] );
  zMulMatMat3D( &m1->m[0], &m2->m[1], &tmp1 );
  zMulMatMat3D( &m1->m[1], &m2->m[3], &tmp2 );
  zMat3DAdd( &tmp1, &tmp2, &mout->m[1] );
  zMulMatMat3D( &m1->m[2], &m2->m[0], &tmp1 );
  zMulMatMat3D( &m1->m[3], &m2->m[2], &tmp2 );
  zMat3DAdd( &tmp1, &tmp2, &mout->m[2] );
  zMulMatMat3D( &m1->m[2], &m2->m[1], &tmp1 );
  zMulMatMat3D( &m1->m[3], &m2->m[3], &tmp2 );
  zMat3DAdd( &tmp1, &tmp2, &mout->m[3] );
  return mout;
}

/* zMulMatTMat6D
 * - multiply a 6x6 matrix by the transpose of another from the left side.
 */
zMat6D *zMulMatTMat6D(zMat6D *m1, zMat6D *m2, zMat6D *mout)
{
  zMat3D tmp1, tmp2;

  zMulMatTMat3D( &m1->m[0], &m2->m[0], &tmp1 );
  zMulMatTMat3D( &m1->m[2], &m2->m[2], &tmp2 );
  zMat3DAdd( &tmp1, &tmp2, &mout->m[0] );
  zMulMatTMat3D( &m1->m[0], &m2->m[1], &tmp1 );
  zMulMatTMat3D( &m1->m[2], &m2->m[3], &tmp2 );
  zMat3DAdd( &tmp1, &tmp2, &mout->m[1] );
  zMulMatTMat3D( &m1->m[1], &m2->m[0], &tmp1 );
  zMulMatTMat3D( &m1->m[3], &m2->m[2], &tmp2 );
  zMat3DAdd( &tmp1, &tmp2, &mout->m[2] );
  zMulMatTMat3D( &m1->m[1], &m2->m[1], &tmp1 );
  zMulMatTMat3D( &m1->m[3], &m2->m[3], &tmp2 );
  zMat3DAdd( &tmp1, &tmp2, &mout->m[3] );
  return mout;
}

/* zMulMatMatT6D
 * - multiply a 6x6 matrix by the transpose of another from the right side.
 */
zMat6D *zMulMatMatT6D(zMat6D *m1, zMat6D *m2, zMat6D *mout)
{
  zMat3D tmp1, tmp2;

  zMulMatMatT3D( &m1->m[0], &m2->m[0], &tmp1 );
  zMulMatMatT3D( &m1->m[1], &m2->m[1], &tmp2 );
  zMat3DAdd( &tmp1, &tmp2, &mout->m[0] );
  zMulMatMatT3D( &m1->m[0], &m2->m[2], &tmp1 );
  zMulMatMatT3D( &m1->m[1], &m2->m[3], &tmp2 );
  zMat3DAdd( &tmp1, &tmp2, &mout->m[1] );
  zMulMatMatT3D( &m1->m[2], &m2->m[0], &tmp1 );
  zMulMatMatT3D( &m1->m[3], &m2->m[1], &tmp2 );
  zMat3DAdd( &tmp1, &tmp2, &mout->m[2] );
  zMulMatMatT3D( &m1->m[2], &m2->m[2], &tmp1 );
  zMulMatMatT3D( &m1->m[3], &m2->m[3], &tmp2 );
  zMat3DAdd( &tmp1, &tmp2, &mout->m[3] );
  return mout;
}

/* zMat6DDyad
 * - dyadic product of two 6x1 vectors.
 */
zMat6D *zMat6DDyad(zVec6D *v1, zVec6D *v2, zMat6D *mout)
{
  zMat3DDyad( zVec6DLin(v1), zVec6DLin(v2), &mout->m[0] );
  zMat3DDyad( zVec6DLin(v1), zVec6DAng(v2), &mout->m[1] );
  zMat3DDyad( zVec6DAng(v1), zVec6DLin(v2), &mout->m[2] );
  zMat3DDyad( zVec6DAng(v1), zVec6DAng(v2), &mout->m[3] );
  return mout;
}

/* zMat6DFWrite
 * - output a 6x6 matrix to the current position of a file.
 */
void zMat6DFWrite(FILE *fp, zMat6D *m)
{
  if( !m )
    fprintf( fp, "(null 6D matrix)\n" );
  else{
    fprintf( fp, "{\n" );
    fprintf( fp, " %.10g, %.10g, %.10g | %.10g, %.10g, %.10g\n",
      m->m[0].e[0][0], m->m[0].e[1][0], m->m[0].e[2][0],
      m->m[1].e[0][0], m->m[1].e[1][0], m->m[1].e[2][0] );
    fprintf( fp, " %.10g, %.10g, %.10g | %.10g, %.10g, %.10g\n",
      m->m[0].e[0][1], m->m[0].e[1][1], m->m[0].e[2][1],
      m->m[1].e[0][1], m->m[1].e[1][1], m->m[1].e[2][1] );
    fprintf( fp, " %.10g, %.10g, %.10g | %.10g, %.10g, %.10g\n",
      m->m[0].e[0][2], m->m[0].e[1][2], m->m[0].e[2][2],
      m->m[1].e[0][2], m->m[1].e[1][2], m->m[1].e[2][2] );
    fprintf( fp, "------------------------------------------\n" );
    fprintf( fp, " %.10g, %.10g, %.10g | %.10g, %.10g, %.10g\n",
      m->m[2].e[0][0], m->m[2].e[1][0], m->m[2].e[2][0],
      m->m[3].e[0][0], m->m[3].e[1][0], m->m[3].e[2][0] );
    fprintf( fp, " %.10g, %.10g, %.10g | %.10g, %.10g, %.10g\n",
      m->m[2].e[0][1], m->m[2].e[1][1], m->m[2].e[2][1],
      m->m[3].e[0][1], m->m[3].e[1][1], m->m[3].e[2][1] );
    fprintf( fp, " %.10g, %.10g, %.10g | %.10g, %.10g, %.10g\n",
      m->m[2].e[0][2], m->m[2].e[1][2], m->m[2].e[2][2],
      m->m[3].e[0][2], m->m[3].e[1][2], m->m[3].e[2][2] );
    fprintf( fp, "}\n" );
  }
}
