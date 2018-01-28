/* Zeo - Z/Geometry and optics computation library.
 * Copyright (C) 2005 Tomomichi Sugihara (Zhidao)
 *
 * zeo_vec3d_pca - principal component analysis for 3D vectors.
 */

#include <zeo/zeo_mat3d.h>

/* zVec3DBarycenterPL
 * - barycenter of vector cloud given by a list.
 */
zVec3D *zVec3DBarycenterPL(zVec3DList *vl, zVec3D *c)
{
  zVec3DListCell *vc;

  zVec3DClear( c );
  zListForEach( vl, vc )
    zVec3DAddDRC( c, vc->data );
  return zVec3DDivDRC( c, zListNum(vl) );
}

/* zVec3DBarycenter
 * - barycenter of vector cloud given by an array.
 */
zVec3D *zVec3DBarycenter(zVec3D v[], int num, zVec3D *c)
{
  register int i;

  zVec3DClear( c );
  for( i=0; i<num; i++ )
    zVec3DAddDRC( c, &v[i] );
  return zVec3DDivDRC( c, num );
}

/* zVec3DPCA_PL
 * - PCA against vector cloud given by a list.
 */
zVec3D *zVec3DPCA_PL(zVec3DList *vl, zVec3D evec[])
{
  zMat3D pv, vm;
  double eval[3];
  zVec3DListCell *vc;

  zMat3DClear( &vm );
  zListForEach( vl, vc ){
    zMat3DDyad( vc->data, vc->data, &pv );
    zMat3DAddDRC( &vm, &pv );
  }
  zMat3DSymEig( &vm, eval, evec );
  return evec;
}

/* zVec3DPCA
 * - PCA against vector cloud given by an array.
 */
zVec3D *zVec3DPCA(zVec3D v[], int num, zVec3D evec[])
{
  zMat3D pv, vm;
  double eval[3];
  register int i;

  zMat3DClear( &vm );
  for( i=0; i<num; i++ ){
    zMat3DDyad( &v[i], &v[i], &pv );
    zMat3DAddDRC( &vm, &pv );
  }
  zMat3DSymEig( &vm, eval, evec );
  return evec;
}

/* zVec3DBaryPCA_PL
 * - barycenter of and PCA against vector cloud given by a list.
 */
zVec3D *zVec3DBaryPCA_PL(zVec3DList *vl, zVec3D *c, zVec3D evec[])
{
  zMat3D pv, vm;
  double eval[3];
  zVec3DListCell *vc;
  zVec3D dp;

  zVec3DBarycenterPL( vl, c );
  zMat3DClear( &vm );
  zListForEach( vl, vc ){
    zVec3DSub( vc->data, c, &dp );
    zMat3DDyad( &dp, &dp, &pv );
    zMat3DAddDRC( &vm, &pv );
  }
  zMat3DSymEig( &vm, eval, evec );
  return c;
}

/* zVec3DBaryPCA
 * - barycenter of and PCA against vector cloud given by an array.
 */
zVec3D *zVec3DBaryPCA(zVec3D v[], int num, zVec3D *c, zVec3D evec[])
{
  zMat3D pv, vm;
  double eval[3];
  zVec3D dp;
  register int i;

  zVec3DBarycenter( v, num, c );
  zMat3DClear( &vm );
  for( i=0; i<num; i++ ){
    zVec3DSub( &v[i], c, &dp );
    zMat3DDyad( &dp, &dp, &pv );
    zMat3DAddDRC( &vm, &pv );
  }
  zMat3DSymEig( &vm, eval, evec );
  return c;
}
