/* Zeo - Z/Geometry and optics computation library.
 * Copyright (C) 2005 Tomomichi Sugihara (Zhidao)
 *
 * zeo_vec3d - 3D vector.
 */

#include <zeo/zeo_vec3d.h>

/* ********************************************************** */
/* CLASS: zVec3D
 * 3D vector class
 * ********************************************************** */

/* OBJECT:
 * zvec3Dzero, zvec3Dx, zvec3Dy, zvec3Dz
 * - 3D zero vector and unit vectors along (x,y,z) axis.
 */
const zVec3D zvec3Dzero = { { 0, 0, 0 } };
const zVec3D zvec3Dx    = { { 1, 0, 0 } };
const zVec3D zvec3Dy    = { { 0, 1, 0 } };
const zVec3D zvec3Dz    = { { 0, 0, 1 } };

/* zVec3DCreate
 * - create a 3D vector.
 */
zVec3D *zVec3DCreate(zVec3D *v, double x, double y, double z)
{
  v->e[zX] = x;
  v->e[zY] = y;
  v->e[zZ] = z;
  return v;
}

/* zVec3DCreatePolar
 * - create a 3D vector by the set of value for a polar
 *   expression.
 */
zVec3D *zVec3DCreatePolar(zVec3D *v, double r, double theta, double phi)
{
  double rs;

  rs = r * sin( theta );
  return zVec3DCreate( v, rs*cos(phi), rs*sin(phi), r*cos(theta) );
}

/* zVec3DMatch
 * - check if two 3D vectors match each other.
 */
bool zVec3DMatch(zVec3D *v1, zVec3D *v2)
{
  return v1->e[zX] == v2->e[zX] &&
         v1->e[zY] == v2->e[zY] &&
         v1->e[zZ] == v2->e[zZ];
}

/* zVec3DEqual
 * - check if two 3D vectors are equal.
 */
bool zVec3DEqual(zVec3D *v1, zVec3D *v2)
{
  return zIsTiny( v1->e[zX] - v2->e[zX] ) &&
         zIsTiny( v1->e[zY] - v2->e[zY] ) &&
         zIsTiny( v1->e[zZ] - v2->e[zZ] );
}

/* zVec3DIsTol
 * - check if a 3D vector is tiny enough.
 */
bool zVec3DIsTol(zVec3D *v, double tol)
{
  return zIsTol( v->e[zX], tol ) &&
         zIsTol( v->e[zY], tol ) &&
         zIsTol( v->e[zZ], tol );
}

/* zVec3DIsNan
 * - check if a 3D vector includes NaN or Inf components.
 */
bool zVec3DIsNan(zVec3D *v)
{
  return zIsNan( v->e[zX] ) || zIsInf( v->e[zX] ) ||
         zIsNan( v->e[zY] ) || zIsInf( v->e[zY] ) ||
         zIsNan( v->e[zZ] ) || zIsInf( v->e[zZ] );
}

/* ********************************************************** */
/* arithmetics
 * ********************************************************** */

/* zVec3DAdd
 * - add two 3D vectors.
 */
zVec3D *zVec3DAdd(zVec3D *v1, zVec3D *v2, zVec3D *v)
{
  v->e[zX] = v1->e[zX] + v2->e[zX];
  v->e[zY] = v1->e[zY] + v2->e[zY];
  v->e[zZ] = v1->e[zZ] + v2->e[zZ];
  return v;
}

/* zVec3DSub
 * - subtract a 3D vector from another.
 */
zVec3D *zVec3DSub(zVec3D *v1, zVec3D *v2, zVec3D *v)
{
  v->e[zX] = v1->e[zX] - v2->e[zX];
  v->e[zY] = v1->e[zY] - v2->e[zY];
  v->e[zZ] = v1->e[zZ] - v2->e[zZ];
  return v;
}

/* zVec3DRev
 * - reverse a 3D vector.
 */
zVec3D *zVec3DRev(zVec3D *v, zVec3D *rv)
{
  rv->e[zX] = -v->e[zX];
  rv->e[zY] = -v->e[zY];
  rv->e[zZ] = -v->e[zZ];
  return rv;
}

/* zVec3DMul
 * - multiply a 3D vector by a scalar.
 */
zVec3D *zVec3DMul(zVec3D *v, double k, zVec3D *mv)
{
  mv->e[zX] = k * v->e[zX];
  mv->e[zY] = k * v->e[zY];
  mv->e[zZ] = k * v->e[zZ];
  return mv;
}

/* zVec3DDiv
 * - divide a 3D vector by a scalar.
 */
zVec3D *zVec3DDiv(zVec3D *v, double k, zVec3D *dv)
{
  if( k == 0 ){
    ZRUNWARN( ZEO_ERR_ZERODIV );
    return NULL;
  }
  k = 1.0 / k;
  dv->e[zX] = k * v->e[zX];
  dv->e[zY] = k * v->e[zY];
  dv->e[zZ] = k * v->e[zZ];
  return dv;
}

/* zVec3DAmp
 * - amplify a 3D vector by another.
 */
zVec3D *zVec3DAmp(zVec3D *v, zVec3D *a, zVec3D *av)
{
  av->e[zX] = a->e[zX] * v->e[zX];
  av->e[zY] = a->e[zY] * v->e[zY];
  av->e[zZ] = a->e[zZ] * v->e[zZ];
  return av;
}

/* zVec3DCat
 * - concatenate 3D vectors.
 */
zVec3D *zVec3DCat(zVec3D *v1, double k, zVec3D *v2, zVec3D *v)
{
  v->e[zX] = v1->e[zX] + k * v2->e[zX];
  v->e[zY] = v1->e[zY] + k * v2->e[zY];
  v->e[zZ] = v1->e[zZ] + k * v2->e[zZ];
  return v;
}

/* zVec3DSqrNorm
 * - squared norm of a 3D vector.
 */
double zVec3DSqrNorm(zVec3D *v)
{
  return v->e[zX]*v->e[zX] + v->e[zY]*v->e[zY] + v->e[zZ]*v->e[zZ];
}

/* zVec3DWSqrNorm
 * - squared weighted norm of a 3D vector.
 */
double zVec3DWSqrNorm(zVec3D *v, zVec3D *w)
{
  return v->e[zX]*v->e[zX]*w->e[zX] + v->e[zY]*v->e[zY]*w->e[zY] + v->e[zZ]*v->e[zZ]*w->e[zZ];
}

/* zVec3DSqrDist
 * - distance between two positions.
 */
double zVec3DSqrDist(zVec3D *v1, zVec3D *v2)
{
  double dx, dy, dz;

  dx = v1->e[zX] - v2->e[zX];
  dy = v1->e[zY] - v2->e[zY];
  dz = v1->e[zZ] - v2->e[zZ];
  return dx*dx + dy*dy + dz*dz;
}

/* zVec3DNormalizeNC
 * - normalize a 3D vector without checking vector size.
 */
double zVec3DNormalizeNC(zVec3D *v, zVec3D *nv)
{
  double l, k;

  k = 1.0 / ( l = zVec3DNorm(v) );
  nv->e[zX] = k * v->e[zX];
  nv->e[zY] = k * v->e[zY];
  nv->e[zZ] = k * v->e[zZ];
  return l;
}

/* zVec3DNormalize
 * - normalize a 3D vector.
 */
double zVec3DNormalize(zVec3D *v, zVec3D *nv)
{
  double l, k;

  if( zVec3DIsTiny( v ) ){
    ZRUNWARN( ZEO_ERR_ZERONORM );
    return -1;
  }
  k = 1.0 / ( l = zVec3DNorm(v) );
  nv->e[zX] = k * v->e[zX];
  nv->e[zY] = k * v->e[zY];
  nv->e[zZ] = k * v->e[zZ];
  return l;
}

/* zVec3DInnerProd
 * - inner product of two 3D vectors.
 */
double zVec3DInnerProd(zVec3D *v1, zVec3D *v2)
{
  return v1->e[zX]*v2->e[zX] + v1->e[zY]*v2->e[zY] + v1->e[zZ]*v2->e[zZ];
}

/* zVec3DOuterProd
 * - outer product of two 3D vectors.
 */
zVec3D *zVec3DOuterProd(zVec3D *v1, zVec3D *v2, zVec3D *v)
{
  double x, y, z;

  /* v1 and v2 might be the same with v */
  x = v1->e[zY] * v2->e[zZ] - v1->e[zZ] * v2->e[zY];
  y = v1->e[zZ] * v2->e[zX] - v1->e[zX] * v2->e[zZ];
  z = v1->e[zX] * v2->e[zY] - v1->e[zY] * v2->e[zX];
  v->e[zX] = x;
  v->e[zY] = y;
  v->e[zZ] = z;
  return v;
}

/* zVec3DOuterProdNorm
 * - norm of outer product of two 3D vectors.
 */
double zVec3DOuterProdNorm(zVec3D *v1, zVec3D *v2)
{
  zVec3D v;

  return zVec3DNorm( zVec3DOuterProd( v1, v2, &v ) );
}

/* zVec3DGrassmannProd
 * - scalar triple product of three 3D vectors.
 */
double zVec3DGrassmannProd(zVec3D *v1, zVec3D *v2, zVec3D *v3)
{
  double x, y, z;

  /* v1 and v2 might be the same with v */
  x = v2->e[zY] * v3->e[zZ] - v2->e[zZ] * v3->e[zY];
  y = v2->e[zZ] * v3->e[zX] - v2->e[zX] * v3->e[zZ];
  z = v2->e[zX] * v3->e[zY] - v2->e[zY] * v3->e[zX];
  return v1->e[zX]*x + v1->e[zY]*y + v1->e[zZ]*z;
}

/* zVec3DTripleProd
 * - vector triple product of three 3D vectors.
 */
zVec3D *zVec3DTripleProd(zVec3D *v1, zVec3D *v2, zVec3D *v3, zVec3D *v)
{
  double d1, d2;

  d1 = v1->e[zX]*v3->e[zX] + v1->e[zY]*v3->e[zY] + v1->e[zZ]*v3->e[zZ];
  d2 = v1->e[zX]*v2->e[zX] + v1->e[zY]*v2->e[zY] + v1->e[zZ]*v2->e[zZ];
  v->e[zX] = v2->e[zX]*d1 - v3->e[zX]*d2;
  v->e[zY] = v2->e[zY]*d1 - v3->e[zY]*d2;
  v->e[zZ] = v2->e[zZ]*d1 - v3->e[zZ]*d2;
  return v;
}

/* ********************************************************** */
/* geometry
 * ********************************************************** */

/* zVec3DInterDiv
 * - interior division of two the positions.
 */
zVec3D *zVec3DInterDiv(zVec3D *v1, zVec3D *v2, double ratio, zVec3D *v)
{
  v->e[zX] = v1->e[zX] + ratio * ( v2->e[zX] - v1->e[zX] );
  v->e[zY] = v1->e[zY] + ratio * ( v2->e[zY] - v1->e[zY] );
  v->e[zZ] = v1->e[zZ] + ratio * ( v2->e[zZ] - v1->e[zZ] );
  return v;
}

/* zVec3DMid
 * - middle point of the two positions.
 */
zVec3D *zVec3DMid(zVec3D *v1, zVec3D *v2, zVec3D *v)
{
  v->e[zX] = 0.5 * ( v1->e[zX] + v2->e[zX] );
  v->e[zY] = 0.5 * ( v1->e[zY] + v2->e[zY] );
  v->e[zZ] = 0.5 * ( v1->e[zZ] + v2->e[zZ] );
  return v;
}

/* zVec3DAngle
 * - angle between the two vectors.
 */
double zVec3DAngle(zVec3D *v1, zVec3D *v2, zVec3D *n)
{
  double c, s;
  zVec3D d;

  c = zVec3DInnerProd( v1, v2 );
  s = zVec3DNorm( zVec3DOuterProd( v1, v2, &d ) );
  if( n && zVec3DInnerProd( &d, n ) < 0 ) s = -s;
  return atan2( s, c );
}

/* zVec3DProj
 * - projection of a vector.
 */
zVec3D *zVec3DProj(zVec3D *v, zVec3D *n, zVec3D *pv)
{
  double l;

  if( zVec3DIsTiny( n ) ){
    ZRUNWARN( ZEO_ERR_ZERONORM );
    return NULL;
  }
  l = ( n->e[zX]*v->e[zX] + n->e[zY]*v->e[zY] + n->e[zZ]*v->e[zZ] ) / ( n->e[zX]*n->e[zX] + n->e[zY]*n->e[zY] + n->e[zZ]*n->e[zZ] );
  pv->e[zX] = l * n->e[zX];
  pv->e[zY] = l * n->e[zY];
  pv->e[zZ] = l * n->e[zZ];
  return pv;
}

/* zVec3DOrthogonalize
 * - orthogonalize a vector.
 */
zVec3D *zVec3DOrthogonalize(zVec3D *v, zVec3D *n, zVec3D *ov)
{
  double l;

  if( zVec3DIsTiny( n ) ){
    ZRUNWARN( ZEO_ERR_ZERONORM );
    return NULL;
  }
  l = ( n->e[zX]*v->e[zX] + n->e[zY]*v->e[zY] + n->e[zZ]*v->e[zZ] ) / ( n->e[zX]*n->e[zX] + n->e[zY]*n->e[zY] + n->e[zZ]*n->e[zZ] );
  ov->e[zX] = v->e[zX] - l * n->e[zX];
  ov->e[zY] = v->e[zY] - l * n->e[zY];
  ov->e[zZ] = v->e[zZ] - l * n->e[zZ];
  return ov;
}

/* zVec3DOrthoSpace
 * - create the orthogonal space of a vector.
 */
bool zVec3DOrthoSpace(zVec3D *v, zVec3D *sv1, zVec3D *sv2)
{
  double l;

  zVec3DCopy( v, sv1 );
  if( v->e[zY] != 0 || v->e[zZ] != 0 )
    sv1->e[zX] += 1.0;
  else
    sv1->e[zY] = 1.0;
  if( zVec3DIsTiny( v ) ){
    ZRUNWARN( ZEO_ERR_ZERONORM );
    return false;
  }
  l = ( v->e[zX]*sv1->e[zX] + v->e[zY]*sv1->e[zY] + v->e[zZ]*sv1->e[zZ] ) / ( v->e[zX]*v->e[zX] + v->e[zY]*v->e[zY] + v->e[zZ]*v->e[zZ] );
  sv1->e[zX] -= l * v->e[zX];
  sv1->e[zY] -= l * v->e[zY];
  sv1->e[zZ] -= l * v->e[zZ];
  sv2->e[zX] = v->e[zY] * sv1->e[zZ] - v->e[zZ] * sv1->e[zY];
  sv2->e[zY] = v->e[zZ] * sv1->e[zX] - v->e[zX] * sv1->e[zZ];
  sv2->e[zZ] = v->e[zX] * sv1->e[zY] - v->e[zY] * sv1->e[zX];
  return true;
}

/* zVec3DRot
 * - rotate a 3D vector along an arbitrary axis.
 */
zVec3D *zVec3DRot(zVec3D *v, zVec3D *aa, zVec3D *rv)
{
  zVec3D v0, v1, v2, n;
  double angle;

  if( zVec3DIsTiny( aa ) ){
    zVec3DCopy( v, rv );
    return rv;
  }
  angle = zVec3DNormalize( aa, &n );
  zVec3DProj( v, &n, &v0 );
  zVec3DSub( v, &v0, &v1 );
  zVec3DOuterProd( &n, v, &v2 );
  zVec3DCat( &v0, cos(angle), &v1, rv );
  return zVec3DCatDRC( rv, sin(angle), &v2 );
}

/* ********************************************************** */
/* differential kinematics
 * ********************************************************** */

/* zVec3DDif
 * - numerical differentiation of 3D vector.
 */
zVec3D *zVec3DDif(zVec3D *v, zVec3D *vnew, double dt, zVec3D *vel)
{
  zVec3DSub( vnew, v, vel );
  return zVec3DDivDRC( vel, dt );
}

/* zVec3DZYXVel2AngVel
 * - convert z-y-x Eulerian angle differential to angular velocity.
 */
zVec3D *zVec3DZYXVel2AngVel(zVec3D *zyxvel, zVec3D *zyx, zVec3D *angvel)
{
  double sa, ca, sb, cb;

  zSinCos( zyx->e[0], &sa, &ca );
  zSinCos( zyx->e[1], &sb, &cb );
  return zVec3DZYXVel2AngVelSC( zyxvel, sa, ca, sb, cb, angvel );
}

/* zVec3DZYXVel2AngVelSC
 * - convert z-y-x Eulerian angle differential to angular velocity,
 *   directly accepting sine/cosine sets.
 */
zVec3D *zVec3DZYXVel2AngVelSC(zVec3D *zyxvel, double sa, double ca, double sb, double cb, zVec3D *angvel)
{
  return zVec3DCreate( angvel,
   -sa*zyxvel->e[zY] + ca*cb*zyxvel->e[zZ],
    ca*zyxvel->e[zY] + sa*cb*zyxvel->e[zZ],
       zyxvel->e[zX] -    sb*zyxvel->e[zZ] );
}

/* zVec3DAngVel2ZYXVel
 * - convert angular velocity to z-y-x Eulerian angle differential.
 */
zVec3D *zVec3DAngVel2ZYXVel(zVec3D *angvel, zVec3D *zyx, zVec3D *zyxvel)
{
  double sa, ca, sb, cb;

  zSinCos( zyx->e[0], &sa, &ca );
  zSinCos( zyx->e[1], &sb, &cb );
  return zVec3DAngVel2ZYXVelSC( angvel, sa, ca, sb, cb, zyxvel );
}

/* zVec3DAngVel2ZYXVelSC
 * - convert angular velocity to z-y-x Eulerian angle differential.
 *   directly accepting sine/cosine sets.
 */
zVec3D *zVec3DAngVel2ZYXVelSC(zVec3D *angvel, double sa, double ca, double sb, double cb, zVec3D *zyxvel)
{
  /* at the singular point, 'zyxvel' remains the same value. */
  if( zIsTiny(cb) ) return zyxvel;
  zyxvel->e[zZ] = ca/cb*angvel->e[zX]+sa/cb*angvel->e[zY];
  zyxvel->e[zX] = sb*zyxvel->e[zZ]+angvel->e[zZ];
  zyxvel->e[zY] =-sa*angvel->e[zX]+ca*angvel->e[zY];
  return zyxvel;
}

/* zVec3DZYZVel2AngVel
 * - convert z-y-z Eulerian angle differential to angular velocity.
 */
zVec3D *zVec3DZYZVel2AngVel(zVec3D *zyzvel, zVec3D *zyz, zVec3D *angvel)
{
  double sa, ca, sb, cb;

  zSinCos( zyz->e[0], &sa, &ca );
  zSinCos( zyz->e[1], &sb, &cb );
  return zVec3DZYZVel2AngVelSC( zyzvel, sa, ca, sb, cb, angvel );
}

/* zVec3DZYZVel2AngVelSC
 * - convert z-y-z Eulerian angle differential to angular velocity.
 *   directly accepting sine/cosine sets.
 */
zVec3D *zVec3DZYZVel2AngVelSC(zVec3D *zyzvel, double sa, double ca, double sb, double cb, zVec3D *angvel)
{
  return zVec3DCreate( angvel,
   -sa*zyzvel->e[zY] + ca*sb*zyzvel->e[zZ],
    ca*zyzvel->e[zY] + sa*sb*zyzvel->e[zZ],
       zyzvel->e[zX] +    cb*zyzvel->e[zZ] );
}

/* zVec3DAngVel2ZYZVel
 * - convert angular velocity to z-y-z Eulerian angle differential.
 */
zVec3D *zVec3DAngVel2ZYZVel(zVec3D *angvel, zVec3D *zyz, zVec3D *zyzvel)
{
  double sa, ca, sb, cb;

  zSinCos( zyz->e[0], &sa, &ca );
  zSinCos( zyz->e[1], &sb, &cb );
  return zVec3DAngVel2ZYZVelSC( angvel, sa, ca, sb, cb, zyzvel );
}

/* zVec3DAngVel2ZYZVelSC
 * - convert angular velocity to z-y-z Eulerian angle differential.
 *   directly accepting sine/cosine sets.
 */
zVec3D *zVec3DAngVel2ZYZVelSC(zVec3D *angvel, double sa, double ca, double sb, double cb, zVec3D *zyzvel)
{
  /* at the singular point, 'zyzvel' remains the same value. */
  if( zIsTiny(sb) ) return zyzvel;
  zyzvel->e[zZ] = ca/sb*angvel->e[zX]+sa/sb*angvel->e[zY];
  zyzvel->e[zX] =-cb*zyzvel->e[zZ]+angvel->e[zZ];
  zyzvel->e[zY] =-sa*angvel->e[zX]+ca*angvel->e[zY];
  return zyzvel;
}

/* ********************************************************** */
/* I/O
 * ********************************************************** */

/* zVec3DFRead
 * - input a 3D vector from file.
 */
zVec3D *zVec3DFRead(FILE *fp, zVec3D *v)
{
  v->e[zX] = zFDouble( fp );
  v->e[zY] = zFDouble( fp );
  v->e[zZ] = zFDouble( fp );
  return v;
}

/* zVec3DDataFWrite
 * - output a 3D vector to file.
 */
zVec3D *zVec3DDataFWrite(FILE *fp, zVec3D *v)
{
  if( !v ) return NULL;
  fprintf( fp, " %.10g %.10g %.10g", v->e[zX], v->e[zY], v->e[zZ] );
  return v;
}

/* zVec3DDataNLFWrite
 * - output a 3D vector to file with the new line.
 */
zVec3D *zVec3DDataNLFWrite(FILE *fp, zVec3D *v)
{
  if( !zVec3DDataFWrite( fp, v ) ) return NULL;
  fprintf( fp, "\n" );
  return v;
}

/* zVec3DFWrite
 * - output of 3D vector to file.
 */
zVec3D *zVec3DFWrite(FILE *fp, zVec3D *v)
{
  fprintf( fp, "(" );
  if( !v )
    fprintf( fp, "null 3D vector" );
  else
    zVec3DDataFWrite( fp, v );
  fprintf( fp, ")\n" );
  return v;
}

/* METHOD:
 * zVec3DFWriteXML - xml output.
 * ... yet testing.
 */
void zVec3DFWriteXML(FILE *fp, zVec3D *v)
{
  fprintf( fp, "\"" );
  zVec3DDataFWrite( fp, v );
  fprintf( fp, "\"" );
}
