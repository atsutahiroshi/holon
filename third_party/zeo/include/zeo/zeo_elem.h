/* Zeo - Z/Geometry and optics computation library.
 * Copyright (C) 2005 Tomomichi Sugihara (Zhidao)
 *
 * zeo_elem - 3D shape elements.
 */

#ifndef __ZEO_ELEM_H__
#define __ZEO_ELEM_H__

#include <zeo/zeo_mat2d.h>
#include <zeo/zeo_frame.h>

__BEGIN_DECLS

/* ********************************************************** */
/* CLASS: zPlane3D
 * 3D plane class
 * ********************************************************** */

typedef struct{
  zVec3D *vert, norm;
} zPlane3D;

#define zPlane3DVert(p)      ( (p)->vert )
#define zPlane3DNorm(p)      ( &(p)->norm )

#define zPlane3DSetVert(p,v) ( zPlane3DVert(p) = (v) )
#define zPlane3DSetNorm(p,n) zVec3DNormalize( (n), zPlane3DNorm(p) )

/* METHOD:
 * zPlane3DInit, zPlane3DCreate
 * - initialization and creation of a plane.
 *
 * 'zPlane3DInit()' initializes a plane instance 'p' as
 * the x-y plane.
 *
 * 'zPlane3DCreate()' creates a plane instance which is
 * defined by the pass point 'v' and the normal vector 'n'.
 * 'p' is a pointer to the plane.
 *
 * Due to the implementation, the normalized vector of 'n'
 * is copied to the internal vector of 'p', while 'v' is
 * directly pointed by the internal pointer of 'p'.
 * [RETURN VALUE]
 * Each of 'zPlane3DInit()' and 'zPlane3DCreate()' returns
 * a pointer to 'p'.
 */
__EXPORT zPlane3D *zPlane3DCreate(zPlane3D *p, zVec3D *v, zVec3D *n);
#define zPlane3DInit(p) zPlane3DCreate( (p), ZVEC3DZERO, ZVEC3DZ )

/* METHOD:
 * zPlane3DPointDist, zPlane3DProj
 * - distance between point and plane, and projection of
 *   point to 3D plane.
 *
 * 'zPlane3DPointDist()' calculates the distance between
 * the point 'v' and the plane 'p'.
 *
 * 'zPlane3DProj()' calculates the projection point of
 * the point 'v' to the plane 'p'. The result is put into
 * the vector pointed by 'cp'.
 * [RETURN VALUE]
 * 'zPlane3DPointDist()' returns the distance calculated.
 *
 * 'zPlane3DProj()' also returns the distance between
 * 'p' and 'v', which is obtained on the process of
 * calculation of the projection.
 */
__EXPORT double zPlane3DPointDist(zPlane3D *p, zVec3D *v);
__EXPORT double zPlane3DProj(zPlane3D *p, zVec3D *v, zVec3D *cp);

/* METHOD:
 * zPlane3DMean
 * - mean plane of set of points.
 *
 * 'zPlane3DMean()' calculates the mean plane 'pl' of
 * a set of points 'v'. 'n' is the number of points.
 * Namely, 'pl' is the plane to which the mean of
 * distance of every points is minimized.
 *
 * 'pc' is the mean point of 'v', which is also on 'pl'.
 * [RETURN VALUE]
 * 'zPlane3DMean()' returns a pointer 'pl'.
 */
__EXPORT zPlane3D *zPlane3DMean(zPlane3D *pl, zVec3D *pc, zVec3D v[], int n);

/* METHOD:
 * zPlane3DFWrite, zPlane3DWrite - output of 3D plane.
 *
 * 'zPlane3DFWrite()' writes the information of a
 * plane 'p' to the current position of the file 'fp'
 * in the following format:
 *
 *  vert { <x> <y> <z> } <- the passing point
 *  norm { <x> <y> <z> } <- the normal vector
 *
 * 'zPlane3DWrite()' writes the information of the
 * plane 'p' to the standard out.
 * [RETURN VALUE]
 * Neither 'zPlane3DFWrite()' nor 'zPlane3DWrite()'
 * return any values.
 */
__EXPORT void zPlane3DFWrite(FILE *fp, zPlane3D *p);
#define zPlane3DWrite(p) zPlane3DFWrite( stdout, (p) )

/* ********************************************************** */
/* CLASS: zEdge3D
 * 3D edge class
 * ********************************************************** */

typedef struct{
  zVec3D *vert[2], vec;
} zEdge3D;

#define zEdge3DVert(e,i)      (e)->vert[(i)]
#define zEdge3DVec(e)         ( &(e)->vec )

#define zEdge3DSetVert(e,i,p) ( (e)->vert[(i)] = (p) )
#define zEdge3DSetVec(e,v)    zVec3DCopy( v, zEdge3DVec(e) )

/* METHOD:
 * zEdge3DInit, zEdge3DCreate
 * - initialization and creation of a 3D edge.
 *
 * 'zEdge3DInit()' initializes a edge instance 'e', setting
 * both endpoints for null vectors.
 *
 * 'zEdge3DCreate()' creates a edge from two endpoints 'v1'
 * and 'v2'.
 * [RETURN VALUE]
 * Each of 'zEdge3DInit()' and 'zEdge3DCreate()' returns
 * a pointer to 'e'.
 */
__EXPORT zEdge3D *zEdge3DInit(zEdge3D *e);
__EXPORT zEdge3D *zEdge3DCreate(zEdge3D *e, zVec3D *v1, zVec3D *v2);

/* METHOD:
 * zEdge3DCalcVec - path vector of 3D edge.
 *
 * 'zEdge3DCalcVec()' calculates the path vector from the
 * first endpoint to the second of the edge 'e'. The vector
 * is contained by the edge itself within. One can access
 * the path vector by calling zEdge3DVec(e) (see 'zshape.h').
 * [RETURN VALUE]
 * 'zEdge3DCalcVec()' returns the pointer to the path vector.
 */
__EXPORT zVec3D *zEdge3DCalcVec(zEdge3D *e);

/* METHOD:
 * zEdge3DPointDist, zEdge3DClosest
 * - distance between point and 3D edge.
 *
 * 'zEdge3DPointDist()' calculates a distance between
 * an edge 'e' and a point 'p'. In this operation,
 * 'e' is regarded as an infinite-length edge, namely,
 * it returns the length of perpendicular from 'p' to 'e'.
 *
 * 'zEdge3DClosest()', on the contrary, calculates
 * the actual closest point on 'e' from 'p' and sets it
 * into 'cp'. When the perpendicular from 'p' to 'e'
 * does not cross with 'e', it returns the closest endpoint
 * of 'e' from 'p'.
 * [RETURN VALUE]
 * 'zEdge3DPointDist()' returns the distance calculated
 * -- the length of perpendicular from 'p' to 'e'.
 *
 * 'zEdge3DClosest()' returns the distance between
 * 'p' and 'cp'.
 */
__EXPORT zVec3D *zEdge3DProj(zEdge3D *e, zVec3D *p, zVec3D *cp);
__EXPORT double zEdge3DPointDist(zEdge3D *e, zVec3D *p);
__EXPORT bool zEdge3DPointIsOn(zEdge3D *e, zVec3D *p);
__EXPORT double zEdge3DLinScale(zEdge3D *e, zVec3D *p, double *l0, double *l1, zVec3D *cp);
__EXPORT double zEdge3DClosest(zEdge3D *e, zVec3D *p, zVec3D *cp);

/* METHOD:
 * zEdge3DContigVert - contiguous vertix of edge to a point.
 *
 * 'zEdge3DContigVert()' returns the contiguous vertix of
 * edge 'e' to the given point 'p'(namely, the value returned
 * has to be either the first or the second vertix of 'e').
 * [RETURN VALUE]
 * 'zEdge3DContigVert()' returns a pointer to the vertix found.
 */
__EXPORT zVec3D *zEdge3DContigVert(zEdge3D *e, zVec3D *p, double *d);

/* METHOD:
 * zEdge3DFWrite, zEdge3DWrite - output of 3D edge.
 *
 * 'zEdge3DFWrite()' writes the coordinates of each
 * endpoint to the current position of the file 'fp',
 * according to the following format.
 *
 *  vert 0: <x0> <y0> <z0>
 *  vert 1: <x1> <y1> <z1>
 *  vec: <vx> <vy> <vz>
 *
 * where vert 0 is the first endpoint, vert 1 is the
 * second and vec is for the path vector.
 * 'zEdge3DVertWrite()' writes the same information
 * of 'e' to the standard output.
 * [RETURN VALUE]
 * 'zEdge3DFWrite()' and 'zEdge3DWrite()' return
 * no values.
 */
__EXPORT void zEdge3DFWrite(FILE *fp, zEdge3D *e);
#define zEdge3DWrite(e) zEdge3DFWrite( stdout, (e) )

/* ********************************************************** */
/* CLASS: zTri3D
 * 3D triangle class
 * ********************************************************** */

typedef struct{
  zVec3D *v[3], norm;
} zTri3D;

#define zTri3DNorm(t)        ( &(t)->norm )
#define zTri3DVert(t,i)      (t)->v[i]
#define zTri3DVertNext(t,i)  zTri3DVert( t, (i)==2 ? 0 : (i)+1 )

#define zTri3DSetNorm(t,n)   zVec3DCopy( n, zTri3DNorm(t) )
#define zTri3DSetVert(t,i,v) ( zTri3DVert(t,i) = (v) )

/* METHOD:
 * zTri3DInit, zTri3DCreate, zTri3DCreateRev,
 * zTri3DRev, zTri3DRevDRC
 * - initialize and create 3D triangle.
 *
 * 'zTri3DInit()' initializes a triangle instance 't',
 * setting three vertices for the null vector.
 *
 * 'zTri3DCreate()' creates a triangle from three vertices
 * 'v1', 'v2' and 'v3'.
 * 'zTri3DCreateRev()' creates a triangle from 'v1', 'v2'
 * and 'v3' in reversed order of 'zTri3DCreate()', i.e.
 * it is equivalent to 'zTri3DCreate( v1, v3, v2 )'.
 *
 * 'zTri3DRev()' reverses the source triangle 'src',
 * and create the destination triangle 'dest'. It is
 * permitted to let 'dest' point to the same address
 * with 'src'. Actually, 'zTri3DRevDRC()', which is
 * the destructive version of 'zTri3DRev()' is defined
 * as so (see 'zeo_elem.h').
 * [RETURN VALUE]
 * 'zTri3DInit()', 'zTri3DCreate()', 'zTri3DCreateRev()'
 * and 'zTri3DRevDRC()' return a pointer 't'.
 *
 * 'zTri3DRev()' returns a pointer 'dest'.
 */
__EXPORT zTri3D *zTri3DInit(zTri3D *t);
__EXPORT zTri3D *zTri3DCreate(zTri3D *t, zVec3D *v1, zVec3D *v2, zVec3D *v3);
__EXPORT zTri3D *zTri3DCreateRev(zTri3D *t, zVec3D *v1, zVec3D *v2, zVec3D *v3);
__EXPORT zTri3D *zTri3DRev(zTri3D *src, zTri3D *dest);
#define zTri3DRevDRC(t) zTri3DRev( t, t )

/* METHOD:
 * zTri3DArea, zTri3DCalcNorm, zTri3DBarycenter,
 * zTri3DCircumcenter, zTri3DIncenter,
 * zTri3DOrthocenter
 * - area, normal vector and various centers of 3D triangle.
 *
 * 'zTri3DArea()' calculates the area of a triangle
 * 't'.
 *
 * 'zTri3DCalcNorm()' calculates the normal vector of
 * triangle 't'. The vector is contained by the triangle
 * itself within. One can access the normal vector by
 * calling zTri3DNorm(p) (see 'zeo_elem.h').
 *
 * 'zTri3DBarycenter()', 'zTri3DCircumcenter()',
 * 'zTri3DIncenter()' and 'zTri3DOrthocenter()'
 * calculate barycenter, circumcenter, incenter
 * and orthocenter of 't', respectively.
 * The result is put into 'c'.
 * [RETURN VALUE]
 * 'zTri3DArea()' returns the area calculated.
 * 'zTri3DCalcNorm()' returns a pointer to the normal vector.
 * 'zTri3DBarycenter()', 'zTri3DCircumcenter()',
 * 'zTri3DIncenter()' and 'zTri3DOrthocenter()'
 * return a pointer to 'c'.
 */
__EXPORT double zTri3DArea(zTri3D *t);
__EXPORT zVec3D *zTri3DCalcNorm(zTri3D *t);
__EXPORT zVec3D *zTri3DBarycenter(zTri3D *t, zVec3D *c);
__EXPORT zVec3D *zTri3DCircumcenter(zTri3D *t, zVec3D *c);
__EXPORT zVec3D *zTri3DIncenter(zTri3D *t, zVec3D *c);
__EXPORT zVec3D *zTri3DOrthocenter(zTri3D *t, zVec3D *c);

/* METHOD:
 * zTri3DToPlane3D
 * - convert a triangle to a plane.
 *
 * 'zTri3DToPlane3D()' converts a given 3D triangle
 * 't' to an infinite 3D plane 'p' which 't' is on.
 * [RETURN VALUE]
 * 'zTri3DToPlane3D()' returns a pointer to 't'.
 */
#define zTri3DToPlane3D(t,p) zPlane3DCreate( p, zTri3DVert(t,0), zTri3DNorm(t) )

/* METHOD:
 * zTri3DContigVert - contiguous vertix of triangle to a point.
 *
 * 'zTri3DContigVert()' returns the contiguous vertix of
 * triangle 't' to the given point 'p' (namely, the value
 * returned has to be any of three vertices of 't').
 * [RETURN VALUE]
 * 'zTri3DContigVert()' returns a pointer to the vertix found.
 */
__EXPORT zVec3D *zTri3DContigVert(zTri3D *t, zVec3D *p, double *d);

/* METHOD:
 * zTri3DPointDist, zTri3DProj
 * - distance from a point to a triangle.
 *
 * 'zTri3DPointDist()' calculates the distance
 * between a triangle 't' and a point 'v'.
 * The result is
 *  - a positive value when 'v' is above 't'('v' exists in
 *    the direction of the normal vector of 't'),
 *  - a negative value when 'v' is below 't'('v' exists in
 *    the opposite direction of the normal vector of 't'),
 *  - zero when 'v' is on 't'.
 * The funciton does not care if the projection point of
 * 'v' is inside of 't' or not, just calculating the length
 * of perpendicular from 'v' to 't'.
 *
 * 'zTri3DProj()' calculates the projection point of 'v'
 * into a triangle 't' - the footpoint of perpendicular
 * from 'v' to 't', and sets it into 'cp'.
 * [RETURN VALUE]
 * 'zTri3DPointDist()' and 'zTri3DProj()' return
 * the distance from 'v' to 't'.
 */
__EXPORT double zTri3DPointDist(zTri3D *t, zVec3D *v);
__EXPORT bool zTri3DPointIsOn(zTri3D *t, zVec3D *v);
__EXPORT double zTri3DProj(zTri3D *t, zVec3D *v, zVec3D *cp);

/* METHOD:
 * zTri3DPointIsInside
 * - check if a point is inside of a triangle.
 *
 * 'zTri3DPointIsInside()' checks if the given point 'v'
 * is inside of a triangle 't'. "'v' is inside of 'p'"
 * means that the projection point of 'v' to 't' is inside
 * of 't'.
 *
 * If the true value is given for 'rim', 'v' on the edge
 * or the corner of 't' is judged to be inside of 't'.
 * [RETURN VALUE]
 * 'zTri3DPointIsInside()' returns the true value if
 * 'v' is inside of 't', or the false value otherwise.
 */
__EXPORT bool zTri3DPointIsInside(zTri3D *t, zVec3D *v, bool rim);

/* METHOD:
 * zTri3DClosest
 * - the closest point from a point to 3D triangle.
 *
 * 'zTri3DClosest()' calculates the closest point
 * on a triangle 't' from the given point 'v'.
 * It sets the point calculated into 'cp'.
 * [RETURN VALUE]
 * 'zTri3DClosest()' returns the distance from
 * 'v' to the closest point calculated.
 */
__EXPORT double zTri3DLinScale(zTri3D *t, zVec3D *p, double *l0, double *l1, double *l2, zVec3D *cp);
__EXPORT double zTri3DClosest(zTri3D *t, zVec3D *v, zVec3D *cp);

/* METHOD:
 * zTri3DConeVolume, zTri3DConeInertia,
 * zTri3DConeBarycenter, zTri3DConeCircumcenter
 * - volume, inertia, barycenter and circumcenter of cone.
 *
 * 'zTri3DConeVolume()' calculates a volume of
 * the cone which consists of a triangle 't' as
 * the base and a vector 'v' as the vertex.
 *
 * 'zTri3DConeInertia()' calculates the inertia
 * tensor of the cone which consists of 't' and the
 * original point and sets it into a 3x3 matrix 'i'.
 *
 * 'zTri3DConeBarycenter()' calculates the barycenter
 * of the cone which consists of 't' and 'v' and
 * sets it into 'c'.
 *
 * 'zTri3DConeCircumcenter()' calculates the
 * circumcenter of the cone which consists of 't'
 * and the original point and puts it into 'c'.
 * [RETURN VALUE]
 * 'zTri3DConeVolume()' returns the volum calculated.
 *
 * 'zTri3DConeInertia()' returns a pointer to 'i'.
 *
 * 'zTri3DConeBarycenter()' returns a pointer to 'c'.
 *
 * 'zTri3DConeCircumcenter()' returns a pointer
 * to 'c'.
 */
__EXPORT double zTri3DConeVolume(zTri3D *t, zVec3D *v);
__EXPORT zMat3D *zTri3DConeInertia(zTri3D *t, zMat3D *i);
__EXPORT zVec3D *zTri3DConeBarycenter(zTri3D *t, zVec3D *v, zVec3D *c);
__EXPORT zVec3D *zTri3DConeCircumcenter(zTri3D *t, zVec3D *c);

/* METHOD:
 * zTri3DFWrite, zTri3DWrite - output of 3D triangle.
 *
 * 'zTri3DFWrite()' writes the coordinates of each
 * vertex of a triangle 't' to the current position
 * of the file 'fp', according to the following format.
 *
 *  vert 0: <x0> <y0> <z0>
 *  vert 1: <x1> <y1> <z1>
 *  vert 2: <x2> <y2> <z2>
 *  norm: <nx> <ny> <nz>
 *
 * where vert 0 is the first vertex, vert 1 is the second,
 * vert 2 is the third and norm is for the normal vector.
 * 'zTri3DWrite()' writes the same information of
 * 't' to the standard out.
 * [RETURN VALUE]
 * None of 'zTri3DFWrite()', 'zTri3DWrite()',
 * 'zTri3DVertFWrite()' and 'zTri3DVertWrite()'
 * returns any values.
 */
__EXPORT void zTri3DFWrite(FILE *fp, zTri3D *t);
#define zTri3DWrite(t) zTri3DFWrite( stdout, (t) )

__END_DECLS

#include <zeo/zeo_elem_list.h> /* 3D shape element list */

#endif /* __ZEO_ELEM_H__ */
