/* Zeo - Z/Geometry and optics computation library.
 * Copyright (C) 2005 Tomomichi Sugihara (Zhidao)
 *
 * zeo_triangulate - trianglation of non-convex.
 */

#ifndef __ZEO_TRIANGULATE_H__
#define __ZEO_TRIANGULATE_H__

#include <zeo/zeo_elem.h>

__BEGIN_DECLS

/* METHOD:
 * zTriangulate - trianglate a non-convex.
 * [SYNOPSIS]
 * int zTriangulate(zVec3D v[], int n, zTri3DList *tlist);
 * [DESCRIPTION]
 * 'zTriangulate()' triangulates a non-convex, namely,
 * divides a non-convex into triangle pieces.
 * The non-convex forms as a loop of vertices given
 * by an array of vertices 'v'. Namely:
 *  v[0]-v[1]-...-v[n-1]-v[0]
 * where 'n' is the number of vectors.
 * As the result, a triangle list 'tlist' is newly
 * created.
 * [NOTES]
 * When freeing 'tlist', give the true value for the
 * third argument of 'zTri3DListDestroy()'.
 * [RETURN VALUE]
 * 'zTriangulate()' returns the number of triangles
 * generated, which is up to 'n'-2.
 */
__EXPORT int zTriangulate(zVec3D v[], int n, zTri3DList *tlist);

__END_DECLS

#endif /* __ZEO_TRIANGULATE_H__ */
