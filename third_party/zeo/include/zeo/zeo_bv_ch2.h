/* Zeo - Z/Geometry and optics computation library.
 * Copyright (C) 2005 Tomomichi Sugihara (Zhidao)
 *
 * zeo_bv_ch2 - bounding volume: planar convex hull.
 */

#ifndef __ZEO_BV_CH2_H__
#define __ZEO_BV_CH2_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

/* ********************************************************** */
/* planar (2D) convex hull
 * ********************************************************** */

/* METHOD:
 * zCH2DPL, zCH2D, zCH2DPH
 * - a planar convex hull.
 *
 * 'zCH2D()' creates a planar convex hull of a set of
 * points 'vert', which are placed on a plane.
 * 'n' is the number of points.
 * The convex hull is represented as a list of
 * pointers to the vertices in 'vert'.
 * #
 * 'zCH3DPL()' also computes a planar convex hull of
 * the set of points given as a vector list 'pl'.
 * #
 * For these functions, it is supposed that the
 * vertices are located on a common plane.
 * #
 * 'zCH2D2PH3D()', zCH2DPL2PH3D create a planar convex
 * hull as a polyhedron 'ch' with dual-face triangles.
 * For each function, the set of points is given as
 * an array and a list of vertices, respectively.
 * [RETURN VALUE]
 * 'zCH2D()' and 'zCH2DPL()' return a pointer 'list',
 * if succeed. When it fails to allocate cells
 * dynamically, they return the null pointer.
 * #
 * 'zCH2D2PH3D()' and 'zCH2DPL2PH3D()' return a pointer
 * to 'ch' created. If they fail to create it, the
 * null pointer is returned.
 */
__EXPORT zVec3DList *zCH2D(zVec3DList *list, zVec3D vert[], int n);
__EXPORT zVec3DList *zCH2DPL(zVec3DList *list, zVec3DList *pl);
__EXPORT zPH3D *zCH2D2PH3D(zPH3D *ch, zVec3D vert[], int n);
__EXPORT zPH3D *zCH2DPL2PH3D(zPH3D *ch, zVec3DList *pl);

/* METHOD:
 * zCH2DClosest - the closest point in a convex hull to a point.
 *
 * 'zCH2DClosest()' finds the closest point in a convex hull
 * 'ch' to a point 'p'. 'ch' is represented by a list of
 * vertices. The result is stored where 'cp' points.
 * If 'p' is inside of 'ch', 'p' is copied to 'cp'.
 * If 'ch' is a non-convex hull, this function does not work
 * validly.
 * [RETURN VALUE]
 * 'zCH2DClosest()' returns the distance from 'p' to 'ch'.
 * If 'p' is inside of 'ch', the value returned is zero.
 * [NOTES]
 * If 'ch' is a non-convex hull, anything might happen.
 */
__EXPORT double zCH2DClosest(zVec3DList *ch, zVec3D *p, zVec3D *cp);

__END_DECLS

#endif /* __ZEO_BV_CH2_H__ */
