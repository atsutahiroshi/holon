/* Zeo - Z/Geometry and optics computation library.
 * Copyright (C) 2005 Tomomichi Sugihara (Zhidao)
 *
 * zeo_elem_list - 3D shape element list.
 */

#ifndef __ZEO_ELEM_LIST_H__
#define __ZEO_ELEM_LIST_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

/* ********************************************************** */
/* CLASS: zTri3DList
 * 3D triangle list
 * ********************************************************** */

zListClass( zTri3DList, zTri3DListCell, zTri3D* );

/* METHOD:
 * zTri3DListInsert, zTri3DListCreate, zTri3DListDestroy
 * - insert and destroy 3D triangle list.
 * [SYNOPSIS]
 * zTri3DListCell *zTri3DListInsert(zTri3DList *list, zTri3D *t, bool flag);
 * void zTri3DListDestroy(zTri3DList *list, bool flag);
 * [DESCRIPTION]
 * 'zTri3DListInsert()' inserts a new 3D triangle list
 * cell 't' at the head of a triangle list 'list'.
 * Each cell inserted will have a copy of 't' which is
 * newly allocated if 'flag' is the true value.
 * Otherwise, the cell has a pointer to 't'.
 * #
 * 'zTri3DListDestroy()' destroys 'list', freeing all
 * cells. If 'flag' is the true value, it also frees
 * the data held by each cell. Otherwise, only the cells
 * are freed.
 * [RETURN VALUE]
 * 'zTri3DListInsert()' returns a pointer to the cell
 * newly inserted.
 * 'zTri3DListDestroy()' returns no value.
 */
__EXPORT zTri3DListCell *zTri3DListInsert(zTri3DList *list, zTri3D *t, bool flag);
__EXPORT void zTri3DListDestroy(zTri3DList *list, bool flag);

/* METHOD:
 * zTri3DListAlign
 * - align triangles to a direction referred by a vector.
 * [SYNOPSIS]
 * void zTri3DListAlign(zTri3DList *list, zVec3D *ref);
 * [DESCRIPTION]
 * 'zTri3DListAlign()' aligns directions of all triangle
 * contained by a triangle list 'list' to one referred
 * by a vector 'ref'. Namely, if the direction of a
 * triangle in 'list' is opposite to 'ref', the triangle
 * is flipped.
 * [RETURN VALUE]
 * 'zTri3DListAlign()' returns no value.
 */
__EXPORT void zTri3DListAlign(zTri3DList *list, zVec3D *ref);

/* METHOD:
 * zTri3DListCopyArray - copy triangles in a list to array.
 * [SYNOPSIS]
 * void zTri3DListCopyArray(zTri3DList *list, zTri3D t[], int n);
 * [DESCRIPTION]
 * 'zTri3DListCopyArray()' copies triangles held by a
 * list 'list' to an array 't'. 'n' is the size of the
 * array.
 * [RETURN VALUE]
 * 'zTri3DListCopyArray()' returns no value.
 */
__EXPORT void zTri3DListCopyArray(zTri3DList *list, zTri3D t[], int n);

__END_DECLS

#endif /* __ZEO_ELEM_LIST_H__ */
