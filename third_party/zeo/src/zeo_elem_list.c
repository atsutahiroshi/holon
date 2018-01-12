/* Zeo - Z/Geometry and optics computation library.
 * Copyright (C) 2005 Tomomichi Sugihara (Zhidao)
 *
 * zeo_elem_list - 3D shape element list.
 */

#include <zeo/zeo_elem.h>

/* ********************************************************** */
/* CLASS: zTri3DList
 * 3D triangle list
 * ********************************************************** */

/* zTri3DListInsert
 * - insert 3D triangle list cell.
 */
zTri3DListCell *zTri3DListInsert(zTri3DList *list, zTri3D *t, bool flag)
{
  zTri3DListCell *cell;

  if( !( cell = zAlloc( zTri3DListCell, 1 ) ) ){
    ZALLOCERROR();
    return NULL;
  }
  if( flag ){
    if( !( cell->data = zAlloc( zTri3D, 1 ) ) ){
      ZALLOCERROR();
      zFree( cell );
      return NULL;
    } else
      memcpy( cell->data, t, sizeof(zTri3D) );
  } else
    cell->data = t;
  zListInsertHead( list, cell );
  return cell;
}

/* zTri3DListDestroy
 * - destroy triangle list.
 */
void zTri3DListDestroy(zTri3DList *list, bool flag)
{
  zTri3DListCell *cell;

  while( !zListIsEmpty(list) ){
    zListDeleteHead( list, &cell );
    if( flag )
      zFree( cell->data );
    zFree( cell );
  }
}

/* zTri3DListAlign
 * - align triangles to a direction referred by a vector.
 */
void zTri3DListAlign(zTri3DList *list, zVec3D *ref)
{
  zTri3DListCell *tp;

  zListForEach( list, tp )
    if( zVec3DInnerProd( ref, zTri3DNorm(tp->data) ) < 0 )
      zTri3DRevDRC( tp->data );
}

/* zTri3DListCopyArray
 * - copy triangles in a list to array.
 */
void zTri3DListCopyArray(zTri3DList *list, zTri3D t[], int n)
{
  zTri3DListCell *tp;
  register int i = 0;

  zListForEach( list, tp ){
    memcpy( &t[i], tp->data, sizeof(zTri3D) );
    if( ++i >= n ) break;
  }
}
