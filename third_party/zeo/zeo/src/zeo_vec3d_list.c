/* Zeo - Z/Geometry and optics computation library.
 * Copyright (C) 2005 Tomomichi Sugihara (Zhidao)
 *
 * zeo_vec3d_list - 3D vector list.
 */

#include <zeo/zeo_mat3d.h>

/* ********************************************************** */
/* CLASS: zVec3DList
 * 3D vector list
 * ********************************************************** */

/* zVec3DListInsert
 * - insert 3D vector list cell.
 */
zVec3DListCell *zVec3DListInsert(zVec3DList *list, zVec3D *v, bool flag)
{
  zVec3DListCell *cell;

  if( !( cell = zAlloc( zVec3DListCell, 1 ) ) ){
    ZALLOCERROR();
    return NULL;
  }
  if( flag ){
    if( !( cell->data = zAlloc( zVec3D, 1 ) ) ){
      ZALLOCERROR();
      zFree( cell );
      return NULL;
    }
    zVec3DCopy( v, cell->data );
  } else
    cell->data = v;
  zListInsertHead( list, cell );
  return cell;
}

/* zVec3DListCreate
 * - create vector list.
 */
zVec3DList *zVec3DListCreate(zVec3DList *list, zVec3D v[], int num, bool flag)
{
  zListInit( list );
  while( num-- > 0 )
    if( !zVec3DListInsert( list, v++, flag ) ){
      ZALLOCERROR();
      break;
    }
  return list;
}

/* zVec3DListDestroy
 * - destroy vector list.
 */
void zVec3DListDestroy(zVec3DList *list, bool flag)
{
  zVec3DListCell *cell;

  while( !zListIsEmpty( list ) ){
    zListDeleteHead( list, &cell );
    if( flag ) zFree( cell->data );
    zFree( cell );
  }
}

/* zVec3DListQuickSort
 * - a quick sort routine for vector list class.
 */
zListQuickSortDef( zVec3DList, zVec3DListCell )

/* zVec3DListFWrite
 * - output 3D vector list.
 */
void zVec3DListFWrite(FILE *fp, zVec3DList *list)
{
  zVec3DListCell *cp;

  fprintf( fp, "%d\n", zListNum(list) );
  zListForEach( list, cp )
    zVec3DFWrite( fp, cp->data );
}

/* zVec3DListDataFWrite
 * - output 3D vector list.
 */
void zVec3DListDataFWrite(FILE *fp, zVec3DList *list)
{
  zVec3DListCell *cp;

  zListForEach( list, cp )
    zVec3DDataNLFWrite( fp, cp->data );
}

/* ********************************************************** */
/* utilities for point clouds
 * ********************************************************** */

/* zVec3DListNN
 * - a naive algorithm to find the nearest neighbor in a 3D vector list.
 */
zVec3D *zVec3DListNN(zVec3DList *list, zVec3D *v, double *dmin)
{
  zVec3DListCell *cell;
  double d;
  zVec3D *nn = NULL;

  *dmin = HUGE_VAL;
  zListForEach( list, cell )
    if( ( d = zVec3DSqrDist( cell->data, v ) ) < *dmin ){
      *dmin = d;
      nn = cell->data;
    }
  *dmin = sqrt( *dmin );
  return nn;
}
