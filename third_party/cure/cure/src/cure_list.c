/* CURE - Commonly-Used Routine Elements
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 *
 * cure_list - list operation.
 */

#include <cure/cure_list.h>

/* ********************************************************** */
/* CATEGORY: dynamically-allocated list
 * ********************************************************** */

/* ********************************************************** */
/* CLASS: zList
 * list structure
 * ********************************************************** */

#ifndef __KERNEL__
/* _zListFWrite
 * - print out an information of a list.
 */
void _zListFWrite(FILE *fp, zList *list)
{
  register int i = 0;
  zListCell *cp;

  fprintf( fp, "number = %d\n", zListNum( list ) );
  zListForEach( list, cp ){
    fprintf( fp, "<%d>", i++ );
    zListCellFWrite( fp, cp );
  }
}
#else
/* zListWrite
 * - print out an information of a list(for kernel module).
 */
void zListWrite(zList *list)
{
  int i;
  zListCell *cp;

  printk( "number = %d\n", zListNum( list ) );
  zListForEach( list, cp ){
    printk( "<%d>", i++ );
    zListCellWrite( cp );
  }
}
#endif /* __KERNEL__ */

/* quick sort for static list structure */

static void _zInnerQuickSort(void **head, void **tail, int (*cmp)(void*,void*,void*), void *priv);

/* (static)
 * zInnerQuickSort - inner operation for 'zQuickSort', which
 * is called recursively.
 */
void _zInnerQuickSort(void **head, void **tail, int (*cmp)(void*,void*,void*), void *priv)
{
  void **hp, **tp, *tmp, *pivot;

  pivot = *head;
  for( hp=head, tp=tail; ; ){
    while( cmp( *hp, pivot, priv ) > 0 ) hp--;
    while( cmp( *tp, pivot, priv ) < 0 ) tp++;
    if( hp == tp || hp == tp-1 )
      break;
    tmp = *hp;
    *hp-- = *tp;
    *tp++ = tmp;
  }
  if( hp != head )
    _zInnerQuickSort( head, hp+1, cmp, priv );
  if( tp != tail )
    _zInnerQuickSort( tp-1, tail, cmp, priv );
}

/* zQuickSort
 * - quick sort for an array of pointers.
 */
void *zQuickSort(void *array[], int size, int (*cmp)(void*,void*,void*), void *priv)
{
  _zInnerQuickSort( array+size-1, array, cmp, priv );
  return array;
}
