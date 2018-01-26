/* CURE - Commonly-Used Routine Elements
 * Copyright (C) 1998 Tomomichi Sugihara (Zhidao)
 */
/*! \file cure_array.h
 * \brief array management.
 * \author Zhidao
 */

#ifndef __CURE_ARRAY_H__
#define __CURE_ARRAY_H__

#include <cure/cure_misc.h>

__BEGIN_DECLS

/* ********************************************************** */
/*! \defgroup array universal array operations.
 * \{ *//* ************************************************** */

/*! \def zArrayClass(array_t,cell_t)
 * \brief generate array class.
 *
 * A macro zArrayClass() can generate a new array class,
 * which consists of the total number of elements and
 * the pointer to the array buffer.
 * \a array_t is the class name to be defined.
 * \a cell_t is the class name of the data to be arrayed.
 *
 * The number of elements is acquired by zArrayNum().
 * The array head can be accessed by zArrayBuf().
 * Each element can be accessed by zArrayElem().
 *
 * When setting the number of elements and the array,
 * zArraySetNum() and zArraySetBuf() are available.
 * But, it is not preferable to use them directly.
 * Use zArrayAlloc() instead.
 * When destroying the array, call zArrayFree().
 *
 * For safe accesses to elements, zArrayGetElem() and
 * zArraySetElem() are prepared. For frequent accesses
 * to the head, neck (one-cell-before-head) and tail,
 * zArrayHead(), zArrayNeck(), zArrayTail(),
 * zArraySetHead(), zArraySetNeck() and zArraySetTail()
 * are prepared.
 */
#ifdef __cplusplus
#define zArrayClass(array_t,cell_t) \
struct array_t{\
  uint num;\
  cell_t *buf;\
}
#else
#define zArrayClass(array_t,cell_t) \
typedef struct{\
  uint num;\
  cell_t *buf;\
} array_t
#endif /* __cplusplus */

#define zArrayNum(arr)      (arr)->num
#define zArrayBuf(arr)      ( (arr)->buf )
#define zArrayElem(arr,i)   ( &zArrayBuf(arr)[i] )

#define zArraySetNum(arr,n) ( zArrayNum(arr) = (n) )
#define zArraySetBuf(arr,b) ( zArrayBuf(arr) = (b) )

#define zArrayPosIsValid(a,p) ( (p) < zArrayNum(a) && (p) >= 0 )

/* NOTE: do not use the following macro before allocating buffer */
#define zArrayElemSize(arr) sizeof(*zArrayBuf(arr))

#define zArrayGetElem(a,i) ( zArrayPosIsValid(a,i) ? zArrayElem(a,i) : NULL )
#define zArraySetElem(a,i,d) \
  if( zArrayPosIsValid(a,i) ) memcpy( zArrayElem(a,i), (d), zArrayElemSize(a) )

#define zArrayHead(a)      zArrayElem( a, zArrayNum(a)-1 )
#define zArraySetHead(a,d) zArraySetElem( a, zArrayNum(a)-1, d )
#define zArrayNeck(a)      zArrayElem( a, zArrayNum(a)-2 )
#define zArraySetNeck(a,d) zArraySetElem( a, zArrayNum(a)-2, d )
#define zArrayTail(a)      zArrayElem( a, 0 )
#define zArraySetTail(a,d) zArraySetElem( a, 0, d )

#define zArrayInit(arr) do{\
  zArraySetNum( arr, 0 );\
  zArraySetBuf( arr, NULL );\
} while(0)
/*! \brief allocate an array.
 * \param arr array class instance to be allocated.
 * \param type the data type of the array cells.
 * \param n the number of cells.
 */
#define zArrayAlloc(arr,type,n) do{\
  zArrayInit( arr );\
  if( (n) > 0 && !zArraySetBuf( arr, zAlloc(type,n) ) ){\
    ZALLOCERROR();\
    zArraySetNum( arr, 0 );\
  } else\
    zArraySetNum( arr, n );\
} while(0)
#define zArrayFree(arr) do{\
  zFree( zArrayBuf(arr) );\
  zArraySetNum( arr, 0 );\
} while(0)

/*! zArrayFindName() is valid for an array of a named class.
 * \sa zNameFind.
 */
#define zArrayFindName(arr,name,ptr) \
  zNameFind( zArrayBuf(arr), zArrayNum(arr), name, ptr )

#ifndef __KERNEL__
/* since 'realloc()' is not prepared in kernel space,
 * the following methods are only available in user space.
 */

/*! \brief add a new cell to an array.
 *
 * zArrayAdd() adds a new cell pointed by \a dat to an
 * array \a arr at the last, incrementing the number of
 * \a arr. \a type is the data type of the cell.
 */
#define zArrayAdd(arr,type,dat) do{\
  type *__zarray_ap;\
  __zarray_ap = zRealloc( zArrayBuf(arr), type, zArrayNum(arr)+1 );\
  if( __zarray_ap == NULL )\
    ZALLOCERROR();\
  else{\
    zArrayNum(arr)++;\
    zArrayBuf(arr) = __zarray_ap;\
    zArraySetHead( arr, dat );\
  }\
} while(0)

/*! \brief insert a new cell into an array.
 *
 * zArrayInsert() inserts a new cell pointed by \a dat to in an
 * array \a arr at the location specified by \a pos, incrementing
 * the number of \a arr. \a type is the data type of the cell.
 */
#define zArrayInsert(arr,type,pos,dat) do{\
  type *__zarray_ap;\
  if( (pos) == zArrayNum(arr) ){\
    zArrayAdd( arr, type, dat );\
  } else if( zArrayPosIsValid(arr,pos) ){\
    __zarray_ap = zRealloc( zArrayBuf(arr), type, zArrayNum(arr)+1 );\
    if( __zarray_ap == NULL )\
      ZALLOCERROR();\
    else{\
      zArrayBuf(arr) = __zarray_ap;\
      memmove( zArrayElem(arr,(pos)+1), zArrayElem(arr,pos), sizeof(type)*(zArrayNum(arr)-(pos)) );\
      zArraySetElem( arr, pos, dat );\
      zArrayNum(arr)++;\
    }\
  }\
} while(0)

/*! \brief delete a cell from an array.
 *
 * zArrayDelete() deletes a cell at the location specified by
 * \a pos in an array \a arr at, decrementing the number of \a arr.
 * \a type is the data type of the cell.
 */
#define zArrayDelete(arr,type,pos) do{\
  type *__zarray_ap;\
  if( zArrayPosIsValid(arr,pos) ){\
    if( (pos) < zArrayNum(arr)-1 )\
      memmove( zArrayElem(arr,pos), zArrayElem(arr,(pos)+1), sizeof(type)*(zArrayNum(arr)-(pos)-1) );\
    if( zArrayNum(arr) > 1 ){\
      __zarray_ap = zRealloc( zArrayBuf(arr), type, zArrayNum(arr)-1 );\
      if( __zarray_ap == NULL )\
        ZALLOCERROR();\
      else{\
        zArrayNum(arr)--;\
        zArrayBuf(arr) = __zarray_ap;\
      }\
    } else{\
      zArrayFree( arr );\
    }\
  }\
} while(0)

/*! \brief append an array to another.
 *
 * zArrayAppend() appends an array pointed by \a subarr
 * to another array pointed by \a arr.
 * \a type is the data type of the cell.
 */
#define zArrayAppend(arr,subarr,type) do{\
  type *__zarray_ap;\
  __zarray_ap = zRealloc( zArrayBuf(arr), type, zArrayNum(arr)+zArrayNum(subarr) );\
  if( __zarray_ap == NULL )\
    ZALLOCERROR();\
  else{\
    zArrayBuf(arr) = __zarray_ap;\
    memcpy( zArrayElem(arr,zArrayNum(arr)), zArrayBuf(subarr), zArrayNum(subarr)*sizeof(type) );\
    zArrayNum(arr) += zArrayNum(subarr);\
  }\
} while(0)

#endif /* __KERNEL__ */

/*! \} */

__END_DECLS

#endif /* __CURE_ARRAY_H__ */
