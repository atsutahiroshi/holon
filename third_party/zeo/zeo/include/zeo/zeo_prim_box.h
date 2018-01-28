/* Zeo - Z/Geometry and optics computation library.
 * Copyright (C) 2005 Tomomichi Sugihara (Zhidao)
 *
 * zeo_prim_box - primitive 3D shapes: box.
 */

#ifndef __ZEO_PRIM_BOX_H__
#define __ZEO_PRIM_BOX_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

/* ********************************************************** */
/* CLASS: zBox3D
 * 3D box class
 * ********************************************************** */

typedef struct{
  zVec3D dia;
  zFrame3D f;
} zBox3D;

#define zBox3DDia(b,d)       zVec3DElem( &(b)->dia, d )
#define zBox3DDepth(b)       zBox3DDia( b, zX )
#define zBox3DWidth(b)       zBox3DDia( b, zY )
#define zBox3DHeight(b)      zBox3DDia( b, zZ )
#define zBox3DCenter(b)      zFrame3DPos( &(b)->f )
#define zBox3DAxis(b,i)      zMat3DVec( zFrame3DAtt(&(b)->f), i )

#define zBox3DSetDia(b,d,l)  zVec3DSetElem( &(b)->dia, (d), l )
#define zBox3DSetDepth(b,d)  zBox3DSetDia( b, zX, d )
#define zBox3DSetWidth(b,w)  zBox3DSetDia( b, zY, w )
#define zBox3DSetHeight(b,h) zBox3DSetDia( b, zZ, h )
#define zBox3DSetCenter(b,c) zVec3DCopy( c, zBox3DCenter(b) )
#define zBox3DSetAxis(b,i,a) zVec3DCopy( a, zBox3DAxis(b,i) )

/* METHOD:
 * zBox3DInit, zBox3DCreate, zBox3DCreateAlign, zBox3DCopy
 * - initialization, creation and copy of 3D box.
 *
 * zBox3DInit() initializes a 3D box \a box, setting its
 * center for the original point and volume for zero.
 *
 * zBox3DCreate() creates a 3D box whose center, width,
 * height and depth are \a c, \a w, \a h and \a d, respectively.
 * \a ax, \a ay and \a az are expected to be perpendicular
 * with each other.
 *
 * zBox3DCopy() copies a 3D box \a src to the other \a dest.
 * \return
 * Each of zBox3DInit() and zBox3DCreate() returns a pointer
 * \a box.
 * zBox3DCopy() returns a pointer the copied \a dest.
 */
__EXPORT zBox3D *zBox3DCreate(zBox3D *box, zVec3D *c, zVec3D *ax, zVec3D *ay, zVec3D *az, double d, double w, double h);
#define zBox3DCreateAlign(b,c,d,w,h) \
  zBox3DCreate( b, c, ZVEC3DX, ZVEC3DY, ZVEC3DZ, d, w, h )
__EXPORT zBox3D *zBox3DInit(zBox3D *box);
__EXPORT zBox3D *zBox3DCopy(zBox3D *src, zBox3D *dest);
__EXPORT zBox3D *zBox3DMirror(zBox3D *src, zBox3D *dest, zAxis axis);

/* METHOD:
 * zBox3DXfer, zBox3DXferInv
 * - transfer 3D box.
 *
 * 'zBox3DXfer()' transfers a 3D box 'src' by a transformation
 * frame 'f', put it into 'dest'.
 *
 * 'zBox3DXferInv()' inversely transfers 'src' by a transformation
 * frame of 'f', put it into 'dest'.
 * [RETURN VALUE]
 * Each of 'zBox3DXfer()' and 'zBox3DXferInv()'
 * returns a pointer 'dest'.
 */
__EXPORT zBox3D *zBox3DXfer(zBox3D *src, zFrame3D *f, zBox3D *dest);
__EXPORT zBox3D *zBox3DXferInv(zBox3D *src, zFrame3D *f, zBox3D *dest);

/* METHOD:
 * zBox3DClosest, zBox3DPointDist, zBox3DPointIsInside
 * - distance from 3D point to 3D box.
 *
 * 'zBox3DClosest()' calculates the closest point
 * from a 3D point 'p' to a 3D box 'box', and put it
 * into 'cp'. When 'p' is inside of 'box', it just
 * copies 'p' to 'cp'.
 *
 * 'zBox3DPointDist()' calculates the distance from
 * a 3D point 'p' to a 3D box 'box'.
 *
 * 'zBox3DPointIsInside()' checks if a 3D point 'p' is
 * inside of a 3D box 'box'. The point on the surface
 * of 'box' is judged to be inside of 'box' if the
 * true value is given for 'rim'.
 * [RETURN VALUE]
 * Each of 'zBox3DClosest()' and 'zBox3DPointDist()'
 * returns the signed distance from 'p' to 'box'.
 * The result is
 *  - a positive value when 'p' is outside of 'box', or
 *  - a negative value when 'p' is inside of 'box'.
 *
 * 'zBox3DPointIsInside()' returns the true value
 * if 'p' is inside of 'box', or the false value
 * otherwise.
 */
__EXPORT double zBox3DClosest(zBox3D *box, zVec3D *p, zVec3D *cp);
__EXPORT double zBox3DPointDist(zBox3D *box, zVec3D *p);
__EXPORT bool zBox3DPointIsInside(zBox3D *box, zVec3D *p, bool rim);

/* METHOD:
 * zBox3DVolume, zBox3DInertia
 * - volume and inertia of 3D box.
 *
 * 'zBox3DVolume()' calculates the volume of a
 * 3D box 'box'.
 *
 * 'zBox3DInertia()' calculates the inertia tensor
 * of a 3D box 'box' around its center. It treats
 * 'box' as a solid model. The result is put into 'inertia'.
 * [RETURN VALUE]
 * 'zBox3DVolume()' returns the volume calculated.
 * 'zBox3DInertia()' returns a pointer 'inertia'.
 */
__EXPORT double zBox3DVolume(zBox3D *box);
__EXPORT zMat3D *zBox3DInertia(zBox3D *box, zMat3D *inertia);

/* METHOD:
 * zBox3DVert - get vertex of a box.
 *
 * 'zBox3DVert()' gets the 'i'th vertex of a box 'box',
 * and puts it into 'v'.
 * The order is according to the figure below.
 *    2----1
 *   /    /|
 *  3----0 |  z
 *  |(6) | 5  |__y
 *  |    |/   /
 *  7----4   x
 * [RETURN VALUE]
 * 'zBox3DVert()' returns a pointer 'v'.
 */
__EXPORT zVec3D *zBox3DVert(zBox3D *box, int i, zVec3D *v);

/* METHOD:
 * zBox3DToPH
 * - conversion from box to polyhedron.
 *
 * 'zBox3DToPH()' converts a box 'box' to a polyhedron
 * 'ph' as a polygon model, allocating eight vectors
 * and twelve polygons. The order of the vertices is
 * according to 'zBox3DVert()'.
 *
 * 'ph' should be initialized in advance.
 * [RETURN VALUE]
 * 'zBox3DToPH()' returns a pointer 'ph'.
 * [SEE ALSO]
 * zBox3DVert
 */
__EXPORT zPH3D *zBox3DToPH(zBox3D *box, zPH3D *ph);

/* METHOD:
 * zBox3DFRead, zBox3DRead, zBox3DFWrite, zBox3DWrite,
 * - input/output of 3D box.
 *
 * 'zBox3DFRead()' reads the information of a 3D box
 * from the current position of the file 'fp', and creates
 * the new box 'box'.
 * An acceptable data file format is as follows.
 *
 *  center: <x> <y> <z>
 *  ax: <x> <y> <z>
 *  ay: <x> <y> <z>
 *  az: <x> <y> <z>
 *  depth: <d>
 *  width: <w>
 *  height: <h>
 *
 * Each bracketed value must be substituted for a real number.
 * 'zBox3DRead()' reads the information for 'box'
 * simply from the standard input.
 *
 * 'zBox3DFWrite()' writes the information of 'box' to
 * the current position of the file 'fp' in the same format
 * with the above. 'zBox3DWrite()' writes the information
 * of 'box' simply to the standard out.
 * [RETURN VALUE]
 * Each of 'zBox3DFRead()' and 'zBox3DRead()' returns
 * a pointer 'box'.
 *
 * Neither 'zBox3DFWrite()' nor 'zBox3DWrite()' returns
 * any values.
 */
__EXPORT zBox3D *zBox3DFRead(FILE *fp, zBox3D *box);
#define zBox3DRead(b) zBox3DFRead( stdin, b )
__EXPORT void zBox3DFWrite(FILE *fp, zBox3D *box);
#define zBox3DWrite(b) zBox3DFWrite( stdout, b )

/*! \brief output a 3D box to a file in a format to be plotted. */
__EXPORT void zBox3DDataFWrite(FILE *fp, zBox3D *box);

/* methods for abstraction */
extern zPrimCom zprim_box3d_com;

__END_DECLS

#endif /* __ZEO_PRIM_BOX_H__ */
