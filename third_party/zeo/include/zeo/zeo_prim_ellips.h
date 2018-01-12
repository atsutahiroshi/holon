/* Zeo - Z/Geometry and optics computation library.
 * Copyright (C) 2005 Tomomichi Sugihara (Zhidao)
 *
 * zeo_prim_ellips - primitive 3D shapes: ellipsoid.
 */

#ifndef __ZEO_PRIM_ELLIPS_H__
#define __ZEO_PRIM_ELLIPS_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

/* ********************************************************** */
/* CLASS: zEllips3D
 * 3D ellipsoid class
 * ********************************************************** */

typedef struct{
  zVec3D radius;
  zFrame3D f;
  int div;
} zEllips3D;

#define zEllips3DRadius(e,d)      zVec3DElem( &(e)->radius, d )
#define zEllips3DRadiusX(e)       zEllips3DRadius( e, zX )
#define zEllips3DRadiusY(e)       zEllips3DRadius( e, zY )
#define zEllips3DRadiusZ(e)       zEllips3DRadius( e, zZ )
#define zEllips3DCenter(e)        zFrame3DPos( &(e)->f )
#define zEllips3DAxis(e,i)        zMat3DVec( zFrame3DAtt(&(e)->f), i )
#define zEllips3DDiv(e)           (e)->div

#define zEllips3DSetRadius(e,d,r) zVec3DSetElem( &(e)->radius, (d), r )
#define zEllips3DSetRadiusX(e,r)  zEllips3DSetRadius( e, zX, r )
#define zEllips3DSetRadiusY(e,r)  zEllips3DSetRadius( e, zY, r )
#define zEllips3DSetRadiusZ(e,r)  zEllips3DSetRadius( e, zZ, r )
#define zEllips3DSetCenter(e,c)   zVec3DCopy( c, zEllips3DCenter(e) )
#define zEllips3DSetAxis(e,i,a)   zVec3DCopy( a, zEllips3DAxis(e,i) )
#define zEllips3DSetDiv(e,d)      ( zEllips3DDiv(e) = (d) )

/*! \brief initialization, creation and copy of 3D ellipsoid.
 *
 * 'zEllips3DInit()' initializes a 3D ellipsoid 'ellips',
 * setting its center for the original point and all radi
 * for zero.
 * #
 * 'zEllips3DCreate()' creates a 3D ellipsoid whose center
 * is \a c and radius along x, y and z axes are \a rx, \a ry and \a yz,
 * respectively.
 * \a div is the number of division for polyhedral approximation.
 * When zero is given for \a div, ZEO_PRIM_DEFAULT_DIV is
 * set instead.
 * #
 * 'zEllips3DCopy()' copies a 3D ellipsoid \a src to the
 * other \a dest.
 * [RETURN VALUE]
 * Each of 'zEllips3DInit()' and 'zEllips3DCreate()'
 * returns a pointer 'ellips'.
 * 'zEllips3DCopy()' returns a pointer the copied 'dest'.
 */
__EXPORT zEllips3D *zEllips3DCreate(zEllips3D *ellips, zVec3D *c, zVec3D *ax, zVec3D *ay, zVec3D *az, double rx, double ry, double rz, int div);
#define zEllips3DCreateAlign(e,c,rx,ry,rz,d) \
  zEllips3DCreate( e, c, ZVEC3DX, ZVEC3DY, ZVEC3DZ, rx, ry, rz, d )
__EXPORT zEllips3D *zEllips3DInit(zEllips3D *ellips);
__EXPORT zEllips3D *zEllips3DCopy(zEllips3D *src, zEllips3D *dest);
__EXPORT zEllips3D *zEllips3DMirror(zEllips3D *src, zEllips3D *dest, zAxis axis);

/* METHOD:
 * zEllips3DXfer, zEllips3DXferInv
 * - transfer 3D ellipsoid.
 *
 * 'zEllips3DXfer()' transforms a 3D ellipsoid 'src' by
 * a transformation frame 'f', put it into 'dest'.
 * #
 * 'zEllips3DXferInv()' transforms 'src' by the
 * inverse transformation frame of 'f', put it into 'dest'.
 * [RETURN VALUE]
 * Each of 'zEllips3DXfer()' and 'zEllips3DXferInv()'
 * returns a pointer 'dest'.
 */
__EXPORT zEllips3D *zEllips3DXfer(zEllips3D *src, zFrame3D *f, zEllips3D *dest);
__EXPORT zEllips3D *zEllips3DXferInv(zEllips3D *src, zFrame3D *f, zEllips3D *dest);

/* METHOD:
 * zEllips3DClosest, zEllips3DPointDist, zEllips3DPointIsInside
 * - distance from 3D point to 3D ellipsoid.
 *
 * 'zEllips3DClosest()' calculates the closest point
 * from a 3D point 'p' to a 3D ellipsoid 'ellips', and put it
 * into 'cp'. When 'p' is inside of 'ellips', it just
 * copies 'p' to 'cp'.
 * #
 * 'zEllips3DPointDist()' calculates the distance from
 * a 3D point 'p' to a 3D ellipsoid 'ellips'.
 * #
 * 'zEllips3DPointIsInside()' checks if a 3D point'p' is
 * inside of a 3D ellipsoid 'ellips'. The point on the
 * surface of 'ellips' is judged to be inside of 'ellips'
 * if the true value is given for 'rim'.
 * [RETURN VALUE]
 * Each of 'zEllips3DClosest()' and 'zEllips3DPointDist()'
 * returns the signed distance from 'p' to 'ellips'.
 * The result is
 *  - a positive value when 'p' is outside of 'ellips', or
 *  - a negative value when 'p' is inside of 'ellips'.
 * #
 * 'zEllips3DPointIsInside()' returns the true value if
 * 'p' is inside of 'ellips', or the false value otherwise.
 */
__EXPORT double zEllips3DClosest(zEllips3D *ellips, zVec3D *p, zVec3D *cp);
__EXPORT double zEllips3DPointDist(zEllips3D *ellips, zVec3D *p);
__EXPORT bool zEllips3DPointIsInside(zEllips3D *ellips, zVec3D *p, bool rim);

/* METHOD:
 * zEllips3DVolume, zEllips3DInertia
 * - volume and inertia of 3D ellipsoid.
 *
 * 'zEllips3DVolume()' calculates the volume of a
 * 3D ellipsoid 'ellips'.
 * #
 * 'zEllips3DInertia()' calculates the inertia tensor
 * of a 3D ellipsoid 'ellips' around its center. It treats
 * 'ellips' as a solid model. The result is put into 'inertia'.
 * [RETURN VALUE]
 * 'zEllips3DVolume()' returns the volume calculated.
 * 'zEllips3DInertia()' returns a pointer 'inertia'.
 */
__EXPORT double zEllips3DVolume(zEllips3D *ellips);
__EXPORT zMat3D *zEllips3DInertia(zEllips3D *ellips, zMat3D *inertia);

/* METHOD:
 * zEllips3DToPH - convert ellipsoid to polyhedron.
 *
 * 'zEllips3DToPH()' converts a ellipsoid 'ellips' to
 * a polyhedron 'ph' as a polygon model.
 * 'ph' should be initialized in advance.
 * It approximately divides the face into rectangles
 * by the stored number of division.
 * [RETURN VALUE]
 * zEllips3DToPH() returns a pointer \a ph.
 * [SEE ALSO]
 * zCyl3DToPH, zCyl3DToPHDRC,
 * zCone3DToPH, zCone3DToPHDRC
 */
/* default longitudinal & latitudinal division number are the same. */
__EXPORT zPH3D *zEllips3DToPH(zEllips3D *ellips, zPH3D *ph);

/* METHOD:
 * zEllips3DFRead, zEllips3DRead, zEllips3DFWrite, zEllips3DWrite,
 * - input/output of 3D ellipsoid.
 *
 * 'zEllips3DFRead()' reads the information of a 3D ellipsoid
 * from the current position of the file 'fp', and creates
 * the new ellipsoid 'ellips'.
 * An acceptable data file format is as follows.
 * #
 *  center: <x> <y> <z>
 *  ax: <x> <y> <z>
 *  ay: <x> <y> <z>
 *  az: <x> <y> <z>
 *  rx: <r>
 *  ry: <r>
 *  rz: <r>
 *  div: <div>
 * #
 * Each bracketed value must be substituted for a real number.
 * 'zEllips3DRead()' reads the information for 'ellips'
 * simply from the standard input.
 * The field div is skippable.
 * #
 * 'zEllips3DFWrite()' writes the information of 'ellips' to
 * the current position of the file 'fp' in the same format
 * with the above. 'zEllips3DWrite()' writes the information
 * of 'ellips' simply to the standard out.
 * [RETURN VALUE]
 * Each of 'zEllips3DFRead()' and 'zEllips3DRead()' returns
 * a pointer 'ellips'.
 * #
 * Neither 'zEllips3DFWrite()' nor 'zEllips3DWrite()' returns
 * any values.
 */
__EXPORT zEllips3D *zEllips3DFRead(FILE *fp, zEllips3D *ellips);
#define zEllips3DRead(e) zEllips3DFRead( stdin, (e) )
__EXPORT void zEllips3DFWrite(FILE *fp, zEllips3D *ellips);
#define zEllips3DWrite(e) zEllips3DFWrite( stdout, (e) )

/* methods for abstraction */
extern zPrimCom zprim_ellips3d_com;

__END_DECLS

#endif /* __ZEO_PRIM_ELLIPS_H__ */
