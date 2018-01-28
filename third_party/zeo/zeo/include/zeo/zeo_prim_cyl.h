/* Zeo - Z/Geometry and optics computation library.
 * Copyright (C) 2005 Tomomichi Sugihara (Zhidao)
 *
 * zeo_prim_cyl - primitive 3D shapes: cylinder.
 */

#ifndef __ZEO_PRIM_CYL_H__
#define __ZEO_PRIM_CYL_H__

/* NOTE: never include this header file in user programs. */

__BEGIN_DECLS

/* ********************************************************** */
/* CLASS: zCyl3D
 * 3D cylinder class
 * ********************************************************** */

typedef struct{
  zVec3D center[2];
  double radius;
  int div;
} zCyl3D;

#define zCyl3DCenter(c,i)     ( &(c)->center[i] )
#define zCyl3DRadius(c)       (c)->radius
#define zCyl3DDiv(c)          (c)->div

#define zCyl3DSetCenter(c,i,p) zVec3DCopy(p,zCyl3DCenter(c,i))
#define zCyl3DSetRadius(c,r)   ( zCyl3DRadius(c) = (r) )
#define zCyl3DSetDiv(c,d)      ( zCyl3DDiv(c) = (d) )

/* METHOD:
 * zCyl3DInit, zCyl3DCreate, zCyl3DCopy
 * - initialization, creation and copy of 3D cylinder.
 *
 * 'zCyl3DInit()' initializes a 3D cylinder 'cyl',
 * setting both of its centers on the bases for the original
 * point and radius for zero.
 *
 * 'zCyl3DCreate()' creates a 3D cylinder whose two
 * centers point on the bases are 'c1' and 'c2', and
 * radius is 'r'.
 * \a div is the number of division for polyhedral approximation.
 * When zero is given for \a div, ZEO_PRIM_DEFAULT_DIV is
 * set instead.
 *
 * 'zCyl3DCopy()' copies a 3D cylinder 'src' to the
 * other 'dest'.
 * [RETURN VALUE]
 * Each of 'zCyl3DInit()' and 'zCyl3DCreate()'
 * returns a pointer 'cyl'.
 * 'zCyl3DCopy()' returns a pointer the copied 'dest'.
 */
__EXPORT zCyl3D *zCyl3DCreate(zCyl3D *cyl, zVec3D *c1, zVec3D *c2, double r, int div);
__EXPORT zCyl3D *zCyl3DInit(zCyl3D *cyl);
__EXPORT zCyl3D *zCyl3DCopy(zCyl3D *src, zCyl3D *dest);
__EXPORT zCyl3D *zCyl3DMirror(zCyl3D *src, zCyl3D *dest, zAxis axis);

/* METHOD:
 * zCyl3DXfer, zCyl3DXferInv
 * - transformation of 3D cylinder.
 *
 * 'zCyl3DXfer()' transforms a 3D cylinder 'src' by
 * a transformation frame 'f', put it into 'dest'.
 * #
 * 'zCyl3DXferInv()' transforms 'src' by the
 * inverse transformation frame of 'f', put it into 'dest'.
 * [RETURN VALUE]
 * Each of 'zCyl3DXfer()' and 'zCyl3DXferInv()'
 * returns a pointer 'dest'.
 */
__EXPORT zCyl3D *zCyl3DXfer(zCyl3D *src, zFrame3D *f, zCyl3D *dest);
__EXPORT zCyl3D *zCyl3DXferInv(zCyl3D *src, zFrame3D *f, zCyl3D *dest);

/* METHOD:
 * zCyl3DPointIsInside
 * - check if a point is inside of a cylinder.
 *
 * 'zCyl3DPointIsInside()' checks if a 3D point'p' is
 * inside of a 3D cylinder 'cyl'.
 * [RETURN VALUE]
 * 'zCyl3DPointIsInside()' returns the true value when
 * 'p' is inside of 'cyl', or the false value otherwise.
 */
__EXPORT double zCyl3DClosest(zCyl3D *cyl, zVec3D *p, zVec3D *cp);
__EXPORT double zCyl3DPointDist(zCyl3D *cyl, zVec3D *p);
__EXPORT bool zCyl3DPointIsInside(zCyl3D *cyl, zVec3D *p, bool rim);

/* METHOD:
 * zCyl3DAxis, zCyl3DHeight, zCyl3DVolume
 * - axis vector, height and volume of 3D cylinder.
 *
 * 'zCyl3DAxis()' calculates the axis vector of
 * a 3D cylinder 'cyl'; the axis from the center
 * point on the bottom base to the center point on the
 * top base.
 * #
 * 'zCyl3DHeight()' calculates the height from
 * the bottom base to the top base of a 3D cylinder
 * 'cyl'.
 * #
 * 'zCyl3DVolume()' calculates the volume of
 * a 3D cylinder 'cyl'.
 * [RETURN VALUE]
 * 'zCyl3DAxis()' returns a pointer 'axis'.
 * 'zCyl3DHeight()' returns the height calculated.
 * 'zCyl3DVolume()' returns the volume calculated.
 */
#define zCyl3DAxis(c,a) \
  zVec3DSub( zCyl3DCenter(c,1), zCyl3DCenter(c,0), a )
__EXPORT double zCyl3DHeight(zCyl3D *cyl);
__EXPORT double zCyl3DVolume(zCyl3D *cyl);
__EXPORT zVec3D *zCyl3DBarycenter(zCyl3D *cyl, zVec3D *c);
__EXPORT zMat3D *zCyl3DInertia(zCyl3D *cyl, zMat3D *inertia);

/* METHOD:
 * zCyl3DToPH - convert cylinder to polyhedron.
 *
 * 'zCyl3DToPH()' converts a cylinder 'cyl' to
 * a polyhedron 'ph' as a polygon model.
 * It approximately divides the side face into rectangles
 * by the stored number of division.
 * 'ph' should be initialized in advance.
 * [RETURN VALUE]
 * zCyl3DToPH() returns a pointer \a ph.
 * [SEE ALSO]
 * zSphere3DToPH, zSphere3DToPH,
 * zCone3DToPH, zCone3DToPHDRC
 */
__EXPORT zPH3D *zCyl3DToPH(zCyl3D *cyl, zPH3D *ph);

/* METHOD:
 * zCyl3DFRead, zCyl3DRead,
 * zCyl3DFWrite, zCyl3DWrite,
 * - input/output of 3D cylinder.
 *
 * 'zCyl3DFRead()' reads the information of a 3D cylinder
 * from the current position of the file 'fp', and creates
 * the new cylinder 'cyl'.
 * An acceptable data file format is as follows.
 * #
 *  center: <x1> <y1> <z1>
 *  center: <x2> <y2> <z2>
 *  radius: <r>
 *  div: <div>
 * #
 * Each bracketed value must be substituted for a real number.
 * When more than two keywords 'center' exist in the field,
 * 'zCyl3DFRead()' ignore them.
 * 'zCyl3DRead()' reads the information for 'cyl'
 * simply from the standard input.
 * The field div is skippable.
 * #
 * 'zCyl3DFWrite()' writes the information of 'cyl'
 * to the current position of the file 'fp' in the same format
 * with the above. 'zCyl3DWrite()' writes the information
 * of 'cyl' simply to the standard out.
 * [RETURN VALUE]
 * Each of 'zCyl3DFRead()' and 'zCyl3DRead()' returns
 * a pointer 'cyl'.
 * #
 * Neither 'zCyl3DFWrite()' nor 'zCyl3DWrite()' returns
 * any values.
 */
__EXPORT zCyl3D *zCyl3DFRead(FILE *fp, zCyl3D *cyl);
#define zCyl3DRead(c) zCyl3DFRead( stdin, (c) )
__EXPORT void zCyl3DFWrite(FILE *fp, zCyl3D *cyl);
#define zCyl3DWrite(c) zCyl3DFWrite( stdout, (c) )

/* methods for abstraction */
extern zPrimCom zprim_cyl3d_com;

__END_DECLS

#endif /* __ZEO_PRIM_CYL_H__ */
