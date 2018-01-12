/* Zeo - Z/Geometry and optics computation library.
 * Copyright (C) 2005 Tomomichi Sugihara (Zhidao)
 *
 * zeo_optic - optical properties
 */

#include <zeo/zeo_optic.h>

/* ********************************************************** */
/* CLASS: zOpticalInfo
 * class for the optical characteristic parameter set
 * ********************************************************** */

static bool _zOpticalInfoFRead(FILE *fp, void *instance, char *buf, bool *success);

/* zOpticalInfoCreate
 * - create a set of optical parameters.
 */
zOpticalInfo *zOpticalInfoCreate(zOpticalInfo *oi, float ar, float ag, float ab, float dr, float dg, float db, float sr, float sg, float sb, double ns, double sns, double alpha, char *name)
{
  zNameSet( oi, name );
  zRGBSet( zOpticalInfoAmb(oi), ar, ag, ab );
  zRGBSet( zOpticalInfoDif(oi), dr, dg, db );
  zRGBSet( zOpticalInfoSpc(oi), sr, sg, sb );
  zOpticalInfoSetExp( oi, ns );
  zOpticalInfoSetSns( oi, sns );
  zOpticalInfoSetAlpha( oi, alpha );
  return oi;
}

/* zOpticalInfoCopy
 * - copy a set of optical parameters.
 */
zOpticalInfo *zOpticalInfoCopy(zOpticalInfo *src, zOpticalInfo *dest)
{
  zOpticalInfoSetAmb( dest, zOpticalInfoAmb(src) );
  zOpticalInfoSetDif( dest, zOpticalInfoDif(src) );
  zOpticalInfoSetSpc( dest, zOpticalInfoSpc(src) );
  zOpticalInfoSetExp( dest, zOpticalInfoExp(src) );
  zOpticalInfoSetSns( dest, zOpticalInfoSns(src) );
  zOpticalInfoSetAlpha( dest, zOpticalInfoAlpha(src) );
  return dest;
}

/* zOpticalInfoClone
 * - clone a set of optical parameters.
 */
zOpticalInfo *zOpticalInfoClone(zOpticalInfo *src, zOpticalInfo *dest)
{
  return zNameSet( dest, zName(src) ) ?
    zOpticalInfoCopy( src, dest ) : NULL;
}

/* zOpticalInfoMul
 * - multiply a set of optical parameters to another.
 */
zOpticalInfo *zOpticalInfoMul(zOpticalInfo *oi1, zOpticalInfo *oi2, zOpticalInfo *oi)
{
  zRGBMul( zOpticalInfoAmb(oi1), zOpticalInfoAmb(oi2), zOpticalInfoAmb(oi) );
  zRGBMul( zOpticalInfoDif(oi1), zOpticalInfoDif(oi2), zOpticalInfoDif(oi) );
  zRGBMul( zOpticalInfoSpc(oi2), zOpticalInfoSpc(oi2), zOpticalInfoSpc(oi) );
  zOpticalInfoSetExp( oi, zOpticalInfoExp(oi1) * zOpticalInfoExp(oi2) );
  zOpticalInfoSetSns( oi, zOpticalInfoSns(oi1) * zOpticalInfoSns(oi2) );
  zOpticalInfoSetAlpha( oi, zOpticalInfoAlpha(oi1) * zOpticalInfoAlpha(oi2) );
  return oi;
}

/* (static)
 * _zOpticalInfoFRead
 * - read information of the optical parameter set from a stream.
 */
bool _zOpticalInfoFRead(FILE *fp, void *instance, char *buf, bool *success)
{
  if( strcmp( buf, "name" ) == 0 )
    zNameSet( (zOpticalInfo *)instance, zFToken( fp, buf, BUFSIZ ) );
  else if( strcmp( buf, "ambient" ) == 0 )
    zRGBFRead( fp, zOpticalInfoAmb((zOpticalInfo *)instance) );
  else if( strcmp( buf, "diffuse" ) == 0 )
    zRGBFRead( fp, zOpticalInfoDif((zOpticalInfo *)instance) );
  else if( strcmp( buf, "specular" ) == 0 )
    zRGBFRead( fp, zOpticalInfoSpc((zOpticalInfo *)instance) );
  else if( strcmp( buf, "exp" ) == 0 )
    zOpticalInfoSetExp( (zOpticalInfo *)instance, zFDouble( fp ) );
  else if( strcmp( buf, "shininess" ) == 0 )
    zOpticalInfoSetSns( (zOpticalInfo *)instance, zFDouble( fp ) );
  else if( strcmp( buf, "alpha" ) == 0 )
    zOpticalInfoSetAlpha( (zOpticalInfo *)instance, zFDouble( fp ) );
  else
    return false;
  return true;
}

/* zOpticalInfoFRead
 * - read information of the optical parameter set from a stream.
 */
zOpticalInfo *zOpticalInfoFRead(FILE *fp, zOpticalInfo *oi)
{
  zOpticalInfoInit( oi );
  zFieldFRead( fp, _zOpticalInfoFRead, oi );
  if( zNamePtr( oi ) ) return oi;
  ZRUNERROR( ZEO_ERR_OPT_UNNAME );
  return NULL;
}

/* zOpticalInfoFWrite
 * - output of the information of the optical parameter set.
 */
void zOpticalInfoFWrite(FILE *fp, zOpticalInfo *oi)
{
  fprintf( fp, "name: %s\n", zName(oi) );
  fprintf( fp, "ambient: " );
  zRGBFWrite( fp, zOpticalInfoAmb(oi) );
  fprintf( fp, "diffuse: " );
  zRGBFWrite( fp, zOpticalInfoDif(oi) );
  fprintf( fp, "specular: " );
  zRGBFWrite( fp, zOpticalInfoSpc(oi) );
  fprintf( fp, "exp: %.10g\n", zOpticalInfoExp(oi) );
  fprintf( fp, "shininess: %.10g\n", zOpticalInfoSns(oi) );
  fprintf( fp, "alpha: %.10g\n", zOpticalInfoAlpha(oi) );
  fprintf( fp, "\n" );
}

/* zOpticalInfoFWriteXML
 * - xml output.
 */
void zOpticalInfoFWriteXML(FILE *fp, zOpticalInfo *oi, int indent)
{
  zIndent( indent ); fprintf( fp, "<optic name=\"%s\"\n", zName(oi) );
  zIndent( indent ); fprintf( fp, "       ambient=" ); zRGBFWriteXML(fp,zOpticalInfoAmb(oi)); fprintf( fp, "\n" );
  zIndent( indent ); fprintf( fp, "       diffuse=" ); zRGBFWriteXML(fp,zOpticalInfoDif(oi)); fprintf( fp, "\n" );
  zIndent( indent ); fprintf( fp, "       specular=" ); zRGBFWriteXML(fp,zOpticalInfoSpc(oi)); fprintf( fp, "\n" );
  zIndent( indent ); fprintf( fp, "       alpha=\"%f\"\n", zOpticalInfoAlpha(oi) );
  zIndent( indent ); fprintf( fp, "       exp=\"%f\"/>\n", zOpticalInfoExp(oi) );
  zIndent( indent ); fprintf( fp, "       shininess=\"%f\"/>\n", zOpticalInfoSns(oi) );
}
