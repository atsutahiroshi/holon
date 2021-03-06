/* RoKi-GL - Robot Kinetics library: visualization using OpenGL
 * Copyright (C) 2000 Tomomichi Sugihara (Zhidao)
 *
 * rkgl_misc - miscellanies
 */

#ifndef __RKGL_MISC_H__
#define __RKGL_MISC_H__

#include <cure/cure.h>

#include <GL/gl.h>
#include <GL/glu.h>

__BEGIN_DECLS

void rkglEnableDefault(void);

#define rkglClear() glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT )
#define rkglClearColor(r,g,b) do{\
  glClearColor( r, g, b, 1.0 );\
  rkglClear();\
} while(0)

/* matrix operation */

/* why does not OpenGL have glMultInvMatrixd() ? */
void rkglInvTranslated(double m[], double *x, double *y, double *z);
void rkglMultInvMatrixd(double m[]);

__END_DECLS

#endif /* __RKGL_MISC_H__ */
