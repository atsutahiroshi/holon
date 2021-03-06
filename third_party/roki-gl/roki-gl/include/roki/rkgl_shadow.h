/* RoKi-GL - Robot Kinetics library: visualization using OpenGL
 * Copyright (C) 2000 Tomomichi Sugihara (Zhidao)
 *
 * rkgl_shadow - shadow map
 */

#ifndef __RKGL_SHADOW_H__
#define __RKGL_SHADOW_H__

#include <roki/rkgl_optic.h>
#include <roki/rkgl_camera.h>

__BEGIN_DECLS

typedef struct{
  int width, height; /* texture size */
  double ratio; /* darkness ratio */
  double radius; /* radius of bounding sphere */
  bool antizfighting; /* flag to work out anti-Z-fighting */
  /*! @cond */
  GLuint tex; /* texture name */
  GLuint fb; /* framebuffer name */
  double _lightview[16];
  /*! @endcond */
} rkglShadow;

void rkglShadowInit(rkglShadow *shadow, int width, int height, double radius, double ratio);
void rkglShadowSetLight(rkglShadow *shadow, rkglLight *light);
void rkglShadowDraw(rkglShadow *shadow, rkglCamera* cam, rkglLight *light, void (* scene)(void));

#define rkglShadowEnableAntiZFighting(s)  ( (s)->antizfighting = true )
#define rkglShadowDisableAntiZFighting(s) ( (s)->antizfighting = false )

__END_DECLS

#endif /* __RKGL_SHADOW_H__ */
