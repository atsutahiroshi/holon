#include <roki/rkgl_glut.h>
#include <roki/rkgl_camera.h>
#include <roki/rkgl_shape.h>

int ms_id;
zMShape3D ms;
rkglCamera cam;
rkglLight light;

double r = 0;

void display(void)
{
  rkglCALoad( &cam );
  rkglLightPut( &light );

  glPushMatrix();
  glRotated( r, 0, 0, 1 );
  rkglClear();
  glCallList( ms_id );
  glPopMatrix();
  glutSwapBuffers();
}

void init(void)
{
  rkglBGSet( &cam, 0.6, 0.6, 0.6 );
  rkglCASet( &cam, 1, 0, 0.5, 0, -30, 0 );
  rkglSetCallbackParamGLUT( &cam, 0.05, 0.1, 5.0, 0.05, 5 );

  glEnable( GL_LIGHTING );
  rkglLightCreate( &light, 0, 0.4, 0.4, 0.4, 1, 1, 1, 0, 0, 0, 0 );
  rkglLightSetPos( &light, 1, 3, 6 );

  zMShape3DReadFile( &ms, "../model/scc.z3d" );
  ms_id = rkglMShapeEntry( &ms, RKGL_FACE );
  /*
  ms_id = rkglMShapeEntry( &ms, RKGL_WIREFRAME );
  */
}

int main(int argc, char *argv[])
{
  rkglInitGLUT( &argc, argv );
  rkglWindowCreateGLUT( 0, 0, 320, 240, argv[0] );

  glutDisplayFunc( display );
  glutIdleFunc( rkglIdleFuncGLUT );
  glutVisibilityFunc( rkglVisFuncGLUT );
  glutReshapeFunc( rkglReshapeFuncGLUT );
  glutKeyboardFunc( rkglKeyFuncGLUT );
  glutSpecialFunc( rkglSpecialFuncGLUT );
  glutMouseFunc( rkglMouseFuncGLUT );
  glutMotionFunc( rkglMouseDragFuncGLUT );
  init();
  glutMainLoop();
  return 0;
}
