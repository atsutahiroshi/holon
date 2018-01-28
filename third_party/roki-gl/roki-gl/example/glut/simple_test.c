#include <roki/rkgl_glut.h>

void display(void)
{
  rkglClear();
  glColor3d( 1.0, 0.0, 0.0 );
  glBegin( GL_POLYGON );
  glVertex2d(-0.9,-0.9 );
  glVertex2d( 0.9,-0.9 );
  glVertex2d( 0.9, 0.9 );
  glVertex2d(-0.9, 0.9 );
  glEnd();
  glutSwapBuffers();
}

void init(void)
{
  glClearColor( 1, 1, 1, 1 );
}

int main(int argc, char *argv[])
{
  rkglInitGLUT( &argc, argv );
  rkglWindowCreateGLUT( 0, 0, 320, 240, argv[0] );
  glutDisplayFunc( display );
  init();
  glutMainLoop();
  return 0;
}
