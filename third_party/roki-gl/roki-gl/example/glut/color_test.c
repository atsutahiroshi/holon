#include <roki/rkgl_glut.h>

GLdouble vertex[][3] = {
  { 0, 0, 0 },
  { 1, 0, 0 },
  { 1, 1, 0 },
  { 0, 1, 0 },
  { 0, 0, 1 },
  { 1, 0, 1 },
  { 1, 1, 1 },
  { 0, 1, 1 },
};

int face[][4] = {
  { 0, 1, 2, 3 },
  { 1, 5, 6, 2 },
  { 5, 4, 7, 6 },
  { 4, 0, 3, 7 },
  { 4, 5, 1, 0 },
  { 3, 2, 6, 7 },
};

GLdouble color[][4] = {
  { 1, 0, 0 },
  { 0, 1, 0 },
  { 0, 0, 1 },
  { 1, 1, 0 },
  { 1, 0, 1 },
  { 0, 1, 1 },
};

void display(void)
{
  int i, j;
  static double r = 0;

  rkglClear();

  glMatrixMode( GL_MODELVIEW );
  glLoadIdentity();
  gluLookAt( 3.0, 4.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0 );

  glRotated( r, 0, 1, 0 );
  glBegin( GL_QUADS );
  for( i=0; i<6; i++ ){
    glColor3dv( color[i] );
    for( j=0; j<4; j++ )
      glVertex3dv( vertex[face[i][j]] );
  }
  glEnd();
  glutSwapBuffers();

  if( ( r+=0.1 ) >= 360 ) r = 0;
}

void idle(void)
{
  glutPostRedisplay();
}

void resize(int w, int h)
{
  double aspect, height, width;

  glViewport( 0, 0, w, h );
  glMatrixMode( GL_PROJECTION );
  glLoadIdentity();
  aspect = (double)w / (double)h;
  height = 1.0 / 400 * h;
  width = height * aspect;
  /* keep size */
  glFrustum( -0.5*width, 0.5*width, -0.5*height, 0.5*height, 1.0, 10.0 );
}

void keyboard(unsigned char key, int x, int y)
{
  switch( key ){
  case 'q': case 'Q': case '\033': exit( EXIT_SUCCESS );
  default: ;
  }
}

void init(void)
{
  glClearColor( 1, 1, 1, 1 );
  glCullFace( GL_FRONT );
}

int main(int argc, char *argv[])
{
  rkglInitGLUT( &argc, argv );
  rkglWindowCreateGLUT( 0, 0, 320, 240, argv[0] );

  glutDisplayFunc( display );
  glutIdleFunc( idle );
  glutReshapeFunc( resize );
  glutKeyboardFunc( keyboard );
  init();
  glutMainLoop();
  return 0;
}
