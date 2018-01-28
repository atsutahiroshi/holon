#include <roki/rkgl_glut.h>
#include <roki/rkgl_camera.h>
#include <roki/rkgl_shape.h>

zOpticalInfo opt;
zShape3D shape;

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
  rkglShape( &shape, NULL, RKGL_FACE );
  glPopMatrix();
  glutSwapBuffers();
}

void idle(void)
{
  glutPostRedisplay();
}

void resize(int w, int h)
{
  rkglVPCreate( &cam, 0, 0, w, h );
  rkglFrustumScale( &cam, 1.0/160, 1, 100 );
}

void keyboard(unsigned char key, int x, int y)
{
  switch( key ){
  case 'u': if( opt.dif.v1 > 0.0 ) opt.dif.v1 -= 0.01; break;
  case 'U': if( opt.dif.v1 < 1.0 ) opt.dif.v1 += 0.01; break;
  case 'i': if( opt.dif.v2 > 0.0 ) opt.dif.v2 -= 0.01; break;
  case 'I': if( opt.dif.v2 < 1.0 ) opt.dif.v2 += 0.01; break;
  case 'o': if( opt.dif.v3 > 0.0 ) opt.dif.v3 -= 0.01; break;
  case 'O': if( opt.dif.v3 < 1.0 ) opt.dif.v3 += 0.01; break;
  case ' ':
    r += 10; break;
  case 'q': case 'Q': case '\033':
    zShape3DDestroy( &shape );
    exit( EXIT_SUCCESS );
  default: ;
  }
}

void init(void)
{
  rkglBGSet( &cam, 0.5, 0.5, 0.5 );
  rkglCASet( &cam, 6, 0, 3, 0, -30, 0 );

  glEnable( GL_LIGHTING );
  rkglLightCreate( &light, 0, 0.8, 0.8, 0.8, 1, 1, 1, 0, 0, 0, 0 );
  rkglLightSetPos( &light, 1, 3, 6 );

  zOpticalInfoCreateSimple( &opt, 0.8, 0, 0, NULL );
  zShape3DInit( &shape );
  zShape3DCreateBoxAlign( &shape, Z_ZEROVEC3D, 5, 3, 4 );
  zShape3DSetOptic( &shape, &opt );
}

int main(int argc, char *argv[])
{
  rkglInitGLUT( &argc, argv );
  rkglWindowCreateGLUT( 0, 0, 640, 480, argv[0] );

  glutDisplayFunc( display );
  glutIdleFunc( idle );
  glutReshapeFunc( resize );
  glutKeyboardFunc( keyboard );
  init();
  glutMainLoop();
  return 0;
}
