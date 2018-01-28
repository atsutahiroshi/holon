#include <roki/rkgl_glut.h>
#include <roki/rkgl_shape.h>

typedef struct{
  GLfloat e[3];
} rkglVec3Df;

static void _rkglVertexf(rkglVec3Df *v);

void _rkglVertexf(rkglVec3Df *v)
{
  glVertex3f( v->e[0], v->e[1], v->e[2] );
}

typedef struct{
  int nu;
  int nv;
  int uslice;
  int vslice;
  rkglVec3Df *ctl;
} rkglBezier;

rkglVec3Df *rkglBezierCtl(rkglBezier *bez, int i, int j)
{
  return &bez->ctl[i*bez->nv+j];
}

rkglBezier *rkglBezierCreate(rkglBezier *bez, int nu, int nv, int uslice, int vslice)
{
  bez->nu = nu;
  bez->nv = nv;
  bez->uslice = uslice;
  bez->vslice = vslice;
  if( !( bez->ctl = zAlloc( rkglVec3Df, nu*nv ) ) ){
    zFree( bez->ctl );
    ZALLOCERROR();
    return NULL;
  }
  return bez;
}

void rkglBezierDestroy(rkglBezier *bez)
{
  zFree( bez->ctl );
}

void rkglBezierDraw(rkglBezier *bez, GLint mode)
{
  glMap2f( GL_MAP2_VERTEX_3,
    0, 1,         3, bez->nv,
    0, 1, bez->nv*3, bez->nu, (GLfloat *)bez->ctl );
  glMapGrid2f( bez->uslice, 0, 1, bez->vslice, 0, 1 );
  glEnable( GL_MAP2_VERTEX_3 );
  glEvalMesh2( mode, 0, bez->uslice, 0, bez->vslice );
}

void rkglBezierCtlDraw(rkglBezier *bez, GLfloat size, GLfloat r, GLfloat g, GLfloat b)
{
  register int i, j;

  glPointSize( size );
  glDisable( GL_LIGHTING );
  glColor3f( r, g, b );
  glBegin( GL_POINTS );
  for( i=0; i<bez->nu; i++ )
    for( j=0; j<bez->nv; j++ )
      _rkglVertexf( rkglBezierCtl(bez,i,j) );
  glEnd();
  for( i=0; i<bez->nu; i++ )
    for( j=0; j<bez->nv; j++ ){
      if( i > 0 ){
        glBegin(GL_LINES);
        _rkglVertexf( rkglBezierCtl(bez,i,j) );
        _rkglVertexf( rkglBezierCtl(bez,i-1,j) );
        glEnd();
      }
      if( j > 0 ){
        glBegin(GL_LINES);
        _rkglVertexf( rkglBezierCtl(bez,i,j) );
        _rkglVertexf( rkglBezierCtl(bez,i,j-1) );
        glEnd();
      }
    }
  glEnable( GL_LIGHTING );
}


void init_ctl(rkglBezier *bez, zOpticalInfo *oi)
{
  register int i, j;

  for( i=0; i<bez->nu; i++ )
    for( j=0; j<bez->nv; j++ ){
      rkglBezierCtl(bez,i,j)->e[0] = 2.0 * ( (GLfloat)j/bez->nv - 0.5 );
      rkglBezierCtl(bez,i,j)->e[1] = 2.0 * ( (GLfloat)i/bez->nu - 0.5 );
      rkglBezierCtl(bez,i,j)->e[2] =
        (i==0||i==bez->nu-1||j==0||j==bez->nv-1) ? 0 : (GLfloat)zRandI(-1,1);
    }
  zOpticalInfoCreateSimple( oi, zRandF(0.0,1.0), zRandF(0.0,1.0), zRandF(0.0,1.0), NULL );
}


rkglCamera cam;
rkglLight light;

zOpticalInfo oi;
rkglBezier bez;

bool show_ctl = false;

void display(void)
{
  rkglCALoad( &cam );
  rkglLightPut( &light );
  rkglClear();

  rkglMaterial( &oi );
  rkglBezierDraw( &bez, GL_FILL );
  if( show_ctl )
    rkglBezierCtlDraw( &bez, 5.0, 0.5, 1.0, 0.5 );
  glutSwapBuffers();
}

void reshape(int w, int h)
{
  rkglVPCreate( &cam, 0, 0, w, h );
  rkglPerspective( &cam, 30.0, (GLdouble)w/(GLdouble)h, 1.0, 20.0 );
}

void keyboard(unsigned char key, int x, int y)
{
  switch( key ){
  case 'q':
    rkglBezierDestroy( &bez );
    exit(1);
    break;
  case 'i':
    init_ctl( &bez, &oi );
    glutPostRedisplay();
    break;
  case 'p':
    show_ctl = 1 - show_ctl;
    glutPostRedisplay();
    break;
  default: ;
  }
}

void init()
{
  zRandInit();
  rkglSetCallbackParamGLUT( &cam, 0, 0, 0, 0, 0 );

  rkglBGSet( &cam, 0.8, 0.8, 0.8 );
  rkglCALookAt( &cam, 3, 0, 5, 0, 0, 0, 0, 0, 1 );

  glEnable(GL_LIGHTING);
  glLightModeli( GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE );
  rkglLightCreate( &light, 0, 0.0, 0.0, 0.0, 1, 1, 1, 0, 0, 0, 0 );
  rkglLightSetPos( &light, 0, 0, 10 );

  glEnable( GL_AUTO_NORMAL );
  glDisable( GL_CULL_FACE );
  rkglBezierCreate( &bez, 8, 12, 30, 30 );
  init_ctl( &bez, &oi );
}

int main(int argc, char *argv[])
{
  rkglInitGLUT( &argc, argv );
  rkglWindowCreateGLUT( 0, 0, 500, 500, argv[0] );

  glutDisplayFunc( display );
  glutReshapeFunc( reshape );
  glutMouseFunc( rkglMouseFuncGLUT );
  glutMotionFunc( rkglMouseDragFuncGLUT );
  glutKeyboardFunc( keyboard );
  init();
  glutMainLoop();
  return 0;
}
