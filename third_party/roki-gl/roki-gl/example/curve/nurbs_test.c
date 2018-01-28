#include <roki/rkgl_shape.h>
#include <roki/rkgl_glut.h>

typedef struct{
  GLfloat e[3];
} rkglVec3Df;

static void _rkglVertexf(rkglVec3Df *v);

void _rkglVertexf(rkglVec3Df *v)
{
  glVertex3f( v->e[0], v->e[1], v->e[2] );
}

typedef struct{
  GLUnurbsObj *obj;
  int ns;
  int nt;
  int sdim;
  int tdim;
  rkglVec3Df *ctl;
  int nsknot;
  int ntknot;
  GLfloat *sknot;
  GLfloat *tknot;
} rkglNURBS;

rkglVec3Df *rkglNURBSCtl(rkglNURBS *nurbs, int i, int j)
{
  return &nurbs->ctl[i*nurbs->nt+j];
}

#define RKGL_NURBS_DEFAULT_TOL 10.0
rkglNURBS *rkglNURBSCreate(rkglNURBS *nurbs, int ns, int nt, int sdim, int tdim)
{
  if( !( nurbs->obj = gluNewNurbsRenderer() ) ) return NULL;
  gluNurbsProperty( nurbs->obj, GLU_SAMPLING_TOLERANCE, RKGL_NURBS_DEFAULT_TOL );
  nurbs->ns = ns;
  nurbs->nt = nt;
  nurbs->sdim = sdim;
  nurbs->tdim = tdim;
  nurbs->nsknot = ns + sdim;
  nurbs->ntknot = nt + tdim;

  nurbs->ctl = zAlloc( rkglVec3Df, ns*nt );
  nurbs->sknot = zAlloc( GLfloat, nurbs->nsknot );
  nurbs->tknot = zAlloc( GLfloat, nurbs->ntknot );
  if( !nurbs->ctl || !nurbs->sknot || !nurbs->tknot ){
    zFree( nurbs->ctl );
    zFree( nurbs->sknot );
    zFree( nurbs->tknot );
    ZALLOCERROR();
    return NULL;
  }
  return nurbs;
}

/* uniform B-spline
 */
rkglNURBS *rkglNURBSCreateUBS(rkglNURBS *nurbs, int ns, int nt, int sdim, int tdim)
{
  register int i;

  if( !( nurbs = rkglNURBSCreate( nurbs, ns, nt, sdim, tdim ) ) )
    return NULL;
  for( i=0; i<nurbs->nsknot; i++ )
    nurbs->sknot[i] = (GLfloat)i;
  for( i=0; i<nurbs->ntknot; i++ )
    nurbs->tknot[i] = (GLfloat)i;
  return nurbs;
}

rkglNURBS *rkglNURBSCreateBezier(rkglNURBS *nurbs, int ns, int nt)
{
  register int i;

  if( !( nurbs = rkglNURBSCreate( nurbs, ns, nt, ns, nt ) ) )
    return NULL;
  for( i=0; i<nurbs->nsknot; i++ )
    nurbs->sknot[i] = i < nurbs->sdim ? 0.0 : 1.0;
  for( i=0; i<nurbs->ntknot; i++ )
    nurbs->tknot[i] = i < nurbs->tdim ? 0.0 : 1.0;
  return nurbs;
}

void rkglNURBSDestroy(rkglNURBS *nurbs)
{
  gluDeleteNurbsRenderer( nurbs->obj );
  zFree( nurbs->ctl );
}

void rkglNURBSDraw(rkglNURBS *nurbs, GLint mode)
{
  gluNurbsProperty( nurbs->obj, GLU_DISPLAY_MODE, mode );

  gluBeginSurface( nurbs->obj );
  gluNurbsSurface( nurbs->obj,
    nurbs->nsknot, nurbs->sknot,
    nurbs->ntknot, nurbs->tknot,
    nurbs->nt*3, 3, (GLfloat *)nurbs->ctl,
    nurbs->sdim, nurbs->tdim,
    GL_MAP2_VERTEX_3 );
  gluEndSurface( nurbs->obj );
}

void rkglNURBSCtlDraw(rkglNURBS *nurbs, GLfloat size, GLfloat r, GLfloat g, GLfloat b)
{
  register int i, j;

  glPointSize( size );
  glDisable( GL_LIGHTING );
  glColor3f( r, g, b );
  glBegin( GL_POINTS );
  for( i=0; i<nurbs->ns; i++ )
    for( j=0; j<nurbs->nt; j++ )
      _rkglVertexf( rkglNURBSCtl(nurbs,i,j) );
  glEnd();
  for( i=0; i<nurbs->ns; i++ )
    for( j=0; j<nurbs->nt; j++ ){
      if( i > 0 ){
        glBegin(GL_LINES);
        _rkglVertexf( rkglNURBSCtl(nurbs,i,j) );
        _rkglVertexf( rkglNURBSCtl(nurbs,i-1,j) );
        glEnd();
      }
      if( j > 0 ){
        glBegin(GL_LINES);
        _rkglVertexf( rkglNURBSCtl(nurbs,i,j) );
        _rkglVertexf( rkglNURBSCtl(nurbs,i,j-1) );
        glEnd();
      }
    }
  glEnable( GL_LIGHTING );
}


/* test surface */
void init_surface(rkglNURBS *nurbs, zOpticalInfo *oi)
{
  int i, j;

  for( i=0; i<nurbs->ns; i++ )
    for( j=0; j<nurbs->nt; j++ ){
      rkglNURBSCtl(nurbs,i,j)->e[0] = 4.0 * ( (GLfloat)i/nurbs->ns - 0.5 );
      rkglNURBSCtl(nurbs,i,j)->e[1] = 4.0 * ( (GLfloat)j/nurbs->nt - 0.5 );
      rkglNURBSCtl(nurbs,i,j)->e[2] =
        (i==0||i==nurbs->ns-1||j==0||j==nurbs->nt-1) ? 0 : zRandF( -1.0, 1.0 );
    }
  zOpticalInfoCreateSimple( oi, zRandF(0.0,1.0), zRandF(0.0,1.0), zRandF(0.0,1.0), NULL );
}


rkglCamera cam;
rkglLight light;

zOpticalInfo oi;
rkglNURBS nurbs;

bool show_ctl = false;

void keyboard(unsigned char key, int x, int y)
{
  switch( key ){
  case 'q':
    rkglNURBSDestroy( &nurbs );
    exit(1);
    break;
  case 'i':
    init_surface( &nurbs, &oi );
    glutPostRedisplay();
    break;
  case 'p':
    show_ctl = 1 - show_ctl;
    glutPostRedisplay();
    break;
  default: ;
  }
}

void display(void)
{
  rkglCALoad( &cam );
  rkglLightPut( &light );
  rkglClear();
  glPushMatrix();

  rkglMaterial( &oi );
  rkglNURBSDraw( &nurbs, GLU_FILL );
  /*
  rkglNURBSDraw( &nurbs, GLU_OUTLINE_PATCH );
  rkglNURBSDraw( &nurbs, GLU_OUTLINE_POLYGON );
  */
  if( show_ctl )
    rkglNURBSCtlDraw( &nurbs, 5.0, 0.5, 1.0, 0.5 );

  glPopMatrix();
  glutSwapBuffers();
}

void reshape(int w, int h)
{
  rkglVPCreate( &cam, 0, 0, w, h );
  rkglPerspective( &cam, 45.0, (GLdouble)w/(GLdouble)h, 1.0, 10.0 );
}

void idle(void){ glutPostRedisplay(); }

void init(void)
{
  zRandInit();
  rkglSetCallbackParamGLUT( &cam, 0, 0, 0, 0, 0 );

  rkglBGSet( &cam, 0.0, 0.0, 0.0 );
  rkglCALookAt( &cam, 5, 0, 1, 0, 0, 0, 0, 0, 1 );

  glEnable( GL_LIGHTING );
  glLightModeli( GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE );
  glEnable( GL_AUTO_NORMAL );
  glDisable( GL_CULL_FACE );
  rkglLightCreate( &light, 0, 0.0, 0.0, 0.0, 1, 1, 1, 0, 0, 0, 0 );
  rkglLightSetPos( &light, 0, 0, 10 );

#if 1
  rkglNURBSCreateUBS( &nurbs, 8, 8, 4, 4 );
#else
  rkglNURBSCreateBezier( &nurbs, 8, 8 );
#endif
  init_surface( &nurbs, &oi );
}

int main(int argc, char **argv)
{
  rkglInitGLUT( &argc, argv );
  rkglWindowCreateGLUT( 0, 0, 500, 500, argv[0] );

  glutDisplayFunc( display );
  glutIdleFunc( idle );
  glutReshapeFunc( reshape );
  glutMouseFunc( rkglMouseFuncGLUT );
  glutMotionFunc( rkglMouseDragFuncGLUT );
  glutKeyboardFunc( keyboard );
  init();
  glutMainLoop();
  return 0;
}
