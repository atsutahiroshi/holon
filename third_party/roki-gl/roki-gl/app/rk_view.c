/* rk_view - 3D geometry viewer */

#include <unistd.h>
#include <cure/cure_option.h>
#include <roki/rokigl.h>
#include <roki/rkgl_glx.h>
#include <zx11/zximage_png.h>

#define RK_VIEW_TITLE "RK-VIEW"

enum{
  OPT_MODELFILE=0,
  OPT_PAN, OPT_TILT, OPT_ROLL, OPT_OX, OPT_OY, OPT_OZ,
  OPT_WIDTH, OPT_HEIGHT,
  OPT_WIREFRAME,
  OPT_BG,
  OPT_LX, OPT_LY, OPT_LZ,
  OPT_SMOOTH, OPT_FOG, OPT_SHADOW,
  OPT_CAPTURE,
  OPT_HELP,
  OPT_INVALID
};
zOption opt[] = {
  { "model", NULL, "<.z3d file>", "geometric model file", NULL, false },
  { "pan", NULL, "<pan value>", "set camera pan angle", (char *)"0", false },
  { "tilt", NULL, "<tilt value>", "set camera tilt angle", (char *)"0", false },
  { "roll", NULL, "<roll value>", "set camera roll angle", (char *)"0", false },
  { "x", NULL, "<value>", "camera position in x axis", (char *)"5", false },
  { "y", NULL, "<value>", "camera position in y axis", (char *)"0", false },
  { "z", NULL, "<value>", "camera position in z axis", (char *)"0", false },
  { "width", NULL, "<width>", "set window width", (char *)"500", false },
  { "height", NULL, "<height>", "set window height", (char *)"500", false },
  { "wireframe", NULL, NULL, "draw objects as wireframe models", NULL, false },
  { "bg", NULL, "<RGB#hex>", "set background color", (char *)"#010101", false },
  { "lx", NULL, "<value>", "light position in x axis", (char *)"3", false },
  { "ly", NULL, "<value>", "light position in y axis", (char *)"0", false },
  { "lz", NULL, "<value>", "light position in z axis", (char *)"3", false },
  { "smooth", NULL, NULL, "enable antialias", NULL, false },
  { "fog", NULL, NULL, "enable fog", NULL, false },
  { "shadow", NULL, NULL, "enable shadow", NULL, false },
  { "xwd", NULL, "<suf>", "output image format suffix", (char *)"png", false },
  { "help", NULL, NULL, "show this message", NULL, false },
  { NULL, NULL, NULL, NULL, NULL, false },
};

#define RK_VIEW_BUFSIZ 512

Window win;

rkglCamera cam;
rkglLight light;
rkglShadow shadow;
int model = -1;

void rk_viewUsage(void)
{
  eprintf( "Usage: rk_view <options>\n" );
  eprintf( "<options>\n" );
  zOptionHelp( opt );
  exit( 0 );
}

void rk_viewCapture(void)
{
  zxRegion reg;
  zxImage img;
  static char imgfile[RK_VIEW_BUFSIZ];
  static int count = 0;

  sprintf( imgfile, "%04d.%s", count++, opt[OPT_CAPTURE].arg );
  zxGetGeometry( win, &reg );
  zxImageAllocDefault( &img, reg.width, reg.height );
  zxImageFromPixmap( &img, win, img.width, img.height );
  zxImageWritePNGFile( &img, imgfile );
  zxImageDestroy( &img );
}

/**********************************************************/

void rk_viewDraw(void)
{
  glCallList( model );
}

void rk_viewDisplay(void)
{
  rkglActivateGLX( win );
  if( opt[OPT_SHADOW].flag ){
    /* shadow-map rendering */
    rkglShadowDraw( &shadow, &cam, &light, rk_viewDraw );
  } else{
    /* non-shadowed rendering */
    rkglClear();
    rkglCALoad( &cam );
    rkglLightPut( &light );
      rk_viewDraw();
  }
  rkglSwapBuffersGLX( win );
  rkglFlushGLX();
}

void rk_viewInit(void)
{
  zRGB rgb;
  zMShape3D ms;

  win = rkglWindowCreateGLX( NULL, 0, 0, atoi(opt[OPT_WIDTH].arg), atoi(opt[OPT_HEIGHT].arg), RK_VIEW_TITLE );
  rkglKeyEnableGLX( win );
  rkglMouseEnableGLX( win );
  rkglWindowOpenGLX( win );

  zRGBDec( &rgb, opt[OPT_BG].arg );
  rkglBGSet( &cam, rgb.r, rgb.g, rgb.b );
  rkglVPCreate( &cam, 0, 0,
    atoi(opt[OPT_WIDTH].arg), atoi(opt[OPT_HEIGHT].arg) );
  rkglCASet( &cam,
    atof(opt[OPT_OX].arg), atof(opt[OPT_OY].arg), atof(opt[OPT_OZ].arg),
    atof(opt[OPT_PAN].arg), atof(opt[OPT_TILT].arg), atof(opt[OPT_ROLL].arg) );

  glEnable( GL_LIGHTING );
  rkglLightCreate( &light, 0, 0.5, 0.5, 0.5, 0.8, 0.8, 0.8, 0, 0, 0, 0 );
  rkglLightSetPos( &light,
    atof(opt[OPT_LX].arg), atof(opt[OPT_LY].arg), atof(opt[OPT_LZ].arg) );
  rkglShadowInit( &shadow, 512, 512, 1.5, 0.2 );

  if( !zMShape3DReadFile( &ms, opt[OPT_MODELFILE].arg ) ){
    ZOPENERROR( opt[OPT_MODELFILE].arg );
    rk_viewUsage();
    exit( 1 );
  }
  model = rkglMShapeEntry( &ms,
    opt[OPT_WIREFRAME].flag ? RKGL_WIREFRAME : RKGL_FACE );
  zMShape3DDestroy( &ms );
  if( model < 0 ) exit( 1 );

  if( opt[OPT_SMOOTH].flag ) glEnable( GL_LINE_SMOOTH );
  if( opt[OPT_FOG].flag ) rkglBGFog( &cam, 0.1 );
}

bool rk_viewCommandArgs(int argc, char *argv[])
{
  zStrList arglist;
  char *modelfile;

  if( argc <= 1 ) rk_viewUsage();
  zOptionRead( opt, argv, &arglist );
  zStrListGetPtr( &arglist, 1, &modelfile );
  if( opt[OPT_HELP].flag ) rk_viewUsage();
  if( modelfile ){
    opt[OPT_MODELFILE].flag = true;
    opt[OPT_MODELFILE].arg  = modelfile;
  }
  if( !opt[OPT_MODELFILE].flag ){
    ZRUNERROR( "model not assigned" );
    return false;
  }
  rk_viewInit();
  zStrListDestroy( &arglist, false );
  return true;
}

void rk_viewExit(void)
{
  rkglWindowCloseGLX( win );
  rkglCloseGLX();
}

/**********************************************************/
void rk_viewReshape(void)
{
  zxRegion reg;
  double x, y;

  zxGetGeometry( win, &reg );
  x = 0.1;
  rkglVPCreate( &cam, 0, 0, reg.width, reg.height );
  y = x / rkglVPAspect(&cam);
  rkglFrustum( &cam, -x, x, -y, y, 1, 20 );
}

enum{ RK_VIEW_CAM_ROT, RK_VIEW_CAM_PAN, RK_VIEW_CAM_ZOOM };

static int mouse_button = -1;
static int mousex, mousey;
static byte cammode = RK_VIEW_CAM_ROT;

void rk_viewStoreMouseInfo(byte mode)
{
  mouse_button = zxMouseButton;
  mousex = zxMouseX;
  mousey = zxMouseY;
  cammode = mode;
}

void rk_viewMousePress(void)
{
  switch( zxMouseButton ){
  case Button1: rk_viewStoreMouseInfo( RK_VIEW_CAM_ROT );  break;
  case Button3: rk_viewStoreMouseInfo( RK_VIEW_CAM_PAN );  break;
  case Button2: rk_viewStoreMouseInfo( RK_VIEW_CAM_ZOOM ); break;
  case Button4: rkglCARelMove( &cam,-0.1, 0, 0 );          break;
  case Button5: rkglCARelMove( &cam, 0.1, 0, 0 );          break;
  default: ;
  }
}

void rk_viewMouseRelease(void)
{
  mouse_button = -1;
}

void rk_viewMouseDrag(void)
{
  double dx, dy, r;

  if( mouse_button == -1 ) return;
  dx = (double)( zxMouseX - mousex ) / cam.vp[3];
  dy =-(double)( zxMouseY - mousey ) / cam.vp[2];
  switch( cammode ){
  case RK_VIEW_CAM_ROT:
    r = 180 * sqrt( dx*dx + dy*dy );
    zxModkeyCtrlIsOn() ?
      rkglCARotate( &cam, r, -dy, dx, 0 ) :
      rkglCALockonRotate( &cam, r, -dy, dx, 0 );
    break;
  case RK_VIEW_CAM_PAN:
    zxModkeyCtrlIsOn() ?
      rkglCAMove( &cam, 0, dx, dy ) :
      rkglCARelMove( &cam, 0, dx, dy );
    break;
  case RK_VIEW_CAM_ZOOM:
    zxModkeyCtrlIsOn() ?
      rkglCAMove( &cam, -dy, 0, 0 ) :
      rkglCARelMove( &cam, -2*dy, 0, 0 );
    break;
  default: ;
  }
  mousex = zxMouseX;
  mousey = zxMouseY;
}

int rk_viewKeyPress(void)
{
  static bool from_light = false;

  zxModkeyOn( zxKeySymbol() );
  switch( zxKeySymbol() ){
  case XK_l: /* toggle viewpoint to light/camera */
    if( ( from_light = 1 - from_light ) ){
      rkglCALookAt( &cam,
        atof(opt[OPT_LX].arg), atof(opt[OPT_LY].arg), atof(opt[OPT_LZ].arg),
        0, 0, 0, -1, 0, 1 );
    } else{
      rkglCASet( &cam,
        atof(opt[OPT_OX].arg), atof(opt[OPT_OY].arg), atof(opt[OPT_OZ].arg),
        atof(opt[OPT_PAN].arg), atof(opt[OPT_TILT].arg), atof(opt[OPT_ROLL].arg) );
    }
    break;
  case XK_c:
    rk_viewCapture();
    break;
  case XK_q:
    eprintf( "quit.\n" );
    return -1;
  }
  return 0;
}

int rk_viewEvent(void)
{
  switch( zxDequeueEvent() ){
  case Expose:
  case ConfigureNotify: rk_viewReshape();             break;
  case ButtonPress:     rk_viewMousePress();          break;
  case MotionNotify:    rk_viewMouseDrag();           break;
  case ButtonRelease:   rk_viewMouseRelease();        break;
  case KeyPress:
    if( rk_viewKeyPress() < 0 ) return -1;            break;
  case KeyRelease:      zxModkeyOff( zxKeySymbol() ); break;
  default: ;
  }
  return 0;
}

void rk_viewLoop(void)
{
#define RK_VIEW_LOOP_SKIP 5
  int count = 0;

  /* initial draw */
  rk_viewReshape();
  rk_viewDisplay();
  while( rk_viewEvent() == 0 ){
    if( count++ > RK_VIEW_LOOP_SKIP ){
      rk_viewDisplay();
      count = 0;
    }
  }
}

int main(int argc, char *argv[])
{
  rkglInitGLX();
  if( !rk_viewCommandArgs( argc, argv+1 ) ) return 1;
  rk_viewLoop();
  rk_viewExit();
  return 0;
}
