#include <zx11/zxwidget.h>

void *fA(void){ printf( "selected tab A.\n" ); return NULL; }
void *fB(void){ printf( "selected tab B.\n" ); return NULL; }
void *fC(void){ printf( "selected tab C.\n" ); return NULL; }
void *fD(void){ printf( "selected tab D.\n" ); return NULL; }

int main(void)
{
  zxWindow win;
  zxwTabGroup tg;
  register int i;

  zxInit();
  zxWindowCreateAndOpen( &win, 50, 50, 320, 80 );
  while( zxGetEvent() != Expose );
  zxWidgetInit( &win );
  zxWindowSetBG( &win, "lightgray" );

  zxwTabGroupInit( &tg, 20, 20, 280 );
  zxwTabGroupAdd( &tg, "tab A", fA );
  zxwTabGroupAdd( &tg, "tab B", fB );
  zxwTabGroupAdd( &tg, "tab C", fC );
  zxwTabGroupAdd( &tg, "tab D", fD );
  zxwTabGroupDraw( &win, &tg );

  for( i=0; i<tg.num; i++ ){
    tg.active = i;
    zxwTabGroupDrawPress( &win, &tg );
    zxFlush();
    getchar();
  }
  zxwTabGroupDestroy( &tg );
  zxClose();
  return 0;
}
