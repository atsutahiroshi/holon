#include <cure/cure_misc.h>

int main(void)
{
  ZRUNERROR( "error test" );
  ZRUNWARN( "warning test" );
  ZRUNERROR( "error test %d ", 2 );
  ZRUNWARN( "warning test %f", 3.0 );

  eprintf( ">> echo-off\n" );
  zEchoOff();
  ZRUNERROR( "error test" );
  ZRUNWARN( "warning test" );
  ZRUNERROR( "error test %d ", 2 );
  ZRUNWARN( "warning test %f", 3.0 );

  eprintf( ">> echo-on\n" );
  zEchoOn();
  ZRUNERROR( "error test" );
  ZRUNWARN( "warning test" );
  ZRUNERROR( "error test %d ", 2 );
  ZRUNWARN( "warning test %f", 3.0 );
  return 0;
}
