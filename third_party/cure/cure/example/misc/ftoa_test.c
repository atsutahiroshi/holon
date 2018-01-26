#include <cure/cure_string.h>

char *_itoa(int val)
{
  static char buf[BUFSIZ];

  return itoa( val, buf );
}

char *_ftoa(double val)
{
  static char buf[BUFSIZ];

  return ftoa( val, buf );
}

int main(void)
{
  char buf[BUFSIZ];

  printf( "0 = %s\n", _itoa( 0 ) );
  printf( "0 = %s\n", _ftoa( 0 ) );
  printf( "123456789 = %s\n", _itoa( 123456789 ) );
  printf( "123.456 = %s\n", _ftoa( 123.456 ) );
  printf( "123456789 = %s\n", _ftoa( 123456789 ) );
  printf( "1.234567 = %s\n", _ftoa( 1.234567 ) );
  printf( "0.12345 = %s\n", _ftoa( 0.12345 ) );
  printf( "0.0123 = %s\n", _ftoa( 0.0123 ) );
  printf( "0.00000123456 = %s\n", _ftoa( 0.00000123456 ) );
  printf( "1.2345e-7 = %s\n", _ftoa( 1.2345e-7 ) );
  printf( "1.2345e-12 = %s\n", _ftoa( 1.2345e-12 ) );
  printf( "1.2345e7 = %s\n", _ftoa( 1.2345e7 ) );
  printf( "1.2345e12 = %s\n", _ftoa( 1.2345e12 ) );
  printf( "1.999999999 = %s\n", _ftoa( 1.999999999 ) );
  printf( "1.399999999 = %s\n", _ftoa( 1.399999999 ) );
  printf( "1.999999999999 = %s\n", _ftoa( 1.999999999999 ) );
  printf( "1.399999999999 = %s\n", _ftoa( 1.399999999999 ) );
  printf( "0.0001999999999 = %s\n", _ftoa( 0.0001999999999 ) );
  printf( "0.00001 = %s\n", _ftoa( 0.00001 ) );
  printf( "0.000001 = %s\n", _ftoa( 0.000001 ) );
  printf( "0.0000011 = %s\n", _ftoa( 0.0000011 ) );
  printf( "0.000000000001 = %s\n", _ftoa( 0.000000000001 ) );
  zEndl();
  printf( "%s %s %s\n", itoa(1,buf), itoa(2,buf), itoa(3,buf) );
  printf( "%s ", _itoa(1) );
  printf( "%s ", _itoa(2) );
  printf( "%s\n",_itoa(3) );
  return 0;
}
