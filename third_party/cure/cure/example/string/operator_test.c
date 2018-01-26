#include <cure/cure_string.h>

int main(void)
{
  void operatortest(char c){
    printf( "<operator?>case %c:", c );
    zIsOperator( c ) ? printf( "yes\n" ) : printf( "no\n" );
  }
  void delimitertest(char c){
    printf( "<delimiter?>case %c:", c );
    zIsDelimiter( c ) ? printf( "yes\n" ) : printf( "no\n" );
  }

  operatortest( '=' );
  operatortest( '+' );
  operatortest( ' ' );
  operatortest( ':' );
  operatortest( '.' );
  operatortest( '\n' );

  delimitertest( '=' );
  delimitertest( '+' );
  delimitertest( ' ' );
  delimitertest( ':' );
  delimitertest( '.' );
  delimitertest( '\n' );

  return 0;
}
