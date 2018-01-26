#include <cure/cure_string.h>

char *bm_search(char *text, char *pat)
{
  register int i, j, n, m;
  ubyte skip[0x80];

  m = strlen( pat ) - 1;
  memset( skip, m+1, 0x80 );
  for( i=0; i<m; i++ )
    skip[ (int)pat[i] ] = m - i;
  n = strlen(text) - m;
  for( i=0; i<n; i+= skip[(int)text[i+m]] )
    if( text[i+m] == pat[m] ){
      for( j=m-1; j>=0; j-- )
        if( text[i+j] != pat[j] ) break;
      if( j < 0 ) return text + i;
    }
  return NULL;
}

int main(void)
{
  char *str = "abcdefgabcdefe";
  char pat[BUFSIZ], *result;

  while( 1 ){
    printf( "%s\n", str );
    printf( "enter: " );
    fgets( pat, BUFSIZ, stdin );
    pat[strlen(pat)-1] = '\0';

    if( !strcmp( pat, "quit" ) ) break;
    result = bm_search( str, pat );
    printf( "%s\n", result ? result : "(not found)" );
  }
  return 0;
}
