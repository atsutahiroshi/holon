#include <cure/cure_name.h>

struct test_t{
  Z_NAMED_CLASS
  int val;
} array[] = {
  { "a", 0 },
  { "b", 1 },
  { "c", 2 },
  { "d", 3 },
  { "e", 4 },
  { "f", 5 },
  { "g", 6 },
  { "h", 7 },
  { "i", 8 },
  { "j", 9 },
};

int num;

int main(void)
{
  struct test_t *p;
  char key[BUFSIZ];

  num = sizeof(array) / sizeof(struct test_t);
  memset( key, 0, BUFSIZ );
  printf( "%d components\n", num );
  for( key[0]='a'; key[0]<='l'; key[0]++ ){
    zNameFind( array, num, key, p );
    if( p )
      printf( "%s: %d\n", zName(p), p->val );
    else
      printf( "%s: unknown\n", key );
  }
  return 0;
}
