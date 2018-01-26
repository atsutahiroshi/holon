#include <cure/cure_string.h>

void add_char_test(char buf[], size_t size)
{
  printf( "add_char test.\n" );
  zStrAddChar( buf, size, 'b' );
  printf( "%s\n", buf );
  zStrAddChar( buf, size, 'a' );
  printf( "%s\n", buf );
  zStrAddChar( buf, size, 'a' );
  printf( "%s\n", buf );
}

void insert_char_test(char buf[], size_t size)
{
  printf( "insert_char test.\n" );
  zStrInsChar( buf, size, 1, 'h' );
  printf( "%s\n", buf );
  zStrInsChar( buf, size, 4, 'g' );
  printf( "%s\n", buf );
  zStrInsChar( buf, size, 100, 'y' );
  printf( "%s\n", buf );
}

void override_char_test(char buf[], size_t size)
{
  printf( "override_char test.\n" );
  zStrOvrChar( buf, size, 1, 'u' );
  printf( "%s\n", buf );
  zStrOvrChar( buf, size, 2, 's' );
  printf( "%s\n", buf );
  zStrOvrChar( buf, size, 3, 'i' );
  printf( "%s\n", buf );
  zStrOvrChar( buf, size, 100, 'u' );
  printf( "%s\n", buf );
  zStrOvrChar( buf, size, 100, 's' );
  printf( "%s\n", buf );
  zStrOvrChar( buf, size, 100, 'i' );
  printf( "%s\n", buf );
}

void delete_char_test(char buf[], size_t size)
{
  printf( ">> delete_char test.\n" );
  zStrDelChar( buf, size, 3 );
  printf( "%s\n", buf );
  zStrDelChar( buf, size, 3 );
  printf( "%s\n", buf );
  zStrDelChar( buf, size, 3 );
  printf( "%s\n", buf );
  zStrDelChar( buf, size, 7 );
  printf( "%s\n", buf );
  zStrDelChar( buf, size, 7 );
  printf( "%s\n", buf );
  zStrDelChar( buf, size, 7 );
  printf( "%s\n", buf );
}

void backspace_char_test(char buf[], size_t size)
{
  printf( "backspace_char test.\n" );
  zStrBSChar( buf, size, 3 );
  printf( "%s\n", buf );
  zStrBSChar( buf, size, 2 );
  printf( "%s\n", buf );
  zStrBSChar( buf, size, 1 );
  printf( "%s\n", buf );
  zStrBSChar( buf, size, 0 );
  printf( "%s\n", buf );
  zStrBSChar( buf, size, 100 );
  printf( "%s\n", buf );
  zStrBSChar( buf, size, 100 );
  printf( "%s\n", buf );
  zStrBSChar( buf, size, 100 );
  printf( "%s\n", buf );
  zStrBSChar( buf, size, 100 );
  printf( "%s\n", buf );
}

#define SIZE BUFSIZ

int main(void)
{
  char buf[SIZE];

  strcpy( buf, "foo" );
  printf( "%s\n", buf );
  add_char_test( buf, SIZE );
  insert_char_test( buf, SIZE );
  override_char_test( buf, SIZE );
  delete_char_test( buf, SIZE );
  backspace_char_test( buf, SIZE );
  return 0;
}
