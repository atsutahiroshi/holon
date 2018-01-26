#include <cure/cure_misc.h>

#define TEST(s)  printf( "sizeof("#s") = %d\n", sizeof(s) );

int main(int argc, char *argv[])
{
  TEST(byte);
  TEST(word);
  TEST(long);
  TEST(ubyte);
  TEST(uword);
  TEST(uint);
  TEST(ulong);

  TEST(int8_t);
  TEST(int16_t);
  TEST(int32_t);
  TEST(int64_t);
  TEST(uint8_t);
  TEST(uint16_t);
  TEST(uint32_t);
  TEST(uint64_t);
  return 0;
}
