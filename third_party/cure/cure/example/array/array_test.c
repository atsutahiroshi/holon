#include <cure/cure_array.h>

#define NUM 10

typedef struct{
  double x, y, z;
} coord;

void coord_create(coord *c, double x, double y, double z)
{
  c->x = x;
  c->y = y;
  c->z = z;
}

void coord_write(coord *c)
{
  printf( "( %f, %f, %f )\n", c->x, c->y, c->z );
}

zArrayClass( coord_array_t, coord );

int main(void)
{
  register int i;
  coord_array_t array;

  zArrayAlloc( &array, coord, NUM );
  for( i=0; i<NUM; i++ )
    coord_create( zArrayElem(&array,i), i, i, i );
  for( i=0; i<NUM; i++ )
    coord_write( zArrayElem( &array, i ) );
  zArrayFree( &array );
  return 0;
}
