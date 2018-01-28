#include <zx11/zxsprite.h>

#define WAITCOUNT 60

int main(int argc, char *argv[])
{
  zxWindow win;
  zxsLayer layer;
  zxSprite spr;
  int dx = 1, dy = 0;
  register int j = 0, id = 0;

  zxInit();

  zxWindowCreate( &win, 0, 0, 400, 300 );
  zxWindowSetBG( &win, "dark blue" );

  zxSpriteCreate( &win, &spr, 25, 25, 350, 250 );
  zxsLayerInit( &layer );
  zxsLayerCreatePixArray( &layer, 1 );
  zxsLayerReadBGPixFile( &win, &layer, "../pixmaps/quinta-2.xpm" );
  zxsLayerBGSetRegion( &layer, 0, 0, layer.data.bg.width, layer.data.bg.height );
  zxsLayerReadPixFile( &win, &layer, 0, "../pixmaps/peng-movie.xpm" );
  zxsLayerPatternListAddHead( &layer, 0, 1, 12, 140, 100 );

  zxSpriteAddLayer( &spr, &layer );
  zxSpriteDrawAll( &win, &spr );
  zxSpriteAppearAll( &win, &spr );

  zxWindowOpen( &win );
  while( 1 ){
    if( ++j == WAITCOUNT ){
      j = 0;
      if( ++id >= zListTail(&layer.data.plist)->data.col ) id = 0;
    }
    if( j % 2 == 0 ){
      zxsPatternSet( zListHead(&layer.data.plist), 0, id );
      layer.data.bg_reg.x += dx;
      layer.data.bg_reg.y += dy;
      if( layer.data.bg_reg.x > 350 ){ layer.data.bg_reg.x = 350; dx = 0; dy = 1; }
      if( layer.data.bg_reg.y > 250 ){ layer.data.bg_reg.y = 250; dx =-1; dy = 0; }
      if( layer.data.bg_reg.x <   0 ){ layer.data.bg_reg.x =   0; dx = 0; dy =-1; }
      if( layer.data.bg_reg.y <   0 ){ layer.data.bg_reg.y =   0; dx = 1; dy = 0; }
    }
    zxSpriteDrawAll( &win, &spr );
    zxSpriteAppearAll( &win, &spr );
    zxFlush();
  }
  return 0;
}
