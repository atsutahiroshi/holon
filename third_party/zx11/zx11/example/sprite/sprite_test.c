#include <zx11/zxsprite.h>

#define WAITCOUNT 30

int main(int argc, char *argv[])
{
  zxWindow win;
  zxsPattern *pat;
  zxsLayer layer;
  zxSprite spr;
  uint i = 0, j = 0, id[] = { 0, 5, 2, 8 };
  int dx[] = { 1, -2, 2, -1 }, dy[] = { 1, 2, -1, -2 };

  zxInit();

  zxsLayerInit( &layer );
  zxsLayerCreatePixArray( &layer, 1 );
  zxWindowCreate( &win, 0, 0, 700, 520 );
  zxWindowSetBG( &win, "blue" );
  zxSpriteCreate( &win, &spr, 80, 10, 600, 500 );

  zxsLayerReadBGPixFile( &win, &layer, "../pixmaps/quinta-2.xpm" );
  zxsLayerBGSetRegion( &layer, 0, 0, layer.data.bg.width, layer.data.bg.height );
  zxsLayerReadPixFile( &win, &layer, 0, "../pixmaps/peng-movie.xpm" );
  zxsLayerPatternListAddHead( &layer, 0, 1, 12, 0, 0 );
  zxsLayerPatternListAddHead( &layer, 0, 1, 12, spr.reg.width-100, 0 );
  zxsLayerPatternListAddHead( &layer, 0, 1, 12, 0, spr.reg.height-100 );
  zxsLayerPatternListAddHead( &layer, 0, 1, 12, spr.reg.width-100, spr.reg.height-100 );

  zxSpriteAddLayer( &spr, &layer );
  zxSpriteDrawAll( &win, &spr );
  zxSpriteAppearAll( &win, &spr );

  zxWindowOpen( &win );
  while( 1 ){
    for( pat=zListTail(&layer.data.plist), j=0;
         pat!=zListRoot(&layer.data.plist);
         pat=zListCellNext(pat), j++ ){
      if( i % WAITCOUNT == 0 ) id[j]++;
      if( id[j] >= pat->data.col ) id[j] = 0;
      zxsPatternSet( pat, 0, id[j] );
      zxsPatternAddPos( pat, dx[j], dy[j] );
      if( pat->data.cur.x < 0 || pat->data.cur.x + pat->data.dx > spr.reg.width )
        dx[j] = -dx[j];
      if( pat->data.cur.y < 0 || pat->data.cur.y + pat->data.dy > spr.reg.height )
        dy[j] = -dy[j];
    }
    zxSpriteUpdate( &spr );
    zxSpriteDraw( &win, &spr );
    zxSpriteAppear( &win, &spr );
    zxFlush();
    i++;
  }
  return 0;
}
