/* LIW - Linux Wizard
 * Copyright (C) 2000 Tomomichi Sugihara (Zhidao)
 *
 * liw_socket - socket communication
 */

#include <cure/cure.h>
#include <liw/liw_socket.h>

/* sockets on UNIX filesystem */

int liwSocketUNOpen(char *name, int type, liwSockAddrUN *addr)
{
  int fd;

  addr->sun_family = AF_UNIX;
  strcpy( addr->sun_path, name );
  if( ( fd = socket( AF_UNIX, type, LIW_SOCKET_DEFAULT_PROTOCOL ) ) == -1 )
    ZSYSERROR();
  return fd;
}

int liwSocketUNBind(char *name, int type, int num)
{
  int fd;
  liwSockAddrUN addr;

  fd = liwSocketUNOpen( name, type, &addr );
  if( bind( fd, (liwSockAddr *)&addr, sizeof(addr) ) ||
      listen( fd, num ) ){
    ZSYSERROR();
    return 0;
  }
  return fd;
}

int liwSocketUNConnect(char *name, int type)
{
  int fd;
  liwSockAddrUN addr;

  fd = liwSocketUNOpen( name, type, &addr );
  if( connect( fd, (liwSockAddr *)&addr, sizeof(addr) ) ){
    ZSYSERROR();
    return 0;
  }
  return fd;
}

int liwSocketUNAccept(int fd)
{
  int ret;
  socklen_t len;
  liwSockAddrUN addr;

  len = sizeof(addr);
  if( ( ret = accept( fd, (liwSockAddr *)&addr, &len ) ) == -1 )
    ZSYSERROR();
  return ret;
}

/* socket on internet system */

int liwSocketINETOpen(char *ip, int port, int type, liwSockAddrINET *addr)
{
  int fd;

  addr->sin_family = AF_INET;
  inet_aton( ip, &addr->sin_addr );
  addr->sin_port = htons( port );
  if( ( fd = socket( AF_INET, type, LIW_SOCKET_DEFAULT_PROTOCOL ) ) == -1 )
    ZSYSERROR();
  return fd;
}

int liwSocketINETBind(char *ip, int port, int type, int num)
{
  int fd;
  liwSockAddrINET addr;

  fd = liwSocketINETOpen( ip, port, type, &addr );
  if( bind( fd, (liwSockAddr *)&addr, sizeof(addr) ) ||
      listen( fd, num ) ){
    ZSYSERROR();
    return 0;
  }
  return fd;
}

int liwSocketINETConnect(char *ip, int port, int type)
{
  int fd;
  liwSockAddrINET addr;

  fd = liwSocketINETOpen( ip, port, type, &addr );
  if( connect( fd, (liwSockAddr *)&addr, sizeof(addr) ) ){
    ZSYSERROR();
    return 0;
  }
  return fd;
}

int liwSocketINETAccept(int fd)
{
  int ret;
  socklen_t len;
  liwSockAddrINET addr;

  len = sizeof(addr);
  if( ( ret = accept( fd, (liwSockAddr *)&addr, &len ) ) == -1 )
    ZSYSERROR();
  return ret;
}
