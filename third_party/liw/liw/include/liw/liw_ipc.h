/* LIW - Linux Wizard
 * Copyright (C) 2000 Tomomichi Sugihara (Zhidao)
 */
/*! \file liw_ipc.h
 * \brief System V IPC (semaphore, shared memory, message queue)
 * \author Zhidao
 */

#ifndef __LIW_SYSVIPC_H__
#define __LIW_SYSVIPC_H__

#include <cure/cure.h>

#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/shm.h>
#include <sys/msg.h>

#define liwIPCKey(s) (key_t)ftok( (s), 0 )

__BEGIN_DECLS

/* ********************************************************** */
/* CLASS: liwSEM
 * semaphore operation
 * ********************************************************** */

#if _SEM_SEMUN_UNDEFINED == 1
union semun{
  int val;
  struct semid_ds *buf;
  unsigned short *array;
  struct seminfo *__buf;
};
#endif

bool liwSEMCreate(int id);
bool liwSEMDestroy(int id);
int liwSEMGet(char *key);

bool liwSEMOperate(int id, short dv);

#define LIW_SEM_PASSEREN  (-1)
#define LIW_SEM_VRIJGEVEN ( 1)

#define liwSEMPasseren(i)  liwSEMOperate( i, LIW_SEM_PASSEREN )
#define liwSEMVrijgeven(i) liwSEMOperate( i, LIW_SEM_VRIJGEVEN )

/* ********************************************************** */
/* CLASS: liwSHM
 * shared memory management
 * ********************************************************** */

typedef struct{
  int id;
  unsigned size;
  void *mp;
} liwSHM;

int liwSHMModeAttach(liwSHM *shm, char *key, int size, int mode);

#define liwSHMWriteAttach(m,k,s)  liwSHMModeAttach((m),(k),(s),0222)
#define liwSHMReadAttach(m,k,s)   liwSHMModeAttach((m),(k),(s),0444)
#define liwSHMAttach(m,k,s)       liwSHMModeAttach((m),(k),(s),0666)

#define liwSHMModeCreate(m,k,s,p) liwSHMModeAttach(m,k,s,IPC_CREAT|p)
#define liwSHMCreate(m,k,s)       liwSHMModeCreate(m,k,s,0666)

int liwSHMModeAlloc(liwSHM *shm, char *key, int size, int mode);

#define liwSHMWriteAlloc(m,k,s)   liwSHMModeAlloc((m),(k),(s),0222)
#define liwSHMReadAlloc(m,k,s)    liwSHMModeAlloc((m),(k),(s),0444)
#define liwSHMAlloc(m,k,s)        liwSHMModeAlloc((m),(k),(s),0666)

void liwSHMClear(liwSHM *shm);

bool liwSHMDetach(liwSHM *shm);
bool liwSHMDestroy(liwSHM *shm);

/* ********************************************************** */
/* CLASS: liwMSQ
 * message queue operation
 * ********************************************************** */

/* all structs transfered through msq must have
 * long integer type member at the head.
 */
#define LIW_MSQ_PACKET long int msg_type

int liwMSQGet(char *key);

int liwMSQSend(int id, void *data, int size);
int liwMSQReceive(int id, void *data, int size, int type);

bool liwMSQDestroy(int id);

__END_DECLS

#endif /* __LIW_SYSVIPC_H__ */
