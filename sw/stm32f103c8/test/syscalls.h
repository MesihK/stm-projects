#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/unistd.h>
#include "uart.h"

void _exit(int status) ;
int _close(int file) ;
int _execve(char *name, char **argv, char **env) ;
int _fork() ;
int _fstat(int file, struct stat *st) ;
int _getpid() ;
int _isatty(int file) ;
int _kill(int pid, int sig) ;
int _link(char *old, char *new) ;
int _lseek(int file, int ptr, int dir) ;
caddr_t _sbrk(int incr) ;
int _read(int file, char *ptr, int len) ;
int _stat(const char *filepath, struct stat *st) ;
clock_t _times(struct tms *buf) ;
int _unlink(char *name) ;
int _wait(int *status) ;
