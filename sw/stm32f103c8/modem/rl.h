#include <string.h>
#include "microrl.h"

int execute (int argc, const char * const * argv);
char ** complet (int argc, const char * const * argv);
void rl_sigint (void);
void rl_reset_cursor(void);
void rl_help();
void rl_print(const char *str);

