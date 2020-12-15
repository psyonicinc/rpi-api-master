#ifndef SIG_CATCH_H
#define SIG_CATCH_H
#include <stdint.h>

extern volatile uint8_t gl_run_ok;
void signal_catch_setup(void);

#endif