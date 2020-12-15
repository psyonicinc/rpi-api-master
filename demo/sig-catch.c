#include "sig-catch.h"
#include <signal.h>
#include <string.h>
#include <unistd.h>

#define SIGTERM_MSG "Signal-based termination received.\r\n"

volatile uint8_t gl_run_ok = 1;

void sig_term_handler(int signum, siginfo_t *info, void *ptr)
{
	gl_run_ok = 0;
    write(STDERR_FILENO, SIGTERM_MSG, sizeof(SIGTERM_MSG));
}

void signal_catch_setup(void)
{
    static struct sigaction _sigact;

    memset(&_sigact, 0, sizeof(_sigact));
    _sigact.sa_sigaction = sig_term_handler;
    _sigact.sa_flags = SA_SIGINFO;

    sigaction(SIGTERM, &_sigact, NULL);
	sigaction(SIGINT, &_sigact, NULL);

}