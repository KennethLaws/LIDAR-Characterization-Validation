#if !defined __BAH_TIME_UTIL_H__
#define __BAH_TIME_UTIL_H__

#include <time.h>

inline void msdelay(unsigned milliseconds)
{
    timespec wait_time;
    wait_time.tv_sec = milliseconds / 1000;
    wait_time.tv_nsec = (milliseconds % 1000) * 1000000;

    while (nanosleep(&wait_time, &wait_time) > 0) ;
}

inline void usdelay(unsigned microseconds)
{
    timespec wait_time;
    wait_time.tv_sec = microseconds / 1000000;
    wait_time.tv_nsec = (microseconds % 1000000) * 1000;

    while (nanosleep(&wait_time, &wait_time) > 0) ;
}

inline int timeval_subtract(timespec * result, const timespec * left, const timespec * right)
{
    timespec x = *left, y = *right;

    // Perform the carry for the later subtraction by updating y.
    if (x.tv_nsec < y.tv_nsec) {
        long int nsec = (y.tv_nsec - x.tv_nsec) / 1000000000 + 1;
        y.tv_nsec -= 1000000000 * nsec;
        y.tv_sec += nsec;
    }

    if (x.tv_nsec - y.tv_nsec > 1000000000) {
        long int nsec = (x.tv_nsec - y.tv_nsec) / 1000000000;
        y.tv_nsec += 1000000000 * nsec;
        y.tv_sec -= nsec;
    }

    // Compute the time remaining to wait.
    // tv_nsec is certainly positive.
    result->tv_sec = x.tv_sec - y.tv_sec;
    result->tv_nsec = x.tv_nsec - y.tv_nsec;
    /* Return 1 if result is negative. */
    return x.tv_sec < y.tv_sec;
}

inline void timeval_add(timespec * result, const timespec * left, const timespec * right)
{
    result->tv_sec = left->tv_sec + right->tv_sec;
    result->tv_nsec = left->tv_nsec + right->tv_nsec;

    while (result->tv_nsec >= 1000000000) {
        result->tv_nsec -= 1000000000;
        result->tv_sec++;
    }
}

inline void delay_until( const timespec * trigger_time )
{
    timespec current_time;
    timespec time_diff;
    timespec sleep_time;
    bool     too_late = false;
    timespec nanosleep_tol = {0, 2000000}; // trust nanosleep down to 2ms

    // nanosleep until you are close to avoid loading the CPU,
    // and then keep checking the time until you are just past
    do {
        clock_gettime(CLOCK_REALTIME, &current_time);
        too_late = timeval_subtract(&time_diff, trigger_time, &current_time);

        if (!timeval_subtract(&sleep_time, &time_diff, &nanosleep_tol)) {
            nanosleep(&sleep_time, NULL);
        }
    } while (!too_late);
}

#endif
