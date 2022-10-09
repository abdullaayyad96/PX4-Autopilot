#ifndef TIMER_HPP
#define TIMER_HPP

#include <time.h>


namespace HEAR{

class Timer {
    private:
    timespec _start, _end;
    // int _start;
    public:
    void tick(){
        clock_gettime(CLOCK_REALTIME, &_start);
    }
    int tockMicroSeconds(){
        clock_gettime(CLOCK_REALTIME, &_end);
        return (_end.tv_sec - _start.tv_sec + (_end.tv_nsec - _start.tv_nsec) / 1000000000.) * 1e6;
    }
    int tockMilliSeconds(){
        clock_gettime(CLOCK_REALTIME, &_end);
        return (_end.tv_sec - _start.tv_sec + (_end.tv_nsec - _start.tv_nsec) / 1000000000.) * 1e3;
    }
};

}

#endif
