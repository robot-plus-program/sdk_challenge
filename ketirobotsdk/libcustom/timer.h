#ifndef THREADTEST_H
#define THREADTEST_H

#include "tcpclient2.h"

#include <chrono>
#include <iostream>
#include <time.h>
#include <functional>
#include <thread>

//#include "Signal.hpp"
#include "robotapi/robot.h"
#include "robotapi/rb10_v2.h"

#include "Signal.hpp"
using namespace std;

class timer
{

private:

    pthread_t thread;


public:
    Signal<> TimerCon;
    robot *THr;

    double a=0;
    bool timer_start;
    bool threadend;
    int interval;
    timer();
    ~timer();
    static void *run(void *arg);
//    void start(int interval,std::function<void(void)> func);

//    template<typename T>
//    void start(int _interval,void(T::*func)());
    void start(int _interval,robot *C);
    void start();

    void test();
    void stop();
    //    void start();
    void start(int _interval);
};

template <typename T,typename... Args>

void connect_member(T *inst, void (T::*func)(Args...) const) {

     [=](Args... args) {
        (inst->*func)(args...);
    };

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////



#endif // THREADTEST_H
