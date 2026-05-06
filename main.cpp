#include "state_machine.hpp"
#ifdef BUILD_SIMULATION
    #define BACKWARD_HAS_DW 1
    #include "backward.hpp"
    namespace backward{
        backward::SignalHandling sh;
    }
#endif
using namespace types;

MotionStateFeedback StateBase::msfb_ = MotionStateFeedback();

#include <csignal>
#include <atomic>
#include <iostream>

std::atomic<bool> g_running(true);
void signal_handler(int sig) {
    g_running = false;
    std::cout << "\n[Signal] Caught SIGINT, shutting down..." << std::endl;
}

int main(){
    std::signal(SIGINT, signal_handler);
    StateMachine state_machine(RobotType::Lite3);
    state_machine.Run();
    return 0;
}