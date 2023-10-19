#include "state_all.h"

void externalSetup(StateStruct states[])
{

    states[ST_STARTUP].enter = startup_enter;
    states[ST_STARTUP].beforeLoop = startup_beforeLoop;
    states[ST_STARTUP].loop = startup_loop;
    states[ST_STARTUP].afterLoop = startup_afterLoop;

    states[ST_IDLE].enter = idle_enter;
    states[ST_IDLE].beforeLoop = idle_beforeLoop;
    states[ST_IDLE].loop = idle_loop;
    states[ST_IDLE].afterLoop = idle_afterLoop;

} // Set all the callbacks
