// Copyright (c) 2018, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <ugcs/vsm/vsm.h>
#include <ugcs/vsm/callback.h>
#include <ugcs/vsm/run_as_service.h>
#include <px4_vehicle_manager.h>
#include <signal.h>
#include <iostream>

DEFINE_DEFAULT_VSM_NAME;

bool terminate;

#ifdef __unix__
void Sigint_handler(int signum __UNUSED)
{
    LOG_INFO("Signal caught, exiting...");
    terminate = true;
}
#endif /* __unix__ */


Px4_vehicle_manager::Ptr manager;

int
start_main(int argc, char *argv[])
{
    ugcs::vsm::Initialize(argc, argv, "vsm-px4.conf");
    manager = Px4_vehicle_manager::Create();
    manager->Enable();
    return 0;
}

void
stop_main()
{
    manager->Disable();
    manager = nullptr;
    ugcs::vsm::Terminate();
}

void
wait_for_termination()
{
    while (!terminate) {
        /* Think about better way. */
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int
main(int argc, char *argv[])
{
    auto ret = ugcs::vsm::Run_as_service(
            "ugcs-vsm-px4",
            argc,
            argv,
            ugcs::vsm::Make_program_init_handler(start_main),
            ugcs::vsm::Make_callback(stop_main));
    if (ret != ugcs::vsm::SERVICE_RESULT_NORMAL_INVOCATION) {
        return ret;
    }

#ifdef __unix__
    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = Sigint_handler;
    sigaction(SIGINT, &action, NULL);
#endif /* __unix__ */

    start_main(argc, argv);
    wait_for_termination();
    stop_main();
    return 0;
}
