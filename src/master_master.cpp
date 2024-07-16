/*
 * ex11_master_master.cpp
 *
 *  Created on: Feb 22, 2010
 *      Author: Christopher Dellin
 *      Author: Dan Cody
 *      Author: Brian Zenowich
 */

#include <iostream>
#include <string>
#include <vector>

#include <boost/thread.hpp>

#include <barrett/detail/stl_utils.h>
#include <barrett/os.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems.h>
#include <barrett/units.h>

#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>

#include "master_master.h"

using namespace barrett;
using detail::waitForEnter;

bool validate_args(int argc, char **argv) {
    if (argc != 2 && argc != 3) {
        printf("Usage: %s <remoteHost> [--auto]\n", argv[0]);
        printf("  --auto : Automatically link WAMs and start Hand or Gimbals Hand Controller thread\n");

        return false;
    }

    return true;
}

// string config_path =
// "/home/robot/catkin_ws/src/barrett-ros-pkg_zeusV/wam_bringup/launch/with_hand_config_teleop_gains/default.conf";
template <size_t DOF> int wam_main(int argc, char **argv, ProductManager &pm, systems::Wam<DOF> &wam) {

    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    // libconfig::Config config = pm.getConfig();

    const jp_type HOME_POS = wam.getJointPositions();

    jp_type SYNC_POS;
    if (DOF == 7) {
        SYNC_POS[0] = 0.0;
        SYNC_POS[1] = -1.5;
        SYNC_POS[2] = -0.01;
        SYNC_POS[3] = 3.11;
        SYNC_POS[4] = 0.0;
        SYNC_POS[5] = 0.0;
        SYNC_POS[6] = 0.0;

    } else if (DOF == 4) {
        SYNC_POS[0] = 0.0;
        SYNC_POS[1] = -1.5;
        SYNC_POS[2] = -0.01;
        SYNC_POS[3] = 3.11;

    } else {
        printf("WARNING: Linking was unsuccessful.\n");
        return false;
    }

    if (DOF == 3) {
        printf("The program is not currently supported for the Proficios");
        return 0;
    }

    MasterMaster<DOF> mm(pm.getExecutionManager(), argv[1]);
    systems::connect(wam.jpOutput, mm.input);

    wam.gravityCompensate();

    std::vector<std::string> autoCmds;
    std::string line;
    v_type gainTmp;
    bool linked_first_time = false;
    bool exit_called = false;
    while (!exit_called) {
        if (autoCmds.empty()) {
            printf(">>> ");
            std::getline(std::cin, line);
        } else {
            line = autoCmds.back();
            autoCmds.pop_back();
        }

        if (!mm.isLinked() && linked_first_time)
            std::cout << "lost link with zeus" << std::endl;

        switch (line[0]) {
        case 'l':
            if (mm.isLinked()) {
                mm.unlink();
            } else {
                wam.moveTo(SYNC_POS);

                printf("Press [Enter] to link with the other WAM.");
                waitForEnter();
                mm.tryLink();
                wam.trackReferenceSignal(mm.output);

                btsleep(0.1); // wait an execution cycle or two
                if (mm.isLinked()) {
                    printf("Linked with remote WAM.\n");
                    linked_first_time = true;
                } else {
                    printf("WARNING: Linking was unsuccessful.\n");
                }
            }

            break;

        case 't':
            size_t jointIndex;
            {
                size_t jointNumber;
                std::cout << "\tJoint: ";
                std::cin >> jointNumber;
                jointIndex = jointNumber - 1;

                if (jointIndex >= DOF) {
                    std::cout << "\tBad joint number: " << jointNumber;
                    break;
                }
            }

            char gainId;
            std::cout << "\tGain identifier (p, i, or d): ";
            std::cin >> line;
            gainId = line[0];

            std::cout << "\tCurrent value: ";
            switch (gainId) {
            case 'p':
                gainTmp = wam.jpController.getKp();
                break;
            case 'i':
                gainTmp = wam.jpController.getKi();
                break;
            case 'd':
                gainTmp = wam.jpController.getKd();
                break;

            default:
                std::cout << "\tBad gain identifier.";
            }
            std::cout << gainTmp[jointIndex] << std::endl;

            std::cout << "\tNew value: ";
            std::cin >> gainTmp[jointIndex];
            switch (gainId) {
            case 'p':
                wam.jpController.setKp(gainTmp);
                break;
            case 'i':
                wam.jpController.setKi(gainTmp);
                break;
            case 'd':
                wam.jpController.setKd(gainTmp);
                break;

            default:
                std::cout << "\tBad gain identifier.";
            }

            break;

        case 'q':
            wam.moveTo(HOME_POS);
            exit_called = true;
            break;

        default:
            printf("\n");
            printf("    'l' to toggle linking with other WAM\n");
            printf("    't' to tune control gains\n");
            printf("    'q' to go home!\n");
            break;
        }
    }

    return 0;
}
