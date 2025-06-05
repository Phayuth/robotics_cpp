#include "mobile_diffdrive.h"

int main(int argc, char const *argv[]) {
    DifferentialDriveRobot robot(.5, .1, .0, .0, .0);

    double t(0.0);
    while (t < 5.0) {
        robot.fkin_ex(.2, .0, .01);
        robot.prt_pose();
        t += .01;
    }

    return 0;
}
