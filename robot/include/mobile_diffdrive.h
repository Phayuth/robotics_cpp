#ifndef MOBILE_DIFFDRIVE_H
#define MOBILE_DIFFDRIVE_H

class DifferentialDriveRobot {
    private:
        double blength;
        double wradius;

    public:
        double x;
        double y;
        double yaw;

        DifferentialDriveRobot(double blength, double wradius, double x, double y,
                               double yaw);
        ~DifferentialDriveRobot();

        void set_pose(double x, double y, double yaw);

        void fkin_ex(double v, double w, double Ts);
        void fkin_in(double wr, double wl, double Ts);

        void prt_pose(void);
};

#endif