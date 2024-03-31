
#include <Eigen/Dense>
#include <cmath>



class PID
{
    public:
        PID(double Kp, double Ki, double Kd, double Qi);
        double update(double error, double dt);
        void reset();
    private:
        double Kp, Ki, Kd, Qi;
        double integral, prev_error;
};

