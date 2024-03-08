
#include <Eigen/Dense>
#include <cmath>



class PID
{
    public:
        PID(float Kp, float Ki, float Kd, float Qi);
        float update(float error, float dt);
        void reset();
    private:
        float Kp, Ki, Kd, Qi;
        float integral, prev_error;
};

