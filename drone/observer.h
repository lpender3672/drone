
#include <Eigen/Dense>

class EKF {
public:
    EKF(float dt, const Eigen::Vector3f& av_sbsnsk_x, const Eigen::Vector3f& av_sbsnsk_y, const Eigen::Vector3f& av_sbsnsk_z,
         const Eigen::Vector3f& asd_nk_x, const Eigen::Vector3f& asd_nk_y, const Eigen::Vector3f& asd_nk_z);

    void EKF::predict();

    void update(const Eigen::Vector2d& z);

    Eigen::Vector3f getStateEstimate();

private:
    double dt_;
    Eigen::Vector2d x_;
    Eigen::Matrix<double, 3, 3> Q_;
    Eigen::Matrix<double, 7, 7> R_;
    Eigen::Matrix<double, 15, 15> F_;
    Eigen::Matrix<double, 2, 2> H_;
    Eigen::Matrix<double, 15, 1> x_;
    Eigen::Matrix<double, 15, 15> P_;
};
