
#include <Eigen/Dense>

class EKF {
public:
    EKF(float dt, const Eigen::Vector3f& av_sbsnsk_x, const Eigen::Vector3f& av_sbsnsk_y, const Eigen::Vector3f& av_sbsnsk_z,
         const Eigen::Vector3f& asd_nk_x, const Eigen::Vector3f& asd_nk_y, const Eigen::Vector3f& asd_nk_z);

    void predict(const Eigen::Vector3f& acc_meas, const Eigen::Quaternionf& q_meas, float current_dt);

    void update(const Eigen::Vector3f& acc_meas, const Eigen::Quaternionf& q_meas);

    Eigen::Matrix<float, 15, 1> getStateEstimate();

private:
    float dt_;
    Eigen::Matrix<float, 15, 15> Q_;
    Eigen::Matrix<float, 7, 7> R_;
    Eigen::Matrix<float, 15, 15> F_;
    Eigen::Matrix<float, 15, 7> H_;
    Eigen::Matrix<float, 15, 1> x_;
    Eigen::Matrix<float, 15, 15> P_;
};
