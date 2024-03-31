
#include <Eigen/Dense>

class EKF {
public:
    EKF(int n, int m, int p);
    ~EKF();

    void initialize(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);
    void predict(const Eigen::Vector3d& acc_meas, const Eigen::Quaterniond& q_meas, double dt);
    void update(const Eigen::VectorXd& z);

    Eigen::VectorXd getState() const;
    Eigen::MatrixXd getCovariance() const;


    // Allan variance parameters
    double acc_white_noise_;
    double acc_bias_instability_;
    double acc_rate_random_walk_;
    double gyro_white_noise_;
    double gyro_bias_instability_;
    double gyro_rate_random_walk_;
    double bias_instability_time_constant_;

private:
    int n_;  // State dimension
    int m_;  // Measurement dimension
    int p_;  // Input dimension

    Eigen::VectorXd x_;  // State vector
    Eigen::MatrixXd P_;  // State covariance matrix
    Eigen::MatrixXd Q_;  // Process noise covariance matrix
    Eigen::MatrixXd R_;  // Measurement noise covariance matrix

    Eigen::MatrixXd F_;  // State transition matrix
    Eigen::MatrixXd H_;  // Measurement matrix

    // Error model parameters
    Eigen::VectorXd bias_;           // Bias vector
    Eigen::VectorXd bias_instability_;  // Bias instability vector
    Eigen::VectorXd noise_;          // White noise vector
    Eigen::VectorXd scale_factor_;   // Scale factor vector
    Eigen::MatrixXd misalignment_;   // Misalignment matrix

    void updateErrorModel(double dt);
    void updateProcessNoiseCovariance(double dt);
    void updateMeasurementNoiseCovariance();
};
