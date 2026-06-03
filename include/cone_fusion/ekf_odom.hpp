#pragma once

#include <Eigen/Dense>

#include "color_logic.hpp"

#include <iostream>

#define N_CONES 400           /* This constant is the initial cones number. */
#define INF 1e1       /* 1e10 Constant value to represent Infinite */

using namespace Eigen;

/**
 *  type defining the signature vector. A signature is way of indexing a particular landmark. In our use case
 * the landmark (cones) are identified by the INDEX in the state vector. This further vector is used to identify
 * the COLOR of the cone which is extremely important. Each element in this vector refers to an already mapped cone in the 
 * state vector with the same index
*/
typedef Eigen::Matrix<ColorLogic, N_CONES, 1> SignatureVector;

class EKFOdom
{
private:
    Eigen::VectorXf x_;     /* Robot state vector */
    Eigen::MatrixXf P_;     /* Covariance matrix */
    Eigen::Matrix2f Q_;     /* Process noise covariance */
    Eigen::Matrix3f R_;     /* Measurement covariance */
    Eigen::MatrixXf Fx_;    /* Convenience matrix used for mapping the 3D state vector to (3, 3N + 3)D during update step */
    Eigen::MatrixXf Fx_k;   /* Convenience matrix used for mapping the 3D state vector to (6,  3N + 3)D during correction step */
    SignatureVector s_;     /* Vector containing the signature for each landmark. In our case the signature is the COLOR of the cone. The ID for each color are defined in hedaer file: "color_logic.hpp" . */

    float act_vel = 0.0;       /* Actual vehicle velocity [m/s] */
    float act_ang_vel = 0.0;   /* Actual vehicle angular_velocity [rad/s] */

    size_t landmark_count = 0;  /* Counter of mapped landmarks */

    float max_new_cone_dist;   /* Max distance for creating a new cone */

    bool is_pose_initialized = false;  /* Bool to check if an initial pose has been set */

    Eigen::Vector3f prev_limo_pose_ = Eigen::Vector3f::Zero();  /* Last FAST-LIMO pose, used to compute the relative motion increment in setPose */

    Eigen::Vector3f prev_limo_cov_ = Eigen::Vector3f::Zero();   /* Last FAST-LIMO pose covariance, used to compute the covariance increment in setPoseCovariance */
    bool is_cov_initialized = false;  /* Bool to check if the first FAST-LIMO covariance has been recorded */

    bool is_first_lap_completed = false; /* Bool to understand if the first lap is completed. In that case, do not map any new cone */

    bool batch_update_ = false; /* If true, correct() fuses ALL cones associated in a scan in one joint EKF update instead of only the last one */

    size_t anchor_ramp_scans_ = 0; /* # of correction scans over which the cone anchor ramps in from lap 2 (0 = instant snap) */
    size_t anchor_scans_ = 0;      /* counter of correction scans elapsed since the anchor became active */

    float assoc_maha_gate_ = 9.21f; /* chi-square (2 DOF) gate for lap-2+ data association by Mahalanobis distance */

    float q_motion_pos_ = 0.0f; /* Additive process noise on x,y per metre travelled [m^2/m]. Keeps P from collapsing so the assoc gate stays honest. 0 = off. */
    float q_motion_yaw_ = 0.0f; /* Additive process noise on theta per radian turned [rad^2/rad]. 0 = off. */

public:
    EKFOdom(Vector2f process_noise, Vector3f measurement_noise, const float alpha);
    virtual ~EKFOdom();
    

    void predict(const float dt);

    /**
     * Correct filter state using measurement(s) z:
     * 
     * @param z: Measurement(s) of actual cones. This vector contains:
     *  - actual range from vehicle to the landmark;
     *  - actual bearing from vehicle to the landmark;
     *  - signature of the landmark;
     *  @param act_cones_detected: Number of actually observed landmarks
     *  @return number of observations that passed data association (and thus
     *          contributed to a correction) this scan — diagnostic.
    */
    size_t correct(const Vector3f *z, const size_t act_cones_detected);
    
    VectorXf getState() const;
    MatrixXf getCovariance() const;
    Vector3f getPoseCovariance() const;
    Matrix2f getProcessNoiseCovariance() const;
    Matrix3f getMeasurementNoiseCovariance() const;
    MatrixXf getFx() const;
    size_t getActMappedLandmarks() const;
    SignatureVector getSignatures() const;

    void setFirstLapCompleted(const bool first_lap_completed);
    void setBatchUpdate(const bool enable);
    void setAnchorRampScans(const size_t scans);
    void setAssocMahaGate(const float gate);
    void setMotionNoise(const float pos, const float yaw);
    void setActVel(const float vel);
    void setActAngVel(const float ang_vel);
    void setPose(const Vector3f pose);
    void setPoseCovariance(const Vector3f pos_cov);


    inline float euclideanDistance(float x1, float x2, float y1, float y2) {
      return sqrt(((x2 - x1)*(x2 - x1)) + ((y2 - y1)*(y2 - y1)));
    }

    inline float normalizeAngle(float angle) {
      while(angle > M_PI)
        angle -= 2*M_PI;
      while(angle < -M_PI)
        angle += 2*M_PI;

      return angle;
    }

    inline float normalizeYaw(float yaw) {
      while(yaw > (2*M_PI))
      {
        yaw -= (2*M_PI);
      }
      while(yaw < -(2*M_PI))
      {
        yaw += (2*M_PI);
      }

      return yaw;
    }



};