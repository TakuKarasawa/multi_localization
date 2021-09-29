//
// reference: ProbabilisticRobotics
//

#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <iostream>
#include <Eigen/Dense>

class KalmanFilter
{
public:
	KalmanFilter() { I.setIdentity(); }

	void set_pose(double x,double y,double yaw)
	{
		MU(0) = x;
		MU(1) = y;
		MU(2) = yaw;
	}

	void set_cov()
	{
		SIGMA.setZero();
		for(int i = 0; i < 3; i++) SIGMA(i,i) = 1e-10;
	}

	void set_motion_noise(double nn,double no,double on,double oo)
	{
		MOTION_NOISE_NN = nn;
		MOTION_NOISE_NO = no;
		MOTION_NOISE_ON = on;
		MOTION_NOISE_OO = oo;
	}

	void set_measurement_noise(double dist_noise_rate,double dir_noise)
	{
		DISTANCE_NOISE_RATE = dist_noise_rate;
		DIRECTION_NOISE = dir_noise;
	}

	void motion_update(double nu,double omega,double dt)
	{
		if(std::abs(omega) < 1e-5) omega = 1e-5;
		Eigen::Matrix2d M = M_(nu,omega,dt);
		Eigen::Matrix<double,3,2> A = A_(nu,omega,dt,MU(2));
		Eigen::Matrix3d G = G_(nu,omega,dt,MU(2));
		
		state_transition(nu,omega,dt,MU);
		SIGMA = G*SIGMA*G.transpose() + A*M*A.transpose();
	}

	void measurement_update(double measured_x,double measured_y,double distance,double direction)
	{
		Eigen::Matrix2d Q = Q_(distance);
		Eigen::Matrix<double,2,3> H = H_(measured_x,measured_y,distance);
		Eigen::Matrix<double,3,2> K = SIGMA*H.transpose()*(Q + H*SIGMA*H.transpose()).inverse();

		Eigen::Vector2d Z;
		Z.setZero();
		Z(0) = distance;
		Z(1) = direction;

		MU += K*(Z - measurement_function(MU,measured_x,measured_y));
		SIGMA = (I - K*H)*SIGMA;
	}

private:
	void state_transition(double nu,double omega,double dt,Eigen::Vector3d& pose)
	{
		double yaw = pose(2);
		if(omega < 1e-10){
			pose(0) += nu*std::cos(yaw)*dt;
			pose(1) += nu*std::sin(yaw)*dt;
			pose(2) += omega*dt;
		}
		else{
			pose(0) += nu/omega*(std::sin(yaw + omega*dt) - std::sin(yaw));
			pose(1) += nu/omega*(-std::cos(yaw + omega*dt) + std::cos(yaw));
			pose(2) += omega*dt;
		}
	}

	Eigen::Vector2d measurement_function(Eigen::Vector3d pose,double measured_x,double measured_y)
	{
		// initialize
		Eigen::Vector2d z;
		z.setZero();
		double diff, phi;

		diff = std::sqrt(std::pow(measured_x - pose(0),2) + std::pow(measured_y - pose(1),1));
		phi = std::atan2(measured_y - pose(1),measured_x - pose(0));
		while(phi >= M_PI) phi -= 2*M_PI;
		while(phi < -M_PI) phi += 2*M_PI;

		z(0) = diff;
		z(1) = phi;

		return z;
	}

	Eigen::Matrix2d M_(double nu,double omega,double dt)
	{
		// initialize
		Eigen::Matrix2d M;
		M.setZero();

		M(0,0) = std::pow(MOTION_NOISE_NN,2)*std::abs(nu)/dt + std::pow(MOTION_NOISE_NO,2)*std::abs(omega)/dt;
		M(1,1) = std::pow(MOTION_NOISE_ON,2)*std::abs(nu)/dt + std::pow(MOTION_NOISE_OO,2)*std::abs(omega)/dt;

		return M;
	}

	Eigen::Matrix<double,3,2> A_(double nu,double omega,double dt,double theta)
	{
		// initialize
		Eigen::Matrix<double,3,2> A;
		A.setZero();

		A(0,0) = (std::sin(theta + omega*dt) - std::sin(theta))/omega;
		A(0,1) = -nu/std::pow(omega,2)*(std::sin(theta + omega*dt) - std::sin(theta)) + nu/omega*dt*std::cos(theta + omega*dt);
		A(1,0) = (-std::cos(theta + omega*dt) + std::cos(theta))/omega;
		A(1,1) = -nu/std::pow(omega,2)*(-std::cos(theta + omega*dt) + std::cos(theta)) + nu/omega*dt*std::sin(theta + omega*dt);
		A(2,0) = 0.0;
		A(2,1) = dt;
		
		return A;
	}

	Eigen::Matrix3d G_(double nu,double omega,double dt,double theta)
	{
		// initialize
		Eigen::Matrix3d G;
		G.setIdentity();

		G(0,2) = nu/omega*(std::cos(theta + omega*dt) - std::cos(theta));
		G(1,2) = nu/omega*(std::sin(theta + omega*dt) - std::sin(theta));

		return G;
	}

	Eigen::Matrix2d Q_(double distance)
	{
		// initialize
		Eigen::Matrix2d Q;
		Q.setZero();

		Q(0,0) = std::pow(distance*DISTANCE_NOISE_RATE,2);
		Q(1,1) = std::pow(DIRECTION_NOISE,2);

		return Q;
	}

	Eigen::Matrix<double,2,3> H_(double measured_x,double measured_y,double distance)
	{
		// initialize
		Eigen::Matrix<double,2,3> H;
		H.setZero();

		H(0,0) = (MU(0) - measured_x)/std::sqrt(distance);
		H(0,1) = (MU(1) - measured_y)/std::sqrt(distance);
		H(1,0) = (MU(1) - measured_y)/distance;
		H(1,1) = (MU(0) - measured_x)/distance;
		H(1,2) = -1;

		return H;
	}


	// states and covariance
	Eigen::Vector3d MU;
	Eigen::Matrix3d SIGMA;

	// identity matrix
	Eigen::Matrix3d I;

	// motion_noise_parameter
	double MOTION_NOISE_NN;
	double MOTION_NOISE_NO;
	double MOTION_NOISE_ON;
	double MOTION_NOISE_OO;

	// measurement_noise_parameter
	double DISTANCE_NOISE_RATE;
	double DIRECTION_NOISE;

};

#endif	// KALMAN_FILTER_H