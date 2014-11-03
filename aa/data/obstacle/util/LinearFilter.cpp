#include "LinearFilter.h"

#include <math/AutoMath.h>
#include <iostream>

using namespace math;
using namespace aa::data::obstacle::util;

LinearFilter::LinearFilter(Vec3 const & initialPosition,
						   Vec3 const & initialVelocity,
						   value_type sdPosition,
						   value_type sdVelocity,
						   value_type sdAcceleration,
						   value_type sdProcess,
						   value_type sdGoodPosMeasurement,
						   value_type sdGoodVelMeasurement,
						   value_type sdBadPosMeasurement,
						   value_type sdBadVelMeasurement)
	: mKalmanFilter(
		// Process Matrix A (is set before each prediction, depends on time offset)
		Filter::a_type(),
		// Observation Matrix H (constant)
		observationMatrix(),
		// Initial state x
		convert(initialPosition, initialVelocity, Vec3(0.0, 0.0, 0.0)),
		// Initial Error covariance matrix P
		errorCovMatrix(math::sqr(sdPosition), math::sqr(sdVelocity), math::sqr(sdAcceleration), initialVelocity))
//		errorCovMatrix(math::sqr(sdPosition), math::sqr(0.1), math::sqr(sdAcceleration)))
	, measurement(initialPosition[0], initialPosition[1])
{
	setQk(math::sqr(sdProcess), math::sqr(sdProcess), math::sqr(sdProcess) * 100.0);
	setRkPos(math::sqr(sdGoodPosMeasurement), math::sqr(sdBadVelMeasurement));
//	setRkVel(math::sqr(sdBadPosMeasurement), math::sqr(sdGoodVelMeasurement));
}


LinearFilter::LinearFilter(LinearFilter const & o)
	: mKalmanFilter(o.mKalmanFilter)
	, mQk(o.mQk)
	, mRk_pos(o.mRk_pos)
//	, mRk_vel(o.mRk_vel)
	, measurement(o.measurement)
{
}


LinearFilter & LinearFilter::operator=(LinearFilter const & o)
{
	if (&o == this) {
		return *this;
	}

	mKalmanFilter = o.mKalmanFilter;
	mQk = o.mQk;
	mRk_pos = o.mRk_pos;
//	mRk_vel = o.mRk_vel;

	measurement = o.measurement;

	return *this;
}


LinearFilter::~LinearFilter()
{
}


void LinearFilter::predict(value_type deltaT)
{
	processMatrix(mKalmanFilter.A, deltaT);
	mKalmanFilter.predict(mQk);
}


void LinearFilter::measurementUpdate(Vec3 const & pos, Vec3 const & vel, Filter::r_type const & Rk)
{
	measurementUpdate(convert(pos, vel), Rk);
}

void LinearFilter::measurementUpdate(Filter::measurement_type const & zk, Filter::r_type const & Rk)
{
	measurement = Vec2(zk(S_X), zk(S_Y));
	mKalmanFilter.measurement_update(zk, Rk);
}



void LinearFilter::position_measurement_update(Vec3 const & pos)
{
	Filter::measurement_type zk;
	zk(S_X, 0) = pos(S_X);
	zk(S_Y, 0) = pos(S_Y);
	zk(S_Z, 0) = pos(S_Z);
	zk(V_X, 0) = mKalmanFilter.x_pred(V_X);
	zk(V_Y, 0) = mKalmanFilter.x_pred(V_Y);
	zk(V_Z, 0) = mKalmanFilter.x_pred(V_Z);
	measurementUpdate(zk, mRk_pos);
}


//void LinearFilter::position_measurement_update(Filter::measurement_type const & zk)
//{
//	measurement = Vec2(zk(S_X), zk(S_Y));
//	mKalmanFilter.measurement_update(zk, mRk_pos);
//}


//void LinearFilter::velocity_measurement_update(Vec3 const & vel)
//{
//	Filter::measurement_type zk;
//	zk(S_X, 0) = mKalmanFilter.x_pred(S_X);
//	zk(S_Y, 0) = mKalmanFilter.x_pred(S_Y);
//	zk(S_Z, 0) = mKalmanFilter.x_pred(S_Z);
//	zk(V_X, 0) = vel(S_X);
//	zk(V_Y, 0) = vel(S_Y);
//	zk(V_Z, 0) = vel(S_Z);
//	velocity_measurement_update(zk);
//}


//void LinearFilter::velocity_measurement_update(Filter::measurement_type const & zk)
//{
//	measurement = Vec2(zk(S_X), zk(S_Y));
//	mKalmanFilter.measurement_update(zk, mRk_vel);
//}


void LinearFilter::updateWithoutMeasurement()
{
	mKalmanFilter.update_without_measurement();
}

void LinearFilter::setQk(value_type varPos, value_type varVel, value_type varAcc)
{
	mQk.fill(0.0);
	// position process error
	mQk(S_X, S_X) = varPos;
	mQk(S_Y, S_Y) = varPos;
	mQk(S_Z, S_Z) = varPos;
	// velocity process error
	mQk(V_X, V_X) = varVel;
	mQk(V_Y, V_Y) = varVel;
	mQk(V_Z, V_Z) = varVel;
	// acceleration process error
	mQk(A_X, A_X) = varAcc;
	mQk(A_Y, A_Y) = varAcc;
	mQk(A_Z, A_Z) = varAcc;
}

void LinearFilter::setRkPos(value_type varPos, value_type varVel)
{
	mRk_pos.fill(0.0);
	mRk_pos(S_X, S_X) = varPos;
	mRk_pos(S_Y, S_Y) = varPos;
	mRk_pos(S_Z, S_Z) = 10.0 * varPos;
	mRk_pos(V_X, V_X) = varVel;
	mRk_pos(V_Y, V_Y) = varVel;
	mRk_pos(V_Z, V_Z) = 10.0 * varVel;
}

//void LinearFilter::setRkVel(value_type varPos, value_type varVel)
//{
//	mRk_vel.fill(0.0);
//	mRk_vel(S_X, S_X) = varPos;
//	mRk_vel(S_Y, S_Y) = varPos;
//	mRk_vel(S_Z, S_Z) = 10.0 * varPos;
//	mRk_vel(V_X, V_X) = varVel;
//	mRk_vel(V_Y, V_Y) = varVel;
//	mRk_vel(V_Z, V_Z) = 10.0 * varVel;
//}


LinearFilter::Filter::p_type LinearFilter::errorCovMatrix(
	value_type varPos,
	value_type varVel,
	value_type varAcc,
	Vec3 const & initialVelocity)
{
	if (initialVelocity.norm() > 0.0) {
		varVel = math::sqr(0.1);
	}


	Filter::p_type P = Filter::p_type::Zero();
	P(S_X, S_X) = varPos;
	P(S_Y, S_Y) = varPos;
	P(S_Z, S_Z) = varPos;
	P(V_X, V_X) = varVel;
	P(V_Y, V_Y) = varVel;
	P(V_Z, V_Z) = varVel;
	P(A_X, A_X) = varAcc;
	P(A_Y, A_Y) = varAcc;
	P(A_Z, A_Z) = varAcc;
	return P;
}


void LinearFilter::processMatrix(LinearFilter::Filter::a_type & A, value_type deltaT) const
{
	A.fill(0);

	for (unsigned int i = 0; i < STATE_SIZE; i++) {
		A(i, i) = 1.0;
	}

	A(S_X, V_X) = deltaT;
	A(S_X, A_X) = 0.5 * math::sqr(deltaT);
	A(S_Y, V_Y) = deltaT;
	A(S_Y, A_Y) = 0.5 * math::sqr(deltaT);
	A(S_Z, V_Z) = deltaT;
	A(S_Z, A_Z) = 0.5 * math::sqr(deltaT);
	A(V_X, A_X) = deltaT;
	A(V_Y, A_Y) = deltaT;
	A(V_Z, A_Z) = deltaT;
}


LinearFilter::Filter::h_type LinearFilter::observationMatrix()
{
	Filter::h_type H = Filter::h_type::Zero();

	for (unsigned int i = 0; i < MEASUREMENT_SIZE; i++) {
		H(i, i) = 1.0;
	}

	return H;
}


LinearFilter::Filter::state_type LinearFilter::convert(
	Vec3 const & position,
	Vec3 const & velocity,
	Vec3 const & acceleration) const
{
	LinearFilter::Filter::state_type initial_guess;

	// The guessed position
	initial_guess(0, 0) = position(0);
	initial_guess(1, 0) = position(1);
	initial_guess(2, 0) = position(2);

	// The guessed velocity
	initial_guess(3, 0) = velocity(0);
	initial_guess(4, 0) = velocity(1);
	initial_guess(5, 0) = velocity(2);

	// The guessed acceleration
	initial_guess(6, 0) = acceleration(0);
	initial_guess(7, 0) = acceleration(1);
	initial_guess(8, 0) = acceleration(2);

	return initial_guess;
}

LinearFilter::Filter::measurement_type LinearFilter::convert(
	Vec3 const & position,
	Vec3 const & velocity) const
{
	LinearFilter::Filter::measurement_type measurement;

	// Measured position
	measurement(S_X, 0) = position(0);
	measurement(S_Y, 0) = position(1);
	measurement(S_Z, 0) = position(2);

	// Measured velocity
	measurement(V_X, 0) = velocity(0);
	measurement(V_Y, 0) = velocity(1);
	measurement(V_Z, 0) = velocity(2);

	return measurement;
}


/**
*	Print
*/


void LinearFilter::printEstimateCovarianceMatrix() const
{
	std::cout << "Estimate Covariance Matrix" << std::endl;
	std::cout << mKalmanFilter.P;
}

void LinearFilter::printPrediction() const
{
	std::cout << "Prediction: " << std::endl;
	std::cout << mKalmanFilter.x_pred << std::endl;
}

void LinearFilter::printEstimation() const
{
	std::cout << "Estimation: " << std::endl;
	std::cout << mKalmanFilter.x << std::endl;
}

//void LinearFilter::print(bool state, bool covariance) const
//{
//	std::cout << "Prediction" << std::endl;
//	std::cout
//			<< std::endl
//			<< predictedPosition()(0) << " | "
//			<< predictedPosition()(1) << " | "
//			<< predictedPosition()(2)
//			<< std::endl
//			<< predictedVelocity()(0) << " | "
//			<< predictedVelocity()(1) << " | "
//			<< predictedVelocity()(2)
//			<< std::endl
//			<< predictedAcceleration()(0) << " | "
//			<< predictedAcceleration()(1) << " | "
//			<< predictedAcceleration()(2)
//			<< std::endl;


//	if (state) {
//#if 0
//		std::cout << "State" << std::endl;
//		value_type position = estimatedPosition().norm();
//		value_type position_deviationX = std::sqrt(mKalmanFilter.P(S_X, S_X));
//		value_type position_deviationY = std::sqrt(mKalmanFilter.P(S_Y, S_Y));
//		value_type position_deviationZ = std::sqrt(mKalmanFilter.P(S_Z, S_Z));
//		value_type velocity = estimatedVelocity().norm();
//		value_type velocity_deviationX = std::sqrt(mKalmanFilter.P(V_X, V_X));
//		value_type velocity_deviationY = std::sqrt(mKalmanFilter.P(V_Y, V_Y));
//		value_type velocity_deviationZ = std::sqrt(mKalmanFilter.P(V_Z, V_Z));
//		value_type acceleration = estimatedAcceleration().norm();
//		value_type acceleration_deviationX = std::sqrt(mKalmanFilter.P(A_X, A_X));
//		value_type acceleration_deviationY = std::sqrt(mKalmanFilter.P(A_Y, A_Y));
//		value_type acceleration_deviationZ = std::sqrt(mKalmanFilter.P(A_Z, A_Z));

//		std::cout
//				<< std::endl
//				<< estimatedPosition()(0) << "/" << position_deviationX << " | "
//				<< estimatedPosition()(1) << "/" << position_deviationY << " | "
//				<< estimatedPosition()(2) << "/" << position_deviationZ << " | "
//				<< std::endl
//				<< estimatedVelocity()(0) << "/" << velocity_deviationX << " | "
//				<< estimatedVelocity()(1) << "/" << velocity_deviationY << " | "
//				<< estimatedVelocity()(2) << "/" << velocity_deviationZ
//				<< std::endl
//				<< estimatedAcceleration()(0) << "/" << acceleration_deviationX << " | "
//				<< estimatedAcceleration()(1) << "/" << acceleration_deviationY << " | "
//				<< estimatedAcceleration()(2) << "/" << acceleration_deviationZ
//				<< std::endl;
//#endif
//	}

//// 	if ( covariance ) {
//	std::cout << "Covariance" << std::endl;
//	printEstimateCovarianceMatrix();

//// 	}
//	std::cout << "System" << std::endl;
//	printSystemMatrix();
//	std::cout << "Observation" << std::endl;
//	printObservationMatrix();
//	std::cout << "MeasurementCovariance" << std::endl;
//	printMeasurementCovarianceMatrix();
//	std::cout << "PropagationCovariance" << std::endl;
//	printPropagationCovarianceMatrix();
//}

void LinearFilter::printSystemMatrix() const
{
	std::cout << mKalmanFilter.A;
}

void LinearFilter::printObservationMatrix() const
{
	std::cout << mKalmanFilter.H;
}

//void LinearFilter::printEstimateCovarianceMatrix() const
//{
//	std::cout << mKalmanFilter.P;
//}

//void LinearFilter::printMeasurementCovarianceMatrix() const
//{
//	std::cout << mRk;
//}

//void LinearFilter::printPropagationCovarianceMatrix() const
//{
//	std::cout << mQk;
//}

