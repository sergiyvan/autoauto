#pragma once
#include <math/KalmanFilter.h>
#include <math/Types.h>

namespace aa
{
namespace data
{
namespace obstacle
{
namespace util
{

class LinearFilter
{
public:
	typedef ::math::Vec3 Vec3;
	typedef ::math::Vec2 Vec2;
	typedef ::math::flt flt;

	enum Dimensions {
		MEASUREMENT_SIZE = 6,
		STATE_SIZE = 9
	};
	typedef math::KalmanFilter<STATE_SIZE, MEASUREMENT_SIZE> Filter;
	typedef Filter::value_type value_type;
	typedef Filter::p_type estimate_cov;

	enum {
		S_X = 0,
		S_Y = 1,
		S_Z = 2,
		V_X = 3,
		V_Y = 4,
		V_Z = 5,
		A_X = 6,
		A_Y = 7,
		A_Z = 8
	};

	/// Initial standard deviations [cm;cm/s;cm/ss]
	enum InitialStandardDeviationValues {
		INITIAL_SD_POSITION             = 50,
		INITIAL_SD_VELOCITY             = 125,
		INITIAL_SD_ACCELERATION         = 160,
		INITIAL_SD_PROCESS              = 2,
		INITIAL_SD_GOOD_POS_MEASUREMENT = 10,
		INITIAL_SD_GOOD_VEL_MEASUREMENT = 10,
		INITIAL_SD_BAD_POS_MEASUREMENT  = 10000000,
		INITIAL_SD_BAD_VEL_MEASUREMENT  = 10000000
	};

	LinearFilter(
		Vec3 const & initialPosition,
		Vec3 const & initialVelocity    = Vec3(0.0, 0.0, 0.0),
		value_type sdPosition           = INITIAL_SD_POSITION * 0.01,
		value_type sdVelocity           = INITIAL_SD_VELOCITY * 0.01,
		value_type sdAcceleration       = INITIAL_SD_ACCELERATION * 0.01,
		value_type sdProcess            = INITIAL_SD_PROCESS * 0.01,
		value_type sdGoodPosMeasurement = INITIAL_SD_GOOD_POS_MEASUREMENT * 0.01,
		value_type sdGoodVelMeasurement = INITIAL_SD_GOOD_VEL_MEASUREMENT * 0.01,
		value_type sdBadPosMeasurement  = INITIAL_SD_BAD_POS_MEASUREMENT * 0.01,
		value_type sdBadVelMeasurement  = INITIAL_SD_BAD_VEL_MEASUREMENT * 0.01);

	~LinearFilter();

	LinearFilter(LinearFilter const & o);
	LinearFilter & operator=(LinearFilter const & o);

	/// Getter
	Vec3 estimatedPosition() const {
		return Vec3(mKalmanFilter.x(S_X, 0), mKalmanFilter.x(S_Y, 0), mKalmanFilter.x(S_Z, 0));
	}
	Vec2 estimatedPosition2D() const {
		return Vec2(mKalmanFilter.x(S_X, 0), mKalmanFilter.x(S_Y, 0));
	}
	Vec3 estimatedVelocity() const {
		return Vec3(mKalmanFilter.x(V_X, 0), mKalmanFilter.x(V_Y, 0), mKalmanFilter.x(V_Z, 0));
	}
	Vec3 estimatedAcceleration() const {
		return Vec3(mKalmanFilter.x(A_X, 0), mKalmanFilter.x(A_Y, 0), mKalmanFilter.x(A_Z, 0));
	}
	Vec3 predictedPosition() const {
		return Vec3(mKalmanFilter.x_pred(S_X, 0), mKalmanFilter.x_pred(S_Y, 0), mKalmanFilter.x_pred(S_Z, 0));
	}
	Vec2 predictedPosition2D() const {
		return Vec2(mKalmanFilter.x_pred(S_X, 0), mKalmanFilter.x_pred(S_Y, 0));
	}
	Vec3 predictedVelocity() const {
		return Vec3(mKalmanFilter.x_pred(V_X, 0), mKalmanFilter.x_pred(V_Y, 0), mKalmanFilter.x_pred(V_Z, 0));
	}
	Vec3 predictedAcceleration() const {
		return Vec3(mKalmanFilter.x_pred(A_X, 0), mKalmanFilter.x_pred(A_Y, 0), mKalmanFilter.x_pred(A_Z, 0));
	}
	value_type positionStandardDeviation() const {
		return std::sqrt(mKalmanFilter.P(S_X, S_X) + mKalmanFilter.P(S_Y, S_Y) + mKalmanFilter.P(S_Z, S_Z));
	}
	value_type velocityStandardDeviation() const {
		return std::sqrt(mKalmanFilter.P(V_X, V_X) + mKalmanFilter.P(V_Y, V_Y) + mKalmanFilter.P(V_Z, V_Z));
	}
	value_type accelerationStandardDeviation() const {
		return std::sqrt(mKalmanFilter.P(A_X, A_X) + mKalmanFilter.P(A_Y, A_Y) + mKalmanFilter.P(A_Z, A_Z));
	}
	value_type positionStandardDeviation2D() const {
		return std::sqrt(mKalmanFilter.P(S_X, S_X) + mKalmanFilter.P(S_Y, S_Y));
	}
	value_type positionStandardDeviationX() const {
		return std::sqrt(mKalmanFilter.P(S_X, S_X));
	}
	value_type positionStandardDeviationY() const {
		return std::sqrt(mKalmanFilter.P(S_Y, S_Y));
	}
	value_type velocityStandardDeviation2D() const {
		return std::sqrt(mKalmanFilter.P(V_X, V_X) + mKalmanFilter.P(V_Y, V_Y));
	}
	value_type velocityStandardDeviationX() const {
		return std::sqrt(mKalmanFilter.P(V_X, V_X));
	}
	value_type velocityStandardDeviationY() const {
		return std::sqrt(mKalmanFilter.P(V_Y, V_Y));
	}
	value_type accelerationStandardDeviation2D() const {
		return std::sqrt(mKalmanFilter.P(A_X, A_X) + mKalmanFilter.P(A_Y, A_Y));
	}
	Filter const & filter() const {
		return mKalmanFilter;
	}
	flt deltaT() const {
		return mKalmanFilter.A(S_X, V_X);
	}
	estimate_cov const & estimateCovarainceMatrix() const {
		return mKalmanFilter.P;
	}

	void setQk(value_type varPos, value_type varVel, value_type varAcc);
	void setRkPos(value_type varPos, value_type varVel);
//	void setRkVel(value_type varPos, value_type varVel);

	/// Prediction
	void predict(value_type deltaT);

	/// Update

	void measurementUpdate(Vec3 const & pos, Vec3 const & vel, Filter::r_type const & r);

	void position_measurement_update(Vec3 const & pos);
//	void position_measurement_update(Vec3 const & pos, Filter::r_type const & r);
//	void position_measurement_update(Filter::measurement_type const & zk);
//	void velocity_measurement_update(Vec3 const & vel);
//	void velocity_measurement_update(Filter::measurement_type const & zk);

	void updateWithoutMeasurement();

	/// Print
	void printPrediction() const;
	void printEstimation() const;
	void printEstimateCovarianceMatrix() const;

	void printSystemMatrix() const;
	void printObservationMatrix() const;
	//void printMeasurementCovarianceMatrix() const;
	//void printPropagationCovarianceMatrix() const;

	//	void print(bool state, bool covariance) const;

	// Debug
	Vec2 measurement;

private:

	void measurementUpdate(Filter::measurement_type const & zk, Filter::r_type const & r);

	/// Initialisation
	void processMatrix(Filter::a_type & A, value_type deltaT) const;
	static Filter::h_type observationMatrix();
	static Filter::p_type errorCovMatrix(value_type varPosition,
										 value_type varVelocity,
										 value_type varAcceleration,
										 Vec3 const & initialVelocity);

	/// The kalman filter
	Filter mKalmanFilter;

	/// State propagation covariance matrix [statesize x statesize]
	Filter::q_type mQk;
	/// Measurement covariance matrix that corresponds to a precise position
	/// measurement and a unreliable velocity measurement [measurementsize x measurementsize]
	Filter::r_type mRk_pos;
	/// Measurement covariance matrix that corresponds to a precise velocity
	/// measurement and a unreliable position measurement [measurementsize x measurementsize]
//	Filter::r_type mRk_vel;


	/**
	* \param[in] position Initial position
	* \param[in] velocity Initial velocity
	* Initiates a new KalmanFilter for this obstacle with an initial position and
	* an initial velocity
	*/
	Filter::state_type convert(
		Vec3 const & position,
		Vec3 const & velocity,
		Vec3 const & acceleration) const;


	Filter::measurement_type convert(
		Vec3 const & position,
		Vec3 const & velocity) const;

};

}
}
}
}
