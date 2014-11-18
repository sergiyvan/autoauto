#pragma once
#include <math/AutoMath.h>
#include <math/Rotate.h>

#include <iostream>
#include <modules/models/carstate/CarState.h>

#define SIMPLE_CARMODEL

namespace aa
{
namespace modules
{
namespace nav
{
namespace simulator
{

struct CarPose {
	typedef ::math::flt flt;
	typedef ::math::Vec3 Vec3;
	typedef ::math::Quaternion Quaternion;

	CarPose()
		: pos(Vec3::Zero())
		, orientation()
		, curvature(0)
		, speed(0)
	{}

	CarPose(Vec3 const & _pos, Quaternion const & _orientation, flt _curvature, flt _speed)
		: pos(_pos)
		, orientation(_orientation)
		, curvature(_curvature)
		, speed(_speed)
	{}

	Vec3 pos;
	Quaternion orientation;
	flt curvature;
	flt speed;
};

class CarModel
{
public:
	typedef CarPose::flt flt;
	typedef CarPose::Vec3 Vec3;
	typedef ::math::Vec2 Vec2;
	typedef CarPose::Quaternion Quaternion;

	explicit CarModel(RTT::PropertyBag const & properties);

	~CarModel()
	{}

	flt frontShaftDistance() const {
		return mFrontShaftDistance;
	}

	void setFrontShaftDistance(flt frontShaftDistance) {
		mFrontShaftDistance = frontShaftDistance;
		mShaftDistance = mFrontShaftDistance + mRearShaftDistance;
	}

	flt rearShaftDistance() const {
		return mRearShaftDistance;
	}

	void setRearShaftDistance(flt rearShaftDistance) {
		mRearShaftDistance = rearShaftDistance;
		mShaftDistance = mFrontShaftDistance + mRearShaftDistance;
	}

	flt shaftDistance() const {
		return mShaftDistance;
	}

	void control(flt speed, flt steer, flt timeStep) {
		assert(!std::isnan(speed));
		assert(!std::isnan(steer));
		assert(!std::isnan(timeStep));

		flt acceleration = adjustAcceleration(speed);
		flt newSteer = adjustSteer(steer, timeStep);
		currSteer = newSteer;

		mWheelDir(0) = sin(currSteer);
		mWheelDir(1) = cos(currSteer);

		assert(std::abs(mWheelDir(1)) > 1e-6);	// We can't have a steering angle of almost 90°

		flt curvature  = mWheelDir(0) / (mWheelDir(1) * mShaftDistance);	// = tan(currSteer) / mShaftDistance

		simulate(acceleration, curvature, timeStep);

	}

	void simulate(flt acceleration, flt curvature, flt timeStep) {
		flt newSpeed;

// 		std::cout << "  before:" << currSpeed << "  " << acceleration << std::endl;
        if (mGear == ::modules::models::carstate::CarState::GEAR_REVERSE) {
			newSpeed = math::rangeCut(-mMaxReverseSpeed, currSpeed + acceleration * timeStep, flt(0));
		}

        else if (mGear == ::modules::models::carstate::CarState::GEAR_SPORT) {
			newSpeed = math::rangeCut(flt(0), currSpeed + 1.5 * acceleration * timeStep, mMaxSpeed);
		}
		else {
			newSpeed = math::rangeCut(flt(0), currSpeed + acceleration * timeStep, mMaxSpeed);
		}

// 		std::cout << "  after:" << newSpeed << std::endl;

		// make it true to use the dynamic model (it does not work)
#if defined(SIMPLE_CARMODEL)

		flt const avgSpeed = 0.5 * (currSpeed + newSpeed);
		mCurvature = curvature;
		mYawRate = mCurvature * avgSpeed;
// 		std::cout << "mYawRate:" << currSteer << std::endl;
		// here we compute the new direction of our car simple version
		flt const dangle = mYawRate * timeStep;
#if defined(USE_EIGEN)
		Quaternion dirChange;
		dirChange = Eigen::AngleAxis<flt>(dangle, Vec3::UnitZ());
		Vec3 oldMovementDirection = orientation * Vec3::UnitX();
#else
		Quaternion dirChange(Vec3(0.f, 0.f, 1.f), dangle);
		Vec3 oldMovementDirection = orientation.rotate(Vec3(1, 0, 0));
#endif
		orientation = orientation * dirChange;
#if defined(USE_EIGEN)
		movementDirection = orientation * Vec3::UnitX();
#else
		movementDirection = orientation.rotate(Vec3(1, 0, 0));
#endif
		Vec3 movementvector = movementDirection * flt(timeStep * avgSpeed);
		pos += movementvector + (oldMovementDirection - movementDirection) * flt(rearShaftDistance());
// 		std::cout << "Movement:" << movementvector(0) << "," << movementvector(1) << std::endl;
		currSpeed = newSpeed;
#else

		// dynamic model
		/// gesucht Ux,Uy

		flt stiff = mTireStiffness;
		speed *= 4.0f;//p_curSpeed;
		steer = carModel.currSteer;
		flt A = mFrontShaftDistance;
		flt B = mRearShaftDistance;
		flt m = mass;
		flt I = torque;

		flt atantermA = 0.0f;
		flt atantermB = 0.0f;

		if (Ux != 0.0f) {
			atantermA = atan((Uy + yaw * A) / Ux);
			atantermB = atan((Uy + yaw * B) / Ux);
		}

		flt UxDiff = Uy * yaw + (speed + speed * cos(steer) - (-stiff * tan(atantermA - steer)) * sin(steer)) / m ;
		flt UyDiff = Ux * yaw + ((-stiff * tan(atantermB)) + speed * sin(steer) + (-stiff * tan(atantermA - steer)) * cos(steer)) / m;

		flt yawDiff = (A * speed * sin(steer) + A * (-stiff * tan(atantermA - steer)) * cos(steer) - B * (-stiff * tan(atantermB))) / I;

		Ux += UxDiff * timeStep;
		Uy += UyDiff * timeStep;
		yaw += yawDiff * timeStep;
		//cout << "Ux:" << Ux << ", Uy:" << Uy << ", yaw:" << yaw << endl;

		p_currDirection.set(rotate::rotateZ(p_currDirection,
											yaw * timeStep).normalize());


		Vec3 side = Vec3(currDirection(1),	-currDirection(0), 0.0f);
		movementvector = (p_currDirection * (flt)Ux + side * (flt)Uy) * timeStep;
#endif
		// We also set our wheel directions
	}

	flt adjustAcceleration(flt speedCtrl) const {
		// when in reverse mode we need to invert the speed in the end, and also use
		// different speed-values
		// TODO: also use a different adjustSpeed because the acceleration will also be different
		//       on reverse, but that needs test-data in reverse
        if (mGear == ::modules::models::carstate::CarState::GEAR_REVERSE) {
			return -adjustSpeed(speedCtrl, -currSpeed);
		}
		else {
			return adjustSpeed(speedCtrl, currSpeed);
		}
	}

	flt adjustSpeed(flt speedCtrl, flt _currSpeed) const {
		if (speedCtrl > 1.0 || speedCtrl < -1.0) {
			return 0.0;
		}


		//new passat car speed model

		if (mCarName == "mig") {
			if (speedCtrl >= 0.0) {
				//Bug correction 4.0 V Bug:
				speedCtrl = (speedCtrl * 3.25 + 0.75) / 4.0;

				flt const coefficient = 4.1 / (1.0 + pow(flt(2.709), flt(15.0 * (0.55 - speedCtrl)))) + 0.4;

				flt time0 = pow(_currSpeed / coefficient, flt(1.25));
				flt vel2 = coefficient * pow((time0 + 1.0), 0.8);

                flt a = (vel2 - _currSpeed) - _currSpeed*_currSpeed*0.0005;

				//std::cout << "<A speedControl " << speedCtrl << " Acc " << a << " time " << time0 << " currSpeed " << _currSpeed << std::endl;

				if ((currSpeed >= 3) && (speedCtrl <= 0.1)) {
					a = 0.0;
				}

				return a;
			}
			else if (speedCtrl < 0.0) {
				flt const coefficient = 13.0 * pow(-speedCtrl, flt(0.8)) + 0.2;
				flt time0 = _currSpeed / coefficient;

				flt vel2 = coefficient * (time0 - 1.0);

				if (vel2 < 0) {
					vel2 = 0.0;
				}

				flt a = vel2 - _currSpeed;

				//std::cout << " >B speedControl " << speedCtrl << " Acc " << a << " time " << time0 << " currSpeed " << _currSpeed << std::endl;

				return a;
			}
			else {
				std::cout << "Bug In Carmodel Mig Acc" << std::endl;
				return 0;
			}
		}

		// fi mig

		if (speedCtrl > 0.5) {
			if (currSpeed > 4.0) {
				return mMaxSpeedChange * ((speedCtrl - 0.5) * 4.0 / (_currSpeed * 0.25));
			}
			else {
				return mMaxSpeedChange * (speedCtrl - 0.5) * 4.0;
			}
		}
		else if (speedCtrl > -0.1) {
			if (currSpeed > speedCtrl + 0.8) {
				return -mMaxSpeedChange * (speedCtrl + 0.1) * 4.0;
			}
			else {
				return mMaxSpeedChange * (speedCtrl + 0.1) * 2.0;
			}
		}
		else {
			return mMaxBreakChange * (speedCtrl + 0.1);
		}
	}

	flt adjustSteer(flt steer, flt timeStep) {
		assert(!std::isnan(currSteer));



		/////SIMULATE STEER DELAY STARTS

		if (mCarName == "mig") {
			mRingBufferCommands[mIndexOfCommandRequested] = steer;

			flt delayInSeconds = 0.0;

			//for Debuggin overwrite delay
			//delayInSeconds = 0.1; //Hack by Daniel
			int delayInFrames = (int)(delayInSeconds / /*getPeriod()*/ 0.02); // Delay = 0.1s, Frequency = 50 Hz = 1/0.02

			//delayInFrames = 0; ///For debugging set to zero

			//		int delayInFrames = 5;	//50 Hz = 100ms
			steer = mRingBufferCommands[(mIndexOfCommandRequested+mRingBufferCommands.size()-delayInFrames)%mRingBufferCommands.size()];

			mIndexOfCommandRequested++;
			mIndexOfCommandRequested = mIndexOfCommandRequested % mRingBufferCommands.size();


			/////DELAY ENDS
			mCurrentSteerError = steer - currSteer;
			mSteerIntegral = math::rangeCut(-mSteerIntegralLimit, mSteerIntegral += mCurrentSteerError, mSteerIntegralLimit);
			mSteerMomentum = math::rangeCut(-10.0, 0.2 * 530 / 0.59 * mCurrentSteerError /*+ 0.005 * 530 / 0.59 * 2 * mSteerIntegral*/ + 1000 * (mCurrentSteerError - mOldSteerError), 10.0);


			mCurrentSteerSpeed += mSteerMomentum / 10 * 2.5 * mMaxSteerAngleAcceleration * timeStep;

			mCurrentSteerSpeed = math::rangeCut(-mMaxSteerChange, mCurrentSteerSpeed, mMaxSteerChange);

			//if (mCurrentSteerSpeed > 0) {mCurrentSteerSpeed = std::max(0.0, mCurrentSteerSpeed - 0.05 * mMaxSteerAngleAcceleration * timeStep);}
			//if (mCurrentSteerSpeed < 0) {mCurrentSteerSpeed = std::min(0.0, mCurrentSteerSpeed + 0.05 * mMaxSteerAngleAcceleration * timeStep);}


			//mCurrentSteerSpeed *= std::pow(0.90, timeStep / 0.02);


			mOldSteerError = mCurrentSteerError;

			return math::rangeCut(-mMaxSteer, currSteer + mCurrentSteerSpeed * timeStep, mMaxSteer);
		}








		// Spob Steering
		if (steer > currSteer) {
			return std::min(steer, currSteer + mMaxSteerChange * timeStep);
		}
		else if (steer < currSteer) {
			return  std::max(steer, currSteer - mMaxSteerChange * timeStep);
		}

		return steer;
	}


	/**
	* @brief computes the drifting factor for the rear wheels
	* @param
	* @return driftVector
	*/
	Vec3 getDriftFactorRear() const {
		if (currSpeed == 0.0f) {
			return Vec3(0.0f, 0.0f, 0.0f);
		}

		//TODO: compute tire rotation speed
		flt const omega = currSpeed;

// 		flt const A = mFrontShaftDistance;
		flt const B = mRearShaftDistance;
		flt const Ux = 1.0f; // cos(dot_product(currDirection, movingDirection));
		flt const Uy = 0.01f;// sin(dot_product(currDirection, movingDirection));

		flt const alpha = (atan(Ux - mYawRate * B) / Uy) - currSteer;
		flt const Vy = currSpeed;
		//
		flt const Vx = tan(alpha) / Vy;

		// here we compute the forward tire force Fx/Fy
		flt const Fx =  -mTireStiffness * tan(alpha);

		flt const Fy =  -mTireStiffness * ((Vx - mTireRadius * omega) / Vy);

		return Vec3(Fx, Fy, 0.0f);
	}


	/**
	* @brief computes the drifting factor for the front wheels
	* @param
	* @return driftVector
	*/
	Vec3 getDriftFactorFront() const {
		if (currSpeed == 0.0f) {
			return Vec3(0, 0, 0);
		}

		//TODO: compute tire rotation speed
		flt const omega = currSpeed;

		flt const A = mFrontShaftDistance;
// 		flt const B = mRearShaftDistance;
		flt const Ux = 1.0f; // cos(dot_product(currDirection, movingDirection));
		flt const Uy = 0.01f; // sin(dot_product(currDirection, movingDirection));

		flt const alpha = (atan(Ux + mYawRate * A) / Uy) - currSteer ;
		flt const Vy = currSpeed;
		//
		flt const Vx = tan(alpha) / Vy;

		// here we compute the forward tire force Fx/Fy
		flt const Fx =  -mTireStiffness * tan(alpha);

		flt Fy =  -mTireStiffness * ((Vx - mTireRadius * omega) / Vy);
		//	cout << "U:" << Ux << "," << Uy << ",yaw:" << yawRate << ",alpha:" << alpha << ",F:" << Fx << "," << Fy << endl;
		//std::cout << "drift front : " << Fx << "," << Fy << std::endl;
		return Vec3(Fx, Fy, 0.0f);
	}

	/**
	 * State of the vehicle
	 */
	CarPose pose() const {
		return CarPose(pos, orientation, mCurvature, currSpeed);
	}

	flt currSpeed;	// Longitudinal speed
	flt currSteer;	// Steering angle

	Vec3 pos;
	Quaternion orientation;
	Vec3 movementDirection;

	int		mGear;				//gear position: -1=error, 0=between gears, 1=P, 2=R, 4=N, 8=D

private:
	flt mFrontShaftDistance;
	flt mRearShaftDistance;
	flt mShaftDistance;		// Has to be placed behind Rear + Front
	flt mMaxSpeedChange;
	flt mMaxSteer;
	flt mMaxSteerChange;
	flt mMaxSteerAngleAcceleration;
	flt mMaxBreakChange;
	flt mMaxSpeed;
	flt mMaxReverseSpeed;
	flt mTireStiffness;
	flt mTireRadius;
	flt mCurrentSteerSpeed;
	flt mSteerIntegral;
	flt mSteerIntegralLimit;
	flt mCurrentSteerError;
	flt mOldSteerError ;
	flt mSteerMomentum;
	std::string mCarName;

	flt	mYawRate;
	flt	mCurvature;
	Vec2	mWheelDir;
	boost::array<flt, 100> mRingBufferCommands;
	unsigned int mIndexOfCommandRequested;
};

#undef SIMPLE_CARMODEL

} // namespace modules
} // namespace nav
} // namespace simulator
}
