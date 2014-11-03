#pragma once

#include <boost/noncopyable.hpp>
#include <math/Types.h>

#include <modules/models/carstate/CarState.h>
#include <modules/models/carstate/AuxDevicesData.h>

namespace aa
{
namespace modules
{
namespace nav
{
namespace simulator
{

class Simulant
	: boost::noncopyable
{
public:
	typedef ::math::flt flt;
	typedef ::math::Vec3 Vec3;
	typedef ::math::Quaternion Quaternion;
	typedef MATH_TYPES_VEC(flt, 4) Vec4;
	explicit Simulant(int type);
	virtual ~Simulant();

	virtual void simulate(flt timeStep);

	bool immovable;
	bool dirty;
	int typ;
	unsigned int id;

	virtual void setPosition(Vec3 const & _pos) {
		mPos = _pos;
	}

	virtual void setOrientation(Quaternion const & _orientation) {
		mOrientation = _orientation;
	}

	virtual void setVelocity(Vec3 const & _velocity) {
		mVelocity = _velocity;
	}

	virtual void setSteer(flt const & _steer) {
		mSteer = _steer;
	}

	virtual void setAcceleration(Vec3 const & _acceleration) {
		mAcceleration = _acceleration;
	}

	virtual void setAuxDevicesData(AuxDevicesData const & _auxData) {
		mAuxDevicesData = _auxData;
	}

	Vec3 const & pos() const {
		return mPos;
	}

	Quaternion const & orientation() const {
		return mOrientation;
	}

	Vec3 velocity() const {
		return mVelocity;
	}

	Vec3 acceleration() const {
		return mAcceleration;
	}

	Vec3 size;
	Vec4 material;
	std::string name;
	std::string model;
	Vec3 modelscale;

private:
	Vec3 mPos;
	Quaternion mOrientation;
	Vec3 mVelocity;
	flt mSteer;
	Vec3 mAcceleration;
	AuxDevicesData mAuxDevicesData;
};

} // namespace modules
} // namespace nav
} // namespace simulator
}
