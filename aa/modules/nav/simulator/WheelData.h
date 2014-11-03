#pragma once
#include <math/Types.h>

//This struct stores individual data of all the wheels
namespace aa
{
namespace modules
{
namespace nav
{
namespace simulator
{

class WheelData
{
public:
	typedef ::math::flt flt;
	typedef ::math::Vec3 Vec3;
	WheelData()
		: radius(0.0f)
	{}

	Vec3 lfPosition;
	Vec3 rfPosition;
	Vec3 lrPosition;
	Vec3 rrPosition;

	Vec3 lfDirection;
	Vec3 rfDirection;
	Vec3 lrDirection;
	Vec3 rrDirection;
	flt radius;
};

} // namespace modules
} // namespace nav
} // namespace simulator
}
