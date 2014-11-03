#include <vector>
#include <list>
#include "DummyController.h"

namespace aa
{
namespace modules
{
namespace nav
{
namespace simulator
{

class SimWaypoint
{
public:
	typedef ::math::flt flt;
	typedef ::math::Vec3 Vec3;
	Vec3 position;
	flt speed;
	std::string oldPos;
	SimWaypoint(Vec3 const & p, flt s, std::string const & o = "");
};


class SimObject
{
public:
	typedef ::math::flt flt;
	typedef ::math::Vec3 Vec3;
	typedef ::math::Vec4 Vec4;
	typedef ::math::Quaternion Quaternion;
	typedef ::math::Mat4x4 Mat4x4;
	SimObject(std::string const & t = "box", flt x = 0.0f, flt y = 0.0f, Vec3 const & size = Vec3(1, 1, 1));
	virtual ~SimObject();

	std::string model;
	Vec3 modelscale;

	std::string name;
	bool started;

	virtual void setPosition(Vec3);
	virtual void setDirection(Vec3);
	virtual bool collide(SimObject *);
	virtual void setToPathStartPos();
	virtual void update(DummyController *);
	virtual Vec3 getVelocity();
	virtual Vec3 getAcceleration();
	void addWaypoint(Vec3 const & , flt speed, std::string const & o = "");
	std::pair<Mat4x4, std::string> getDrawData();

	uint id;
	Vec3 position;
	Vec3 direction;
	Vec4 material;
	Quaternion orientation;
	Vec3 size;
	std::list<SimWaypoint> path;
	std::list<SimWaypoint> jumpPath;
	std::string option;
private:
	static uint idsequence;
};


class CarObject
	: public SimObject
{

public:
	CarObject();
	virtual ~CarObject();
};

} // namespace modules
} // namespace nav
} // namespace simulator
}
