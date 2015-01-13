#pragma once

#include <util/RtTaskContext.h>
#include <math/AutoMath.h>
#include <aa/modules/nav/behaviour/ShortestPath.h>


namespace aa
{
namespace modules
{
namespace models
{
namespace rndf
{
class EdgeData;
}
}
namespace nav
{
namespace simulator
{
class SimObject;

class DummyController
	: public util::RtTaskContext
{
public:
	typedef ::math::Vec3 Vec3;

	explicit DummyController(std::string const & name);
	virtual ~DummyController();

	std::string getNewRNDFPointFrom(std::string const &);
    std::pair<const aa::modules::models::rndf::EdgeData *, std::string> getNewRNDFEdge(std::string const &);
	Vec3 rndfToCoord(std::string const &);
	virtual bool loadSimulation(std::string const & filename);
    std::vector<Vec3> addInterpolatedEdge(aa::modules::models::rndf::edge_descr const & edge);

	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();

	virtual void resetMovableObstacles();

    math::flt mUpdateVelocityStdDev;

private:
	class impl;
	std::auto_ptr<impl> pimpl;
    aa::modules::nav::behaviour::ShortestPath mShortestPath;
};


} // namespace modules
} // namespace nav
} // namespace simulator
}
