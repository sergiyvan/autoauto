#pragma once
/*!
 * \file AbstractOcclusionMap.h
 * \brief TODO Brief description.
 */

#include <boost/smart_ptr.hpp>
#include <core/TimedData.h>
#include <math/AutoMath.h>
#include <math/Types.h>


namespace aa
{
namespace data
{
namespace obstacle
{
namespace util
{

class AbstractOcclusionMap;

typedef boost::shared_ptr<AbstractOcclusionMap> SharedOcclusionMap;
typedef boost::shared_ptr<const AbstractOcclusionMap> SharedConstOcclusionMap;

/*!
* \class AbstractOcclusionMap
* \brief TODO Brief description.
*
* TODO Detailed description of AbstractOcclusionMap.
*/
class AbstractOcclusionMap
{
public:
	// Definitions:
	typedef boost::shared_ptr<AbstractOcclusionMap> Ptr;
	typedef TimedData<AbstractOcclusionMap> Timed;
	typedef boost::shared_ptr<Timed> TimedPtr;
	typedef TimedData<Ptr> PtrTimed;
	typedef math::flt flt;
	typedef math::Vec3 Vec3;
	typedef math::Vec2 Vec2;


	// Constructors:
	AbstractOcclusionMap();

	// Destructor:
//	~AbstractOcclusionMap();

	// Selectors (getters):
	virtual flt occludedProbability(Vec3 const &) const = 0;
	virtual flt occludedProbability(Vec3 const & p, Vec2 const & angles) const = 0;
	virtual Vec2 getAngles(Vec3 const & point) const = 0;
//	virtual bool isInRange(Vec3 const &) = 0;
	//virtual osg::ref_ptr<osg::HeightField> getProbabilityHeightField(vector<AbstractOcclusionMap> const&)=0;

	// Modifiers (setters):


	// Functions:

protected:
	// Functions:

	// Attributes:

private:
	// Functions:

	// Attributes:

};

}
}
}
}
