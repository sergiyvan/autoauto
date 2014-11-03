#pragma once
/*!
 * \file FieldOfView.h
 * \brief Describes the field of view of a sensor as a polygon.
 */

#include <boost/smart_ptr.hpp>
#include <core/TimedData.h>
#include <math/Polygon.h>
#include <math/Types.h>

namespace aa
{
namespace data
{
namespace obstacle
{
namespace util
{


/*!
* \class FieldOfView
* \brief Describes the field of view of a sensor as a polygon.
*
* TODO Detailed description of FieldOfView.
*/
class FieldOfView
	: public math::Polygon<math::flt>
{
public:
	// Definitions:
	typedef boost::shared_ptr<FieldOfView> Ptr;
	typedef TimedData<FieldOfView> Timed;
	typedef boost::shared_ptr<Timed> TimedPtr;
	typedef TimedData<Ptr> PtrTimed;
	typedef ::math::flt flt;
	typedef ::math::Vec2 Vec2;
	typedef ::math::Vec3 Vec3;
	typedef ::math::Affine3 Affine3;

	// Constructors:
	FieldOfView();

	// Destructor:
	~FieldOfView();


	FieldOfView & operator=(FieldOfView const & o);
	FieldOfView(FieldOfView const & o);
	FieldOfView & operator=(math::Polygon<math::flt> const & o);


	void transform(Affine3 const & m);

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
