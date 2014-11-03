#include "Zone.h"

using namespace ::math;

namespace aa
{
namespace modules
{
namespace models
{

namespace rndf
{

namespace io
{

Zone::Zone()
	: id(0)
{}

Zone::~Zone()
{}

Zone::Zone(const Zone & z)
	: id(z.id)
	, spots(z.spots)
	, perimeterpoints(z.perimeterpoints)
	, name(z.name)
{}

// int Zone::addSpot(int spotID)
// {
//    Spot sp;
//
//    sp.id = spotID;
//    spots[spotID] = sp;
//    return spotID;
// }
//
// int Zone::addPerimeterPoint(int pptID, flt latit, flt longit)
// {
//    Waypoint pp;
//
//    pp.id = pptID;
//    pp.latitude    = latit;
//    pp.longitude   = longit;
//    pp.isPerimeterPoint   = true;
//    perimeterpoints[pptID] = pp;
//    return pptID;
// }

}


}


}


}


}


