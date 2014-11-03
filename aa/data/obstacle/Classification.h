#pragma once

namespace aa
{
namespace data
{
namespace obstacle
{
/**
  * definition of Classification status available
  **/
class Classification
{

public:
	enum ClassificationType {
		UNCLASSIFIED	= 0,	// no estimate yet
		PERSON		= 1,	// pole, pedestrian or bike or other small static or dynamic obstacle
		CAR		= 2,
		TRUCK		= 3,
		BIKE		= 4,
		UNKNOWN_BIG	= 5,	// either small car or truck
		SMALL_CAR	= 6,
		UNKNOWN_MEDIUM	= 7,	// car or bike or static obstacle
		ENVIRONMENT	= 8,	// tree, house, fence, or big static structures
		UNKNOWN_HUGE	= 9,	// dynamic object, bigger than truck
		UNKNOWN_SMALL	= 10,
		SIDE_BOUNDARY	= 11
	};

	Classification();

private:

	ClassificationType mClassification;

};

}
}
}
