#include "IbeoData.h"

namespace aa
{
namespace data
{
namespace obstacle
{
namespace util
{

namespace ecudata
{

unsigned int convertFlagToCommon(unsigned int flag)
{
	return flag & 15;
}

}

namespace luxdata
{

unsigned int convertFlagToCommon(unsigned int flag)
{
	flag = flag & 15;
	unsigned int common = 0;

	if (flag & TRANSPARENT_MASK) {
		common |= BITMASK_TRANSPARENT;
	}

	if (flag & CLUTTER_MASK) {
		common |= BITMASK_CLUTTER;
	}

	if (flag & GROUND_MASK) {
		common |= BITMASK_GROUND;
	}

	if (flag & DIRT_MASK) {
		common |= BITMASK_DIRT;
	}

	return common;
}

}

}
}
}
}