/**
 * \file EgoStateFilter.cpp
 * \author Georg Bremer
 */

#include "EgoStateFilter.h"
#include <util/TaskContextFactory.h>
#include <util/OrocosHelperFunctions.h>

using namespace aa::modules::models::egostate;
using namespace ::math;

REGISTERTASKCONTEXT(EgoStateFilter);

EgoStateFilter::EgoStateFilter(const std::string & name)
	: util::RtTaskContext(name)
	, mEgoStateIn("EgoStateIn")
	, mEgoStateOut("EgoStateOut")
	, mBufferSize("BufferSize", "the size of the internal buffer used for averaging", 100)
{
	ports()->addEventPort(mEgoStateIn);
	ports()->addPort(mEgoStateOut);
	addProperty(mBufferSize);
}

EgoStateFilter::~EgoStateFilter()
{
}

bool EgoStateFilter::startHook()
{
	REQUIRED_PORT(mEgoStateIn);
	return true;
}

void EgoStateFilter::stopHook()
{
}

void EgoStateFilter::updateHook()
{
	if (mEgoStateBuffer.capacity() != mBufferSize) {
		mEgoStateBuffer.clear();
		mFilteredEgoState = TimedEgoState();
		mEgoStateBuffer.set_capacity(mBufferSize);
	}

	if (mEgoStateBuffer.full()) {
		Vec3 filteredRpy = mFilteredEgoState.rollPitchYaw();
		Vec3 frontRpy = mEgoStateBuffer.front().rollPitchYaw();
		Vec3 newRpy(
			filteredRpy(0) - frontRpy(0) * (flt(1) / mBufferSize),
			filteredRpy(1) - frontRpy(1) * (flt(1) / mBufferSize),
			filteredRpy(2)
		);
		mFilteredEgoState.setRollPitchYaw(newRpy);
	}

	TimedEgoState egoIn;
	mEgoStateIn.read(egoIn);

	mEgoStateBuffer.push_back(egoIn);

	Vec3 rpy = egoIn.rollPitchYaw();
	Vec3 filteredRpy = mFilteredEgoState.rollPitchYaw();
	Vec3 newRpy(
		filteredRpy(0) + rpy(0) * (flt(1) / mBufferSize),
		filteredRpy(1) + rpy(1) * (flt(1) / mBufferSize),
		rpy(2)
	);
	mFilteredEgoState = egoIn;
	mFilteredEgoState.setRollPitchYaw(newRpy);
	mEgoStateOut.write(mFilteredEgoState);
}

