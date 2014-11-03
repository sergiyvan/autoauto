#pragma once

#include <util/RtTaskContext.h>

#include <aa/modules/nav/controller/Plan.h>
#include <modules/models/egostate/EgoState.h>

namespace aa
{
namespace modules
{
namespace nav
{

namespace behaviour
{




/*!
* \brief  CarFollowingBehaviour module
*
*/
class CarFollowingBehaviour
	: public util::RtTaskContext
{
public:
	enum swerve_type {
		NONE				=   0,
		SHIFT				=   1,
		SWERVE				=   2,
		NUDGE				=   4,
		LANE_CHANGE			=   8
	};

	typedef ::math::flt flt;
	typedef ::math::Vec2 Vec2;


	typedef std::vector< std::vector< std::vector< aa::modules::nav::controller::Plan_ptr > > > MicroPlanVec;		//vector of lanes of shifts of plans

	typedef math::PolySpline<Vec2, flt, 4u> SplineType;

	explicit CarFollowingBehaviour(std::string const & name);
	virtual ~CarFollowingBehaviour();

	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();
	virtual void errorHook();

protected:
	/** @name Attributes: */
    int mCounter;


	/** @name Properties: */
	/*! \{ */
    flt mFilterDist;
    flt mCheckDist;
    /*! \} */

	/** @name InputPorts: */
	/*! \{ */
	RTT::InputPort< TimedEgoState > mEgoStateIn;					/** reads own egostate */
	/*! \} */

	/** @name OutputPorts: */
	/*! \{ */
	RTT::OutputPort< aa::modules::nav::controller::Plan_ptr > mPlanOut;							/** write out a selected plan */
	/*! \} */

	/** @name Methods: */
	/*! \{ */
	/*! \} */
    bool ReplanNow();

	RTT::TaskContext * mStateMachine;

private:

	TimedEgoState mCurEgoState;

	aa::modules::nav::controller::Plan_ptr mPlan;

    std::vector<std::pair<math::Vec3, math::Vec3 > > mGpsTrack;

};

}


}

 // modules
}

 // nav
}

 // behaviour
