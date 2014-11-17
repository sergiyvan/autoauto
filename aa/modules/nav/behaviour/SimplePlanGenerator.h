#pragma once

#include <util/RtTaskContext.h>

#include <aa/modules/nav/controller/Plan.h>

namespace aa
{
namespace modules
{
namespace nav
{

namespace behaviour
{




/*!
* \brief  SimplePlanGenerator module
*
*/
class SimplePlanGenerator
	: public util::RtTaskContext
{
public:

	typedef ::math::flt flt;
	typedef ::math::Vec2 Vec2;


	typedef std::vector< std::vector< std::vector< aa::modules::nav::controller::Plan_ptr > > > MicroPlanVec;		//vector of lanes of shifts of plans

	typedef math::PolySpline<Vec2, flt, 4u> SplineType;

    explicit SimplePlanGenerator(std::string const & name);
    virtual ~SimplePlanGenerator();

	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();
	virtual void errorHook();

protected:

	/** @name OutputPorts: */
	/*! \{ */
	RTT::OutputPort< aa::modules::nav::controller::Plan_ptr > mPlanOut;							/** write out a selected plan */
	/*! \} */

	/** @name Methods: */
	/*! \{ */
    bool ReplanNow();
    /*! \} */

	RTT::TaskContext * mStateMachine;

private:

	aa::modules::nav::controller::Plan_ptr mPlan;

};

}


}

 // modules
}

 // nav
}

 // behaviour
