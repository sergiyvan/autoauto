#include "Plan.h"

#include <algorithm>
#include <ext/algorithm>	// is_sorted
#include <util/Templates.h>
#include <math/PathSpline.h>
#include <math/AutoMath.h>


// corba port
#include <util/MqPortType.h>

// #define VERBOSE

#if defined(VERBOSE)
#include <iostream>
#include <rtt/Logger.hpp>
#define DLOG(X)	Logger::log() << Logger::Debug << X << Logger::endl
#define DCERR(X) std::cerr << X << std::endl
#else
#define DLOG(X) /**/
#define DCERR(X) /**/
#endif

namespace RTT
{
template class InputPort<aa::modules::nav::controller::Plan>;
template class OutputPort<aa::modules::nav::controller::Plan>;

template class InputPort<aa::modules::nav::controller::Plan_ptr>;
template class OutputPort<aa::modules::nav::controller::Plan_ptr>;
}

namespace aa
{
namespace modules
{
namespace nav
{

namespace controller
{



// ATTENTION!! SERIALIZATION IS NOT FULLY IMPLEMENTED
REGISTER_MQ_PORT_TYPE(Plan);



void assertIsNumber(math::flt x)
{
	assert(!std::isnan(x) && !std::isinf(x));
}


void assertTimeContinuity(TimedData<ASpline>::DomVec const & mX)
{
	assertIsNumber(mX[0]);

	for (uint i = 1; i < mX.size(); i++) {
		assert(mX[i] > mX[i - 1]);
		assert(mX[i] != mX[i - 1]);
		assert(mX[i] - mX[i - 1] > 0.f);
		assertIsNumber(mX[i]);
		assertIsNumber(mX[i] - mX[i - 1]);
		assertIsNumber(mX[i] + mX[i - 1]);
	}
}

void assertNoCoeffisNan(TimedData<ASpline>::CoeffVec const  & coeffs)
{
	for (uint i = 0; i < coeffs.size(); i++) {
		for (uint j = 0; i < 4; i++) {
			assertIsNumber(coeffs[i][j](0));
			assertIsNumber(coeffs[i][j](1));
			assert(std::abs(coeffs[i][j](0)) < 1E6);
			assert(std::abs(coeffs[i][j](1)) < 1E6);
		}
	}
}

void determineCoeffs(Plan::coeff_type & coeffs, math::flt dt, math::Vec3 const & pos0, math::Vec3 const & speed0, math::Vec3 const & pos1, math::Vec3 const & speed1)
{
	Plan::Dom const _dt2 = Plan::Dom(1.0) / (dt * dt);

	coeffs[3] = pos0;
	coeffs[2] = speed0;
	coeffs[1] = (Plan::Dom(3.f) * (pos0 - pos1) - dt * (speed0 + Plan::Dom(2.f) * speed1)) * _dt2;
	coeffs[0] = Plan::Dom(1.0f / 3.0f) * _dt2 * (speed0  - (Plan::Dom(2.f) * dt) * pos1 - speed1);
}

Plan::edge_track_type::const_iterator Plan::trackAtTime(Dom t) const
{
	edge_track_type::const_iterator it = mEdgeTrack.begin();

	while (it != mEdgeTrack.end() && t >= it->first) {
		it++;
	}

	if (it != mEdgeTrack.begin()) {
		it--;
	}

// 	assert(it != track().end() && t >= it->first);
	return it;
}


void Plan::correctEdgeTrack(Dom orgFromTime, Dom orgToTime, Dom newFromTime, Dom newToTime)
{
	assert(orgToTime > orgFromTime);
	Dom const
	orgDir = orgToTime - orgFromTime,
	newDir = newToTime - newFromTime;

	edge_track_type::reverse_iterator it = mEdgeTrack.rbegin();

	while (it != mEdgeTrack.rend() && orgToTime < it->first) {
		it++;
	}

	int num = 0;

	while (it != mEdgeTrack.rend() && orgFromTime <= it->first) {
		assert(orgFromTime <= it->first);
		assert(orgToTime >= it->first);
		//std::cerr << "CorrectEdge: " << num;
		//std::cerr << "CorrectEdge: " << num << std::endl;
		num++;
		Dom lambda = (it->first - orgFromTime) / orgDir;
		it->first = newFromTime + lambda * newDir;
		it++;
	}
}

// void Plan::correctActionTrack(Dom orgFromTime, Dom orgToTime, Dom newFromTime, Dom newToTime)
// {
// 	assert(orgToTime > orgFromTime);
// 	Dom const
// 		orgDir = orgToTime - orgFromTime,
// 		newDir = newToTime - newFromTime;
//
// 	action_track_type::reverse_iterator it = mActionTrack.rbegin();
// 	while (it != mActionTrack.rend() && orgToTime < it->first)
// 		it++;
//
// 	int num = 0;
// 	while (it != mEdgeTrack.rend() && orgFromTime <= it->first)
// 	{
// 		assert(orgFromTime <= it->first);
// 		assert(orgToTime >= it->first);
// 		//std::cerr << "CorrectEdge: " << num;
// 		//std::cerr << "CorrectEdge: " << num << std::endl;
// 		num++;
// 		Dom lambda = (it->first - orgFromTime) / orgDir;
// 		it->first = newFromTime + lambda * newDir;
// 		it++;
// 	}
// }

math::flt Plan::brakeDown(Dom const finalSpeed, Dom const maxAcc)
{
#if defined(AKIMAPLAN)
	Dom speed = finalSpeed;
	assertTimeContinuity(TimedData<ASpline>::abscissae());
	assertNoCoeffisNan(TimedData<ASpline>::coeffs());

// 	std::cerr << "Brakedown:" << std::endl;

	// First, determine for each time intervall, how long it should take
	// to get from one point to another in a straight line with an acceleration of maxAcc
	// - Is the current time lower, then we are definitly too fast and we have to lower the speed at
	// the point
	// - Do we take longer than the determined minimum time, we asume everything is correct and don't change the intervall
	std::vector<Dom> dt;

	unsigned long start;

	for (start = abscissae().size() - 1; start > 0; --start) {
		Img const & pos0 = coeffs()[start][3];
		Img const & pos1 = coeffs()[start - 1][3];

		Dom const dist = sqrtf(ssd(pos0, pos1));
		Dom const time = abscissae()[start] - abscissae()[start - 1];
		assertIsNumber(time);

		Dom const p_2 = speed / maxAcc; // p/2 out of the p-q-formula
		Dom const newTime = sqrt(p_2 * p_2 + Dom(2.0) * (dist / maxAcc)) - p_2;
		assertIsNumber(newTime);

		// Are we too fast in getting from start-1 to start
		if (newTime < time) {
			break;
		}

		// Yes, so store the time we should have used
		dt.push_back(newTime);
		// And calculate how fast we could go in the previous time intervall
		speed = speed + maxAcc * newTime;
	}

	// We don't have to change anything
	if (start == abscissae().size() - 1) {
		return finalSpeed;
	}

	// dt[i] now holds the minimum time, we want to cover the space between the points @ abscissae().size() - i-2 and i-1

	typedef boost::tuple<Dom, Dom, Dom, Dom> corr_intervall_type;
	std::vector<corr_intervall_type> correctionIntervalls;

	// Now, we change the time points, starting from the earliest point and accumulating the dt[i]s
	// dt.back() holds the value for mX[start],mX[start+1]

	unsigned long i = start + 1;
	Dom orgFromTime = mX[start];

	for (std::vector<Dom>::reverse_iterator rit = dt.rbegin(); rit != dt.rend(); ++rit, ++i) {
		Dom const timestep = *rit;
		Dom const orgToTime = mX[i];
		mX[i] = mX[i - 1] + timestep;

		correctionIntervalls.push_back(boost::make_tuple(orgFromTime, orgToTime, mX[i - 1], mX[i]));
		//correctEdgeTrack(orgFromTime, orgToTime, mX[i], mX[i+1]);

		orgFromTime = orgToTime;
	}

	// We also have to change the edge track
	// TODO Also the action track and forward track have to be adapted.
	std::vector<corr_intervall_type>::reverse_iterator corr_rit;

	for (corr_rit = correctionIntervalls.rbegin(); corr_rit != correctionIntervalls.rend(); ++corr_rit) {
		correctEdgeTrack(corr_rit->get<0>(), corr_rit->get<1>(), corr_rit->get<2>(), corr_rit->get<3>());
	}

	// Finally, we have to adapt the coefficents to fit the time intervalls
	retabulate(start, abscissae().size() - 1);

// 	currSpeed = interpolateY1( dt[0], *(coeffs().rbegin()+1)).norm();
// 	assert(fabsf(speed-currSpeed) < 0.5f );

	assertTimeContinuity(TimedData<ASpline>::abscissae());
	assertNoCoeffisNan(TimedData<ASpline>::coeffs());
	return finalSpeed;
#else
	Dom currSpeed = coeffs().back()[2].norm();

	if (currSpeed <= finalSpeed) {	// We are already slower than we have to be
		return currSpeed;
	}

// 	std::cout
// 		<< "Final = " << finalSpeed << " Current = " << currSpeed << std::endl;

	Dom targetSpeed = finalSpeed;

	assert(currSpeed > 0.0f);	// TODO Handle the case

	// The last one is special, as it holds a polynomial for the speed, but has no time-intervall
	mCoeff.back()[2] *= (targetSpeed / currSpeed);
	fixContinuity(coeffs().size() - 1);	// TODO: May be superfluous, check it

	// We are too fast, so we have to adept the speed and time accordingly
	Dom prevTargetSpeed = targetSpeed;
	std::vector<Dom> dt;
	unsigned long start;

	for (start = coeffs().size() - 1; start > 0; --start) {
		Img const & pos0 = coeffs()[start - 1][3];
		Img const & pos1 = coeffs()[start][3];

		Dom const dist = sqrt(math::ssd(pos0, pos1));
		Dom const time = abscissae()[start] - abscissae()[start - 1];

		Dom const p_2 = targetSpeed / maxAcc; // p/2 out of the p-q-formula
		Dom const newTime = sqrt(p_2 * p_2 + Dom(2.0) * (dist / maxAcc)) - p_2;

		targetSpeed = targetSpeed + maxAcc * newTime;	// How fast could we go?
		currSpeed = coeffs()[start - 1][2].norm();

// 		std::cout
// 			<< start << '/' << coeffs().size() << ": D = " << dist << " currSpeed=" << currSpeed << " targetSpeed=" << targetSpeed << std::endl
// 			<< "Time: " << newTime << '(' << time << ')' << std::endl;

		if (targetSpeed >= currSpeed) {
			Dom const avgTime = 2.0f * dist / (currSpeed +  prevTargetSpeed);
			Dom const newTime = std::max(avgTime, time);
			dt.push_back(newTime);
			break;
		}

		mCoeff[start - 1][2] *= (targetSpeed / currSpeed); // Scale the speed
		dt.push_back(newTime);
		prevTargetSpeed = targetSpeed;
	}

	// Now, we change the time points, starting from the earliest point and accumulating the dt[i]s
	// dt.back() holds the value for mX[start],mX[start+1]
	// TODO Also the action track has to be adapted.

	fixContinuity(start);
	unsigned long i = start;
	Dom orgFromTime = mX[i - 1];
	// Seems like a noop, but const_iterator -> iterator
	edge_track_type::iterator trackit = mEdgeTrack.begin() + (trackAtTime(orgFromTime) - mEdgeTrack.begin());
	isforward_type::iterator forwardit = mIsForward.begin() + (isForwardAtTime(orgFromTime) - mIsForward.begin());
	action_track_type::iterator actionit;
	{
		action_track_type::reverse_iterator it;

		for (it = mActionTrack.rbegin(); it != mActionTrack.rend() && orgFromTime < it->get<2>(); ++it) {}

		actionit = mActionTrack.begin() + (mActionTrack.rend() - it);
	}


	for (std::vector<Dom>::reverse_iterator rit = dt.rbegin(); rit != dt.rend(); ++rit, ++i) {
		Dom const newDt = *rit;
		Dom const orgToTime = mX[i];
		Dom const newFromTime = mX[i - 1];
		Dom const newToTime = newFromTime + newDt;
		// assert(newDt >= orgToTime  - orgFromTime);

		mX[i] = newToTime;
		fixContinuity(i);

		// Forward the iterators to orgToTime
		for (; trackit != mEdgeTrack.end() && orgToTime > trackit->first; ++trackit) {}

		for (; forwardit != mIsForward.end() && orgToTime > forwardit->first; ++forwardit) {}

		for (; actionit != mActionTrack.end() && orgToTime > actionit->get<2>(); ++actionit) {}

		// Change the times according to the support points
		if (trackit != mEdgeTrack.end() && orgToTime == trackit->first) {
			trackit->first = newToTime;
		}

		if (forwardit != mIsForward.end() && orgToTime == forwardit->first) {
			forwardit->first = newToTime;
		}

		if (actionit != mActionTrack.end() && orgToTime == actionit->get<1>()) {
// 			std::cout
// 				<< "Changing Action " << actionit->get<0>() << " begin: " << actionit->get<1>() << " -> " << newToTime << std::endl;
			actionit->get<1>() = newToTime;
		}

		if (actionit != mActionTrack.end() && orgToTime == actionit->get<2>()) {
// 			std::cout
// 				<< "Changing Action " << actionit->get<0>() << " end: " << actionit->get<2>() << " -> " << newToTime << std::endl;
			actionit->get<2>() = newToTime;
		}

// 		std::cout
// 			<< i << ": " << orgFromTime << ':' << orgToTime << " -> " << mX[i-1] << ':' << mX[i] << std::endl;

		orgFromTime = orgToTime;
	}

	{
		Dom const orgToTime = mX.back();
		Dom const orgDt = orgToTime - orgFromTime;
		Dom const newFromTime = *(mX.rbegin() + 1);
		Dom const newToTime = newFromTime + orgDt;
		mX.back() = newToTime;

		for (; trackit != mEdgeTrack.end() && orgToTime > trackit->first; ++trackit) {}

		if (trackit != mEdgeTrack.end() && orgToTime == trackit->first) {
			trackit->first = newToTime;
		}

		for (; forwardit != mIsForward.end() && orgToTime >  forwardit->first; ++forwardit) {}

		if (forwardit != mIsForward.end() && orgToTime == forwardit->first) {
			forwardit->first = newToTime;
		}
	}

	assertTimeContinuity(TimedData<ASpline>::abscissae());
	assertNoCoeffisNan(TimedData<ASpline>::coeffs());

	return TimedData<ASpline>::coeffs().back()[2].norm();
#endif
}


Plan::isforward_type::const_iterator Plan::isForwardAtTime(Dom t) const
{
	isforward_type::const_iterator
	i = std::lower_bound(isForward().begin() + 1, isForward().end(), std::make_pair(t, true) , util::OrderByFirst< std::pair<Dom, bool> >());
	--i;

	assert(i != isForward().end() && t >= i->first);

	return i;
}

/// findet zeitlich erste action die echt innerhalb des bereich bzw den bereich teilweise enthaellt
/// nur am rand beruehren reicht nicht aus
boost::optional<Plan::action_descr> Plan::findFirstAction(int type, Dom startTime, Dom endTime) const
{
	boost::optional<Plan::action_descr> action;

	action_track_type::const_reverse_iterator rit = mActionTrack.rbegin();

	while (rit != mActionTrack.rend() && startTime < rit->get<2>()) {
//		if ((rit->get<0>() == type || type == ANY) && endTime > rit->get<1>()) {
		bool typeCorrect = ((rit->get<0>() & type) != 0) || type == ANY;

		if (typeCorrect && endTime > rit->get<1>()) {
			action = boost::optional<Plan::action_descr>(*rit);
		}

		rit++;
	}

	return action;
}


boost::optional<Plan::action_descr> Plan::findFirstAction(int type1, int type2, Dom startTime, Dom endTime) const
{
	boost::optional<Plan::action_descr> action;

	action_track_type::const_reverse_iterator rit = mActionTrack.rbegin();

	while (rit != mActionTrack.rend() && startTime < rit->get<2>()) {
		bool typeCorrect = ((rit->get<0>() & type1) != 0) || ((rit->get<0>() & type2) != 0) || type1 == ANY || type2 == ANY;

		if (typeCorrect && endTime > rit->get<1>()) {
			action = boost::optional<Plan::action_descr>(*rit);
		}

		rit++;
	}

	return action;
}

void Plan::insert_action(action_type type, Dom startTime, Dom endTime, boost::any const & data)
{
	action_track_type::reverse_iterator rit = mActionTrack.rbegin();

	while (rit != mActionTrack.rend() && startTime < rit->get<1>()) {
		++rit;
	}

	if (mActionTrack.rbegin() == rit) {
		mActionTrack.push_back(boost::make_tuple(type, startTime, endTime, data));
	}
	else {
		--rit;
		mActionTrack.insert(mActionTrack.begin() + (mActionTrack.rend() - rit), boost::make_tuple(type, startTime, endTime, data));
	}

#if defined(DEBUG)

	if (mActionTrack.size() > 1) {
		for (action_track_type::const_iterator it = mActionTrack.begin(); it != boost::prior(mActionTrack.end()); ++it) {
			assert(it->get<1>() < boost::next(it)->get<1>());
		}
	}

#endif
}

Plan::Dom Plan::findPointAtDistance(Dom timePos, Dom distance, Dom & timeAtDistance, Img & posAtDistance, unsigned int max_iter) const
{
	// This can be done better, as we get to know the distances in several sub-intervalls in each recursion
	// we can make smarter guesses, should the found distance be too large. We don't have to recalculate it
	// for every new guess


	// At timePos, we need to cover a distance of "distance", negative values means back in time
	Dom tX	= timePos;
	Dom remX = distance;

	Dom closestT = timePos;
	Dom closestDistance = distance;

	uint maxiter = max_iter;

	// Newton-Iteration
	while (std::abs(remX) > Dom(0.01) && maxiter-- > 0) {
		Dom dsdt = this->firstDerivative(tX).norm();	// First derivative is always positive, as a distance is always positive
// 		DCERR("t=" << tX << " s=" << remX << " ds/dt=" << dsdt);

		// Get a better derivative
		while (dsdt <= std::numeric_limits<Dom>::epsilon()) {
			Dom tNew = tX + copysign(0.01, remX);
			tNew = std::min(std::max(tNew, mX.front()), mX.back());

			if (tX != tNew) {
				tX = tNew;
			}
			else {
				//assert((closestT-timePos)*distance>=0.f);
				std::cout << "assert((closestT-timePos)*distance>=0.f): SUPRESSEF" << std::endl;
				timeAtDistance = closestT;
				posAtDistance = (*this)(closestT);
				return closestDistance;
			}

			dsdt = this->firstDerivative(tX).norm();
		}

		Dom dT = remX / dsdt;	// Given the speed dstd (which is always positive), we'd need dT (time) to cover the the distance remX
// 		DCERR("dT=" << dT);
		Dom tNew = std::min(std::max(tX + dT, mX.front()), mX.back());		// Time we'd reach the point in distance remX
//		Dom dS = copysign(math::adaptive_simpsons_rule(*this, std::min(tX, tNew), std::max(tX, tNew), Dom(5e-3)), dT);	// Distance, we really have covered
		Dom dS = copysign(math::adaptive_simpsons_rule(*this, std::min(tX, tNew), std::max(tX, tNew), Dom(5e-3)), dT);	// Distance, we really have covered

// 		DCERR("dS=" << dS);

		tX = tNew;
		remX -= dS;

		if (std::abs(remX) < std::abs(closestDistance)) {
			closestT = tX;
			closestDistance = remX;
		}
	}

	//assert((closestT-timePos)*distance>=0.f);
	if ((closestT - timePos)*distance < 0.f) {
		std::cout << "assert((closestT-timePos)*distance>=0.f): SUPRESSEF" << std::endl;
	}

	timeAtDistance = closestT;
	posAtDistance = (*this)(closestT);

// 	DCERR("Remainder=" << remX);

	return closestDistance;
}


/**
 * @brief Plan::getPosFromParamOnTrajectory returns the closest XY position given to the specified Parameter
 *        This method was added to avoid° using the overloaded and unintuitive Constructor usage from Polyspline
 * @param paramOnTrajectory the parameter on the spline, for example generated by "findClosestPoint"
 * @return X and Y coordinate of Point on Trajectory
 */
const math::Vec3 Plan::getPosFromParamOnTrajectory(math::flt paramOnTrajectory) const
{
	// replaces the overloaded Operator from PolySpline
	//    Img operator()(Dom xx) const {
	//		uint index = findIndex(xx);
	//		Dom const dX = xx - mX[index];
	//		return interpolateY(dX, mCoeff[index]);
	//	}

	uint index = findIndex(paramOnTrajectory);
	Dom dX = paramOnTrajectory - mX[index];
	return interpolateY(dX, mCoeff[index]);
}

void Plan::removePast(Dom x)
{
	assertIsNumber(x);
	assertTimeContinuity(TimedData<ASpline>::abscissae());
	assertNoCoeffisNan(TimedData<ASpline>::coeffs());

	math::flt timetoCut = x - 0.01f;// - 3.f;

// 	flt timeAtDistance;
// 	Vec2 posAtDistance;
// 	flt res = findPointAtDistance(	x - 0.01f, -5.f, timeAtDistance, posAtDistance);
// 	timetoCut = timeAtDistance;

	boost::optional<action_descr>
	action = findFirstAction(ANY, timetoCut, timetoCut);

	if (action) {
		assert(action->get<1>() <= timetoCut);
		timetoCut = action->get<1>();
	}

	if (timetoCut < mX[0]) {
		return;
	}

	uint idx = findIndex(timetoCut);
	assert(idx < TimedData<ASpline>::coeffs().size() - 1);
//	uint nRemovableFromFront = std::min(idx, mX.size() - 4);
	uint nRemovableFromFront = uint(std::max<int>(0, int(idx) - 4));
	assert(nRemovableFromFront < TimedData<ASpline>::coeffs().size() - 3);

	assertTimeContinuity(TimedData<ASpline>::abscissae());
	assertNoCoeffisNan(TimedData<ASpline>::coeffs());

	TimedData<ASpline>::CoeffVec coeffsCpy = TimedData<ASpline>::coeffs();
	assertNoCoeffisNan(TimedData<ASpline>::coeffs());
	assertNoCoeffisNan(coeffsCpy);

	if (nRemovableFromFront == 0) {
		return;
	}

	TimedData<ASpline>::CoeffVec coeffsCpy1;
	coeffsCpy1.insert(coeffsCpy1.begin(), TimedData<ASpline>::coeffs().begin() + nRemovableFromFront, TimedData<ASpline>::coeffs().end());

	TimedData<ASpline>::CoeffVec coeffsCpy2 = TimedData<ASpline>::coeffs();
	coeffsCpy2.erase(coeffsCpy2.begin(), coeffsCpy2.begin() + nRemovableFromFront);

	drop_front(nRemovableFromFront);

	assertNoCoeffisNan(coeffsCpy);
	assertNoCoeffisNan(coeffsCpy1);
	assertNoCoeffisNan(coeffsCpy2);
	assertNoCoeffisNan(TimedData<ASpline>::coeffs());
	assertTimeContinuity(TimedData<ASpline>::abscissae());

	// remove past in edge track
	uint newEdgeTrackStartIdx = trackAtTime(mX[0]) - mEdgeTrack.begin();
	Plan::edge_track_type::iterator newEdgeTrackStart = mEdgeTrack.begin() + newEdgeTrackStartIdx;
	mEdgeTrack.erase(mEdgeTrack.begin(), newEdgeTrackStart);

	// remove past in action track
	action_track_type::iterator newFirstAction = mActionTrack.begin();

	while (newFirstAction != mActionTrack.end() && mX[0] > newFirstAction->get<1>()) {
		newFirstAction++;
	}

	mActionTrack.erase(mActionTrack.begin(), newFirstAction);

	assertTimeContinuity(TimedData<ASpline>::abscissae());
	assertNoCoeffisNan(TimedData<ASpline>::coeffs());
}

void Plan::drop_front(uint n)
{
	TimedData<ASpline>::drop_front(n);
}

void Plan::pop_back()
{
	if (ASpline::abscissae().size() > 4) {
		ASpline::pop_back();
	}

	if (ASpline::back().first < mEdgeTrack.back().first) {
		mEdgeTrack.pop_back();
	}

	if (mActionTrack.size()
			// && TimedData<ASpline>::back().first < mActionTrack.back().get<1>()
			&& TimedData<ASpline>::back().first <= mActionTrack.back().get<1>()
			&& TimedData<ASpline>::back().first < mActionTrack.back().get<2>()
	   ) {
		mActionTrack.pop_back();
	}

	if (mIsForward.size()
			&& TimedData<ASpline>::back().first < mIsForward.back().first) {
		mIsForward.pop_back();
	}
}

void Plan::translateDomain(Dom dx)
{
	assertTimeContinuity(TimedData<ASpline>::abscissae());
	assertNoCoeffisNan(TimedData<ASpline>::coeffs());

	TimedData<ASpline>::translateDomain(dx);
	assertTimeContinuity(TimedData<ASpline>::abscissae());
	assertNoCoeffisNan(TimedData<ASpline>::coeffs());


	for (edge_track_type::iterator it = mEdgeTrack.begin(); it != mEdgeTrack.end(); ++it) {
		it->first += dx;
	}

	for (action_track_type::iterator it = mActionTrack.begin(); it != mActionTrack.end(); ++it) {
		it->get<1>() += dx;
		it->get<2>() += dx;
	}

	mFixPlanTime += dx;
	assertTimeContinuity(TimedData<ASpline>::abscissae());
	assertNoCoeffisNan(TimedData<ASpline>::coeffs());
}

#if defined(AKIMAPLAN)

void Plan::dropFuture()
{
	TimedData<ASpline>::DomVec const & mX = TimedData<ASpline>::abscissae();

	Dom xx = getEndTimeOfFixedPlan();
	xx = math::rangeCut(domain(), std::max(xx, mPresentTime + 0.25f));

	uint idx = findIndex(xx);
	assert(idx < mX.size() - 1);

	if (mX[idx] < xx) {
		++idx;
	}

	while (idx < mX.size() - 1) {
		boost::optional<action_descr>
		action = findFirstAction(ANY, mX[idx], mX[idx]);

		if (!action) {
			break;
		}
		else {
			assert(action->get<2>() >= xx);
			xx = action->get<2>();
			idx = findIndex(xx);
			assert(idx < mX.size() - 1);

			if (mX[idx] < xx) {
				++idx;
			}
		}
	}

	while (idx < mX.size() - 1 && mX.size() > 4) {
		pop_back();
	}
}

void Plan::rollback(Dom x)
{
	bool checkActionTrackCorrupted = rollback_no_assert_action_track(x);
	assert(checkActionTrackCorrupted);
}

bool Plan::rollback_no_assert_action_track(Dom x)
{
	if (x < mPresentTime) {
		DCERR("mPresentTime: " << mPresentTime);
		DCERR("x: " << x);
	}

	assertIsNumber(x);
	assertIsNumber(mPresentTime);
	assert(x >= mPresentTime);

	assertTimeContinuity(TimedData<ASpline>::abscissae());
	assertNoCoeffisNan(TimedData<ASpline>::coeffs());

/////////////////////////////////
	{
		TimedData<ASpline>::DomVec const & mX = TimedData<ASpline>::abscissae();
		TimedData<ASpline>::CoeffVec const  & coeffs = TimedData<ASpline>::coeffs();
		uint index = findIndex(x);
		assert(index < mX.size() - 1);

		Dom const dX = x - *(&(*mX.begin()) + index);
		assertIsNumber(dX);
		assert(dX < 1E6);
		Img yy = interpolateY(dX, *(&(*coeffs.begin()) + index));
		assertIsNumber(yy(0));
		assertIsNumber(yy(1));
	}
/////////////////////////////////


	Img rollbackPos = (*this)(x);
	edge_track_type::const_iterator it = trackAtTime(x);

	assert(it != mEdgeTrack.end());
	assertIsNumber(rollbackPos(0));
	assertIsNumber(rollbackPos(1));

	bool popBackLastPos = TimedData<ASpline>::back().first >= x;
	//x = std::max( x, mPresentTime);

	while (popBackLastPos && TimedData<ASpline>::back().first >= x) {
		popBackLastPos = (TimedData<ASpline>::abscissae().size() > 4);

		if (popBackLastPos) {
			pop_back();
		}
	}

	if (popBackLastPos && std::abs(TimedData<ASpline>::back().first - x) > 0.000001) {
		push_back(x, rollbackPos);
		push_back_edge(it->second);
	}

	// did we roll back into the middle of an action
	//assert(mActionTrack.size()==0 || TimedData<ASpline>::back().first >= mActionTrack.back().get<2>());

	assertTimeContinuity(TimedData<ASpline>::abscissae());
	assertNoCoeffisNan(TimedData<ASpline>::coeffs());

	return (mActionTrack.size() == 0 || TimedData<ASpline>::back().first >= mActionTrack.back().get<2>());
//  return (mActionTrack.size()==0 || TimedData<ASpline>::back().first >= mActionTrack.back().get<2>());
}

#endif
}


}


}


}



