#pragma once

#include <algorithm>
#include <vector>
#include <ext/algorithm>
#include <boost/any.hpp>
#include <core/TimedData.h>
#include <util/Templates.h>
#include <util/Ports.h>
#include <util/PooledObjectTemplate.h>

#include <aa/modules/models/rndf/RndfGraph.h>

#include <boost/serialization/utility.hpp>
#include <util/serialization/Mat.h>
#include <util/serialization/CvMat.h>
#include <util/serialization/Tuple.h>

namespace aa
{
namespace modules
{
namespace nav
{

namespace controller
{

// #define AKIMAPLAN

#if defined(AKIMAPLAN)
#include <math/AkimaSpline.h>
typedef math::AkimaSpline< ::math::Vec2, ::math::flt> ASpline;
#else
typedef math::PolySpline< ::math::Vec3, ::math::flt, 4> ASpline;
#endif

class Plan
	: public TimedData<ASpline>
{
public:
	friend class boost::serialization::access;

    typedef std::vector< std::pair<Dom, ::aa::modules::models::rndf::edge_descr> > edge_track_type;
	typedef std::vector< std::pair<Dom, bool> > isforward_type;
    typedef math::PolySpline< ::math::Vec3, ::math::flt, 4> polyspline_type;
	typedef polyspline_type::Coeff coeff_type;

	/// action_descr contains type, start, and end time of an action
	enum action_type {
		START_POINT				=   1,
		CHECK_POINT				=   2,
		GOAL					=   4,
		STOP_SIGN				=   8,
		DECISION_POINT			=  16,
		TRAFFIC_LIGHT			=  32,
		TURN					=  64,		//meta: int > 0 == right, int < 0 == left
		LANE_CHANGE				= 128,		//meta: int > 0 == right, int < 0 == left
		XING					= 256,
		ZONE					= 512,
		REVERSE					= 1024,
		U_TURN					= 2048,
		DEAD_END				= 4096,
		GIVE_WAY				= 8192,
        MRM_MINBRAKEDIST        = 16384,
        MRM_WANTEDRAKEDIST      = 32768,
        MRM_MAXAVLBLBRAKEDIST        = 65536,
		ANY						= -1
	};
// 			LANE_CHANGE				=   1,
// 			UTURN					=   2,
// 			UTURN_AT_BLOCKED_ROAD	=   4,
// 			XING					=   8,
// 			STOP_SIGN				=  16,
// 			CROSS_LANE				=  32,
// 			EMERGENCY_STOP			=  64,
// 			DEFAULTSTOP				= 128,
// 			ANY						= -1};

	static char const * const action_type_string(action_type action) {
		switch (action) {
		case START_POINT:
			return "";

		case CHECK_POINT:
			return "CHECK_POINT";

		case GOAL:
			return "GOAL";

		case STOP_SIGN:
			return "STOP_SIGN";

		case DECISION_POINT:
			return "DECISION_POINT";

		case TRAFFIC_LIGHT:
			return "TRAFFIC_LIGHT";

		case TURN:
			return "TURN";

		case LANE_CHANGE:
			return "LANE_CHANGE";

		case XING:
			return "XING";

		case ZONE:
			return "ZONE";

		case REVERSE:
			return "REVERSE";

		case U_TURN:
			return "U_TURN";

		case DEAD_END:
			return "DEAD_END";

		case GIVE_WAY:
			return "GIVE_WAY";

        case   MRM_MINBRAKEDIST:
            return "MIN_DIST";

        case   MRM_WANTEDRAKEDIST:
            return "MRM_TARGET";

         case  MRM_MAXAVLBLBRAKEDIST:
            return "MAX_DIST";

		default:
			return "-";
		}
	}

	typedef boost::tuple<action_type, Dom, Dom, boost::any > action_descr;
	typedef std::vector<action_descr> action_track_type;

	Plan()
		: TimedData<ASpline>()
		, mIsForward(2, std::make_pair(-std::numeric_limits<Dom>::infinity(), true))
		, mPresentTime(0)
		, mFixPlanTime(-std::numeric_limits<Dom>::infinity())
	{}

	Plan(TimeStamp const & stamp, ASpline const & spline)
		: TimedData<ASpline>(stamp, spline)
		, mIsForward(1, std::make_pair(-std::numeric_limits<Dom>::infinity(), true))
		, mPresentTime(0)
		, mFixPlanTime(-std::numeric_limits<Dom>::infinity())
	{}

	Plan(Plan const & cpy)
		: TimedData<ASpline>(cpy)
		, mEdgeTrack(cpy.mEdgeTrack)
		, mActionTrack(cpy.mActionTrack)
		, mIsForward(cpy.mIsForward)
		, mPresentTime(cpy.mPresentTime)
		, mFixPlanTime(cpy.mFixPlanTime) {
	}


	Plan & operator=(Plan const & rhs) {
		if (this == &rhs) {
			return *this;
		}

		TimedData<ASpline>::operator=(rhs);
		mEdgeTrack = rhs.mEdgeTrack;
		mActionTrack = rhs.mActionTrack;
		mIsForward = rhs.mIsForward;
		mPresentTime = rhs.mPresentTime;
		mFixPlanTime = rhs.mFixPlanTime;

		return *this;
	}

	void clear() {
		ASpline::clear();

		mEdgeTrack.clear();
		mActionTrack.clear();
		mIsForward.clear();

		mPresentTime = 0;
		mFixPlanTime = - std::numeric_limits<Dom>::infinity();
	}

	void translateDomain(Dom dx);

	// Returns the speed reached at the end (should be speed, but may differ)
	Dom brakeDown(Dom speed, Dom maxAcc);
	void removePast(Dom x);

	Dom findPointAtDistance(Dom timePos, Dom distance, Dom & timeAtDistance, Img & posAtDistance, unsigned int max_iter = 10) const;

    const math::Vec3 getPosFromParamOnTrajectory(math::flt paramOnTrajectory) const;


	edge_track_type const & track() const {
		return mEdgeTrack;
	}

	action_track_type const & actionTrack() const {
		return mActionTrack;
	}

	action_track_type & actionTrack() {
		return mActionTrack;
	}

	isforward_type const & isForward() const {
		return mIsForward;
	}

	edge_track_type::const_iterator trackAtTime(Dom time) const;

	isforward_type::const_iterator isForwardAtTime(Dom time) const;

	boost::optional<action_descr> findFirstAction(int type, Dom startTime, Dom endTime) const;

	boost::optional<action_descr> findFirstAction(int type1, int type2, Dom startTime, Dom endTime) const;

#if defined(AKIMAPLAN)
	void rollback(Dom x);

	void push_back(Dom time, Img const & pos) {
		assert(time > domain().second);
		assert(!isnan(time));
		assert(!isnan(pos(0)));
		assert(!isnan(pos(1)));

		TimedData<ASpline>::push_back(time, pos);
	}

	void dropFuture();

#else

	void push_back(Dom time, Img const & pos, Img const & speed) {
		assert(mPoints < 1 || time > abscissae()[mPoints - 1]);
		assert(!isnan(time));
		assert(!isnan(pos(0)));
		assert(!isnan(pos(1)));
		assert(!isnan(speed(0)));
		assert(!isnan(speed(1)));
		coeff_type coeffs;
        coeffs[0] = Img(0.0f, 0.0f, 0.0f);
        coeffs[1] = Img(0.0f, 0.0f, 0.0f);
		coeffs[2] = speed;
		coeffs[3] = pos;

		polyspline_type::push_back(time, time + 0.01f, coeffs);

		if (mPoints > 1) {
			fixContinuity(mPoints - 1);
		}
	}
#endif

	// Sets the edge of the last pushed element
    void push_back_edge(::aa::modules::models::rndf::edge_descr edge) {
		assert(mEdgeTrack.empty() || mEdgeTrack.back().first < abscissae().back());
		assert(abscissae().size() > 1);

		if (mEdgeTrack.begin() == mEdgeTrack.end() || mEdgeTrack.back().second != edge) {
			mEdgeTrack.push_back(std::make_pair(abscissae()[abscissae().size() - 2], edge));
		}
	}

	void insert_action(action_type type, Dom startTime, Dom endTime, boost::any const & data =  boost::any());

	void push_back_action(action_type type, Dom startTime, Dom endTime, boost::any const & data =  boost::any()) {
		assert(startTime < endTime);
		//assert(mActionTrack.size()==0 || mActionTrack.back().get<2>()<startTime);
		assert(mActionTrack.size() == 0 || mActionTrack.back().get<1>() < startTime);

		mActionTrack.push_back(boost::make_tuple(type, startTime, endTime, data));
	}

	void push_back_isForward(bool forward) {
		assert(mIsForward.empty() || mIsForward.back().first < abscissae().back());
		mIsForward.push_back(std::make_pair(abscissae().back(), forward));
	}

	void setPresentTime(Dom presentTime) {
		assert(!isnan(presentTime));
		//std::cerr << "setPresentTime: " << presentTime<< std::endl;
		assert(domain().first <= presentTime && presentTime <= domain().second);

		if (abscissae().empty())	{
			mPresentTime = presentTime;
		}
		else {
			mPresentTime = std::max(presentTime, abscissae()[std::min(size_t(3), abscissae().size() - 1)]);
		}

		assert(!isnan(mPresentTime));
	}

	Dom getPresentTime() const {
		return mPresentTime;
	}

	void fixPlanAtTime(Dom t) {
		mFixPlanTime = t;
	}

	Dom getEndTimeOfFixedPlan() const {
		boost::optional<action_descr>
		action = findFirstAction(ANY, mPresentTime, mPresentTime);

		if (action) {
			assert(action->get<2>() >= mPresentTime);
			return std::max(action->get<2>(), mFixPlanTime);
		}

		return std::max(mPresentTime, mFixPlanTime);
	}

	class const_iterator
	{
	public:
		const_iterator(Plan const & plan, Dom x)
			: mPlan(plan)
			, mIndex(plan.findIndex(x))
			, mCurrEdge(plan.trackAtTime(x))
			, mX(x) {
			mDX = mX - mPlan.mX[mIndex];
		}

		void operator+=(Dom dx) {
			if (dx >= 0) {
				forward(dx);
			}
			else {
				backward(dx);
			}
		}

		void operator-=(Dom dx) {
			(*this) += -dx;
		}

		Img operator()() const {
			return ASpline::interpolateY(mDX, mPlan.mCoeff[mIndex]);
		}

		Img firstDerivative() const {
			return ASpline::interpolateY1(mDX, mPlan.mCoeff[mIndex]);
		}

		Img secondDerivative() const {
			return ASpline::interpolateY2(mDX, mPlan.mCoeff[mIndex]);
		}

		void valueAndFirstDerivative(std::pair<Img, Img>& val) const {
			ASpline::interpolateY01(val, mDX, mPlan.mCoeff[mIndex]);
		}

		edge_track_type::const_iterator currEdge() const {
			return mCurrEdge;
		}

	protected:
		void forward(Dom dx) {
			mX += dx;
			assert(mX <= mPlan.mX.back());

			while (mX > mPlan.mX[mIndex + 1]) {
				mIndex++;
			}

			if (mCurrEdge != mPlan.track().end()) {
				edge_track_type::const_iterator next = boost::next(mCurrEdge);

				while (next != mPlan.track().end() && next->first < mX) {
					++next;
				}

				mCurrEdge = prior(next);
			}

			mDX = mX - mPlan.mX[mIndex];
		}

		void backward(Dom dx) {
			mX -= dx;

			while (mX < mPlan.mX[mIndex - 1]) {
				mIndex--;
			}

			mDX = mX - mPlan.mX[mIndex];
		}

	private:
		Plan const & mPlan;
		uint mIndex;
		edge_track_type::const_iterator mCurrEdge;
		Dom mX;
		Dom mDX;
	};

	const_iterator iterateFrom(Dom x) const {
		return const_iterator(*this, x);
	}

protected:
	// Corrects the higher coefficients
	void fixContinuity(unsigned long ix) {
		coeff_type const & coeff = mCoeff[ix];
		coeff_type & lastcoeff = mCoeff[ix - 1];
		Dom const dt = abscissae()[ix] - abscissae()[ix - 1];
		Dom const _dt2 = Dom(1) / (dt * dt);
		lastcoeff[1] = (Dom(3.0) * (coeff[3] - lastcoeff[3]) - dt * (coeff[2] + Dom(2.0) * lastcoeff[2])) * _dt2;
		lastcoeff[0] = Dom(1.0 / 3.0) * _dt2 * (coeff[2]  - (Dom(2.0) * dt) * lastcoeff[1] - lastcoeff[2]);
	}

private:
	void pop_back();
	void drop_front(uint n);

	bool rollback_no_assert_action_track(Dom x);
	void correctEdgeTrack(Dom orgFromTime, Dom orgToTime,
						  Dom newFromTime, Dom newToTime);

	edge_track_type mEdgeTrack;
	action_track_type mActionTrack;
	isforward_type mIsForward;

	Dom mPresentTime;
	Dom mFixPlanTime;


    // ATTENTION!! SERIALIZATION IS NOT FULLY IMPLEMENTED
    // boost::any and the edgetrack will not be serialized
    // and the information will be los
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
		ar & boost::serialization::base_object<TimedData<ASpline> >(*this);
//		ar & mEdgeTrack; This variable will not be serialized, because the Edge and Vertex Maps cause trouble
        ar & mActionTrack;
		ar & mIsForward;
	}
};

POOLEDOBJECT(Plan, 128);

}


}


}


}




namespace boost
{
    namespace serialization {

        // ATTENTION!! SERIALIZATION IS NOT FULLY IMPLEMENTED
        template<typename Archive>
        void serialize(Archive & ar,
                       boost::any & t,
                       const unsigned int file_version)
        {
//            try
//            {
//               ar & boost::any_cast<int>(t);
//            }
//            catch(const boost::bad_any_cast &)
//            {
//                std::cout << " Any Cast Error " << std::endl;
//            }

        }


    }
}






namespace RTT
{
extern template class InputPort<aa::modules::nav::controller::Plan>;
extern template class OutputPort<aa::modules::nav::controller::Plan>;

extern template class InputPort<aa::modules::nav::controller::Plan_ptr>;
extern template class OutputPort<aa::modules::nav::controller::Plan_ptr>;
}
