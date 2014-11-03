#pragma once

#include <util/RtTaskContext.h>
#include <core/TimedData.h>

#include <modules/models/egostate/EgoState.h>
#include <modules/models/carstate/AuxDevicesData.h>
#include <modules/models/carstate/CarState.h>

typedef TimedData< PlainDataHolder< ::math::flt > > TimedDouble;

struct _SDL_Joystick;
typedef struct _SDL_Joystick SDL_Joystick;

/*!
 * \struct JoystickData
 * \brief Representing joystick input information that is axis and buttons.
 */
struct JoystickData {
	typedef ::math::flt flt;
	JoystickData(JoystickData const & copy)
		: axis(copy.axis)
		, button(copy.button)
	{}

	JoystickData()
		: axis(0)
		, button(0)
	{}

	JoystickData const & operator= (JoystickData const & copy) {
		if (this == &copy) {
			return *this;
		}

		axis = copy.axis;
		button = copy.button;
		return *this;
	}

	std::vector<flt> axis;
	std::vector<bool> button;
};

/**
 * \class Joystick
 * \brief The Joystick module reads input commands from one selected joystick
 *
 */
class Joystick
	: public util::RtTaskContext
{
public:
	typedef ::math::flt flt;
	explicit Joystick(std::string const & name);
	virtual ~Joystick();

	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();
	virtual void errorHook();

protected:
	bool setup(int i);

	/** @name InputPorts: */
	//@{
	RTT::InputPort< TimedEgoState > mEgoStateIn;
	RTT::InputPort< ::modules::models::carstate::TimedCarState > mCarStateIn;
	//@}


	/** @name OutputPorts: */
	//@{
	RTT::OutputPort<JoystickData> mJoystickDataOut;

	RTT::OutputPort<bool> mActivationRequestOut;
	RTT::OutputPort<TimedDouble> mSpeedOut;
	RTT::OutputPort<flt> mSteerOut;
	RTT::OutputPort<int> mGearOut;
	RTT::OutputPort<AuxDevicesData> mAuxDevicesOut;
	//@}


	/** @name Attributes: */
	//@{
	RTT::Attribute<std::vector<const char *> > mJoysticks;
	//@}

	/** @name Properties: */
	//@{
	RTT::Property<std::string> mSelectedJoystick;
	RTT::Property<bool> mVerbose;
	//@}


private:
	int mSelectedJoystickNumber;
	SDL_Joystick * mpJoystick;
	JoystickData mData;

	/** methods */
	bool listJoysticks();
	bool selectJoystickByName(std::string);
	bool selectJoystickByIndex(int);
	bool printOutput();
};
