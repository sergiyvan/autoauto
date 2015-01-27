#pragma once

#include <util/RtTaskContext.h>
#include <data/Image.h>


namespace aa
{
namespace modules
{
namespace vision
{

class ImageMatching
	: public util::RtTaskContext
{
public:

	// Constructors:
    explicit ImageMatching(std::string const & name);

	// Destructor:
    virtual ~ImageMatching();

protected:
	/** \name Ports: */
	/*! \{ */
    RTT::OutputPort< ::data::TimedImagePtr> mImage1Out;
    RTT::OutputPort< ::data::TimedImagePtr> mImage2Out;
    RTT::OutputPort< ::data::TimedImagePtr> mMatchedImageOut;
    /*! \} */
	/** \name Properties: */
	/*! \{ */

	/*! \} */


	// Functions:
	virtual bool startHook();

	virtual void updateHook();

	virtual void stopHook();


private:

    ::data::TimedImagePtr mImage1;
    ::data::TimedImagePtr mImage2;
    ::data::TimedImagePtr mMatchedImage;


};

}
}
}
 // namespace
