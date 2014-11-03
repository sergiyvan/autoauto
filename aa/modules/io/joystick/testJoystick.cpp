#include <util/AaMain.h>
#include <util/TaskContextFactory.h>
#include <rtt/Activity.hpp>
#include <ocl/TaskBrowser.hpp>

using namespace RTT;
using namespace Orocos;
using namespace util;

int AA_main(int argc, char ** argv)
{
	initLogger(argc, argv);
	std::auto_ptr<TaskContext> tc(createContext("Joystick", "Joystick"));

	Activity activity(0, 1.0, tc->engine());
	activity.start();

	TaskBrowser browser(tc.get());
	browser.loop();
	activity.stop();

	return 0;
}
