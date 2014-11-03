#include <rtt/os/main.h>
#include <rtt/Activity.hpp>

#include <util/TaskContextFactory.h>
#include <util/AaMain.h>

#include <util/ConcurrentTaskBrowser.h>
#include <qapplication.h>
#include <GL/glut.h>


using namespace RTT;
using namespace util;


int AA_main(int argc, char ** argv)
{
	initLogger(argc, argv);
	QApplication app(argc, argv);
	app.setQuitOnLastWindowClosed(true);
	glutInit(&argc, argv);
	//const flt minPeriod = 0.01f;
	/// create all modules:
	/// create root task
	TaskContext root("Root");
	ConcurrentTaskBrowser browser2(&root);
	int res = app.exec();
	return res;
}
