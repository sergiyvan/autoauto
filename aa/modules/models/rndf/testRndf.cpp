#include <aaconfig.h>

#include <util/TaskContextFactory.h>
#include <util/AaMain.h>
#include <util/OrocosHelperFunctions.h>
#include <util/ConcurrentTaskBrowser.h>

#include <gui/View3D.h>

#include <rtt/os/main.h>
#include <rtt/Activity.hpp>

#include <qapplication.h>
#include <boost/type_traits.hpp>
#include <rtt/internal/DataSourceGenerator.hpp>
#include <GL/glut.h>
#include <osgDB/FileUtils>

using namespace RTT;
using namespace util;

int AA_main(int argc, char ** argv)
{
	osgDB::getDataFilePathList().push_back(cmake_resource_dir);

	initLogger(argc, argv);

	QApplication app(argc, argv);
	app.setQuitOnLastWindowClosed(true);
	glutInit(&argc, argv);

	/// create view
	gui::View3D * view3D_0 = new gui::View3D();
	view3D_0->show();

	const float minPeriod = 0.02f;

	/// create all modules: DisplaySatelliteData
	std::auto_ptr<TaskContext>
	displaySplineKDTree(createContext("DisplaySplineKDTree", "DisplaySplineKDTree")),
						displayRNDF(createContext("DisplayRNDF", "DisplayRNDF")),
						displayCrossing(createContext("DisplayCrossing", "DisplayCrossing"));

	assert(displayRNDF.get());
	assert(displayCrossing.get());

	/// register views
	if (!callMethod(*displayRNDF, "registerView", internal::GenerateDataSource()(0))) {
		return 1;
	}

	if (!callMethod(*displayCrossing, "registerView", internal::GenerateDataSource()(0))) {
		return 1;
	}

	/// preregister views
	if (!callMethod(*displaySplineKDTree, "preRegisterView", internal::GenerateDataSource()(12))) {
		return 1;
	}

	std::string filename("swri/SwRI_RNDF_Rev_1.7.txt");

	if (argc > 1) {
		filename = argv[1];
	}

	std::string const & ff = filename;

	if (!callMethod(*displayRNDF, "loadRNDF", internal::GenerateDataSource()(ff))) {
		log(Error) << "Could not load RNDF:" << ff << endlog();
		return 1;
	}

	Activity periodicActivity4displaySplineKDTree(0, minPeriod, displaySplineKDTree->engine()),
			 periodicActivity4displayRNDF(0, minPeriod, displayRNDF->engine()),
			 periodicActivity4displayCrossing(0, minPeriod, displayCrossing->engine());

	/// connect ports

	/// start activities

	/// create root task

	ConcurrentTaskBrowser browser2(displayRNDF.get());
	int res = app.exec();
	return res;
}
