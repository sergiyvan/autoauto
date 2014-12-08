#include "SimObject.h"
#include <aa/modules/models/rndf/RndfGraph.h>
#include "SimulatorEngine.h"
#include <data/VehicleData.h>
#include <math/AutoMath.h>
#include <patterns/Singleton.h>

using namespace aa::modules::nav::simulator;
using namespace aa::modules::models::rndf;

typedef patterns::Singleton<SimulatorEngine> theSimulator;

using namespace ::math;

uint SimObject::idsequence = 0;

SimObject::SimObject(std::string const & m, flt x, flt y, Vec3 const & size)
	: position(x, y, 0)
	, direction(1, 0, 0)
	, orientation(0, 0, 0, 1)
	, material(0.5, 0.5, 0.5, 0.25)
	, size(size)
	, model(m)
	, modelscale(1, 1, 1)
	, name(m)
	, started(false)
	, option("") //("jump")
{
	id = idsequence;
	idsequence++;
}

SimObject::~SimObject()
{}

void SimObject::setPosition(Vec3 pos)
{
	position = pos;
}

void SimObject::setDirection(Vec3 dir)
{
	if (dir.squaredNorm() == 0) {
		return;
	}

	direction = normalized(dir);
	orientation = math::quaternionFromDirectionVector(direction);
}

std::pair<Mat4x4, std::string> SimObject::getDrawData()
{
	std::pair<Mat4x4, std::string> rueck(Mat4x4::Identity(), model);
	Mat3x3 R = orientation.toRotationMatrix().transpose();

	// scale by size
	for (uint i = 0; i < 3; ++i) {
		rueck.first(i, 0) = size(i) * R(i, 0);
		rueck.first(i, 1) = size(i) * R(i, 1);
		rueck.first(i, 2) = size(i) * R(i, 2);
	}

	// translate by position
	rueck.first(3, 0) = position(0);
	rueck.first(3, 1) = position(1);
	rueck.first(3, 2) = position(2);

	return rueck;
}

bool SimObject::collide(SimObject * target)
{
	return false;
}

SimWaypoint::SimWaypoint(Vec3 const & p, flt s, std::string const & o)
	: position(p)
	, speed(s)
	, oldPos(o)
{
}

void SimObject::addWaypoint(Vec3 const & pos, flt speed, std::string const & o)
{
	path.push_back(SimWaypoint(pos, speed, o));
}

Vec3 SimObject::getVelocity()
{
	if (path.empty()) {
		return Vec3(0, 0, 0);
	}

	return normalized(direction) * path.begin()->speed;
}

Vec3 SimObject::getAcceleration()
{
	return Vec3(0, 0, 0);
}

void SimObject::setToPathStartPos()
{
	this->path = jumpPath;
	this->started = false;
}

void SimObject::update(DummyController * sim)
{
	if (this->path.size() == 0 && (this->option.compare("jump") == 0)) {
		this->path = jumpPath;
		this->started = false;
	}
	else if (this->path.size() == 0) {
		return;
	}

	if (!started) {
		if (path.begin()->oldPos != "") {
            // We need a random point
            std::string neu = "";
            neu = sim->getNewRNDFPointFrom(neu);
            position = sim->rndfToCoord(neu);
            path.begin()->position = position;
			path.begin()->oldPos = neu;
		}
		else {
			position = path.begin()->position;
		}

		jumpPath = path;
	}

//	std::cout << "alte obstacle pos: " << path.begin()->oldPos
//			<< "pos: " << path.begin()->position
//			<< std::endl;

	started = true;

	Vec3 direction = path.begin()->position - position;
	flt dist = direction.norm();

	if (dist > 0) {
		direction = direction / dist;
		position += direction * path.begin()->speed * theSimulator::instance().getCurrentFrameTime();
	}

	if (dist < 0.2) {
		if (path.begin()->oldPos == "") {
			path.pop_front();
        }
        else {
            // We need a random point
			flt curSpeed = path.begin()->speed;
            // get new spline edge data
            std::pair<const EdgeData *, std::string> edgeData = sim->getNewRNDFEdge(path.begin()->oldPos);

            //remove the old one
            path.pop_front();

            if (edgeData.first == NULL) {
                return;
			}

			if (edgeData.first->laneSpline()) {
				flt sParam = edgeData.first->sourceParam, tParam = edgeData.first->targetParam;
				flt splineLength = tParam - sParam;
				LaneSpline const & spline = *edgeData.first->laneSpline();

				// create the new final point
				SimWaypoint targetWaypoint = SimWaypoint(math::zeroExtend(spline(tParam)), curSpeed, edgeData.second);

				//add all spline points to the path
				for (flt i = sParam; i < tParam; i += 2.0f) {
					path.push_back(SimWaypoint(math::zeroExtend(spline(i)) , curSpeed));
				}

				// add the final point to the path so the next spline gets appended
				path.push_back(targetWaypoint);
			}
			else {
				path.push_back(SimWaypoint(sim->rndfToCoord(edgeData.second), curSpeed, edgeData.second));
			}
		}
	}

	// FIXME: this if condition is only a workaround for a direction bug
	// the direction flips for one iteration after a new set of spline points had been added to the path
	if (dot_product(direction, this->direction) > -0.9) {
		setDirection(direction);
		setPosition(position);
	}
}

