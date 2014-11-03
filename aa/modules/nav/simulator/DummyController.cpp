#include "DummyController.h"

#include <iostream>
#include <set>
#include <algorithm>
#include <rtt/Logger.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include <boost/noncopyable.hpp>
#include <boost/foreach.hpp>
#include <util/TaskContextFactory.h>
#include "SimObject.h"
#include "SimulatorEngine.h"
#include <aa/modules/models/rndf/io/RndfReader.h>
#include <aa/modules/models/rndf/RndfGraph.h>
#include <data/VehicleData.h>
#include <patterns/Singleton.h>
#include <osgDB/FileUtils>

// Qt at the bottom due to signal/slot macros
#include <qdom.h>
#include <qfile.h>
#include <QStringList>
#define VERBOSE

#if defined(VERBOSE)
#define DLOG(X) RTT::log(RTT::Debug) << X << RTT::endlog()
#else
#define DLOG(X) /**/
#endif

using namespace aa::modules::nav::simulator;
using namespace ::math;
using namespace RTT;
using namespace boost;
using namespace std;
using namespace aa::modules::models::rndf;
using RTT::Logger;

REGISTERTASKCONTEXT(DummyController);


typedef patterns::Singleton<RNDFGraph> theRNDFGraph;
typedef patterns::Singleton<SimulatorEngine> theSimulator;


class DummyController::impl
	: boost::noncopyable
{
public:
	impl()
		: lastTimeStamp(0)
		, randomSeed(0)
		, randomSeed2(0)
		, rndfGraph(theRNDFGraph::instance())
	{}

	~impl() {
		clearObjects();
	}

	void clearObjects() {
		for (std::vector<SimObject *>::const_iterator it = objects.begin(); it != objects.end(); ++it) {
			delete *it;
		}

		objects.clear();
	}

	vertex_descr getRandomVertex() {
		uint rand = rand_r(&randomSeed2) % num_vertices(theRNDFGraph::instance().getBoostGraph());
		//DLOG("getting random vertex number " << rand);
		uint counter = 0;
		vertex_iter vi, vi_end;

		for (tie(vi, vi_end) = vertices(theRNDFGraph::instance().getBoostGraph()); vi != vi_end; ++vi) {
			if (rand == counter) {
				return *vi;
			}

			counter++;
		}

		return vertex_descr();
	}

	long long lastTimeStamp;
	uint randomSeed;
	uint randomSeed2;

	std::vector<SimObject *> objects;

	//RNDF stuff
	RNDFGraph & rndfGraph;

};

DummyController::DummyController(std::string const & name)
	: util::RtTaskContext(name)
	, pimpl(new impl())
{
	addOperation("loadSimulation", &DummyController::loadSimulation, this, RTT::ClientThread).doc("load a simulation xml file").arg("fileName", "fileName");
	addOperation("resetMovableObstacles", &DummyController::resetMovableObstacles, this, RTT::ClientThread).doc("reset obstacles to their start position");
}

DummyController::~DummyController()
{
}

bool DummyController::startHook()
{
	for (std::vector<SimObject *>::iterator it = pimpl->objects.begin(); it != pimpl->objects.end(); ++it) {
		(*it)->id = theSimulator::instance().registerObject(((*it)->path.size() == 0), (*it)->position, (*it)->size, (*it)->orientation, (*it)->model, (*it)->modelscale, (*it)->material, (*it)->name);
	}

	return true;
}

void DummyController::updateHook()
{
	Logger::In in("DummyController");

	for (std::vector<SimObject *>::iterator it = pimpl->objects.begin(); it != pimpl->objects.end(); ++it) {
		SimObject * simObj = *it;
		simObj->position = theSimulator::instance().getPosition(simObj->id);
		simObj->update(this);
		theSimulator::instance().setPosition(simObj->id, simObj->position);
		theSimulator::instance().setOrientation(simObj->id, simObj->orientation);
		theSimulator::instance().setVelocity(simObj->id, simObj->getVelocity());
		theSimulator::instance().setAcceleration(simObj->id, simObj->getAcceleration());
	}
}

void DummyController::stopHook()
{
}


void DummyController::resetMovableObstacles()
{

	for (std::vector<SimObject *>::iterator it = pimpl->objects.begin(); it != pimpl->objects.end(); ++it) {
		(*it)->setToPathStartPos();
	}
}

std::string DummyController::getNewRNDFPointFrom(std::string const & waypoint)
{
	aGraph const & graph = theRNDFGraph::instance().getBoostGraph();

	if (num_vertices(graph) <= 0) {
		return "error no vertices in graph";
	}

	property_map<aGraph, vertex_data_t>::const_type vertexDataMap = get(vertex_data_t(), graph);

	vertex_descr vdo = theRNDFGraph::instance().getVertex(waypoint);

// 	if (boost::graph_traits<aGraph>::null_vertex() != vdo) {
	do {
		vdo = pimpl->getRandomVertex();
	}
	while (vertexDataMap[vdo].vertexType.isSet(VertexData::INSERTED) || vertexDataMap[vdo].vertexType.isSet(VertexData::MIRROR_VERTEX));

//  		std::cout << vertexDataMap[vdo].name << std::endl;

	return vertexDataMap[vdo].name;
// 	}

// 	if (out_degree(vdo,graph) == 0) return "I am stuck, no way out";
//
//
// 	adjacency_iter vi, vi_end;
// 	uint rand = rand_r(&(pimpl->randomSeed)) % out_degree(vdo, graph);
// 	uint counter=0;
//
// 	for (tie(vi, vi_end) = adjacent_vertices(vdo, graph); vi != vi_end; vi++) {
// 		if (rand==counter) {
// 			return vertexDataMap[*vi].name;
// 		}
// 		counter++;
// 	}
//
// 	return "can not happen";
}

Vec3 DummyController::rndfToCoord(std::string const & waypoint)
{
	vertex_descr vd = theRNDFGraph::instance().getVertex(waypoint);

	if (boost::graph_traits<aGraph>::null_vertex() != vd) {
		aGraph const & graph = theRNDFGraph::instance().getBoostGraph();
		Vec2 const pos = get(vertex_data_t(), graph)[vd].pos;
		return Vec3(pos(0), pos(1), 0.0f);
	}
	else {
		Logger::log() << Logger::Error << "No Waypoint with that name:" << waypoint << Logger::endl;
		return Vec3(0.0f, 0.0f, 0.0f);
	}
}

std::pair<const EdgeData *, std::string> DummyController::getNewRNDFEdge(std::string const & waypoint)
{
	std::pair<EdgeData *, std::string> rueck;
	rueck.first = NULL;
	rueck.second = "";
	aGraph const & graph = theRNDFGraph::instance().getBoostGraph();

	vertex_descr vdo = theRNDFGraph::instance().getVertex(waypoint);

	if (boost::graph_traits<aGraph>::null_vertex() == vdo) {
		vdo = pimpl->getRandomVertex();
	}

	if (out_degree(vdo, graph) == 0) {
		return rueck;
	}

	property_map<aGraph, edge_data_t>::const_type
	edgeDataMap = get(edge_data_t(), graph);
	property_map<aGraph, vertex_data_t>::const_type
	vertexDataMap = get(vertex_data_t(), graph);

	out_edge_iter ei, ei_end;
	uint rand = rand_r(&(pimpl->randomSeed)) % out_degree(vdo, graph);
	uint counter = 0;

	for (tie(ei, ei_end) = out_edges(vdo, graph); ei != ei_end; ei++) {
		if (rand == counter) {
			return make_pair(&edgeDataMap[*ei], vertexDataMap[target(*ei, graph)].name);
		}

		counter++;
	}

	return rueck;
}


Vec3 parseVector(QString in)
{
	QStringList listLeer = in.split(" ");
	QStringList list = in.split(",");

	if (listLeer.size() > list.size()) {
		list = listLeer;
	}

	if (list.size() == 1) {
		return Vec3(list[0].toFloat(), 0.0f, 0.0f);
	}
	else if (list.size() == 2) {
		return Vec3(list[0].toFloat(), list[1].toFloat(), 0.0f);
	}
	else if (list.size() == 3) {
		return Vec3(list[0].toFloat(), list[1].toFloat(), list[2].toFloat());
	}
	else {
		Logger::log() << Logger::Debug << "could not parse " << in.toStdString() << Logger::endl;
		return Vec3(0.0f, 0.0f, 0.0f);
	}
}

Vec4 parseMaterial(QString in)
{
	QStringList listLeer = in.split(" ");
	QStringList list = in.split(",");

	if (listLeer.size() > list.size()) {
		list = listLeer;
	}

	if (list.size() == 4) {
		return Vec4(list[0].toFloat(), list[1].toFloat(), list[2].toFloat(), list[3].toFloat());
	}
	else {
		Logger::log() << Logger::Debug << "could not parse " << in.toStdString() << Logger::endl;
		return Vec4(0.0f, 0.0f, 0.0f, 0.0f);
	}
}

Quaternion parseQuaternion(QString in)
{
	QStringList listLeer = in.split(" ");
	QStringList list = in.split(",");

	if (listLeer.size() > list.size()) {
		list = listLeer;
	}

	if (list.size() == 4) {
		return Quaternion(list[0].toFloat(), list[1].toFloat(), list[2].toFloat(), list[3].toFloat());
	}
	else if (list.size() == 1) {
		double const angle = list[0].toFloat();
#if defined(USE_EIGEN)
		return Quaternion(Quaternion::AngleAxisType(angle, Vec3::UnitZ()));
#else
		return Quaternion(Vec3(0, 0, 1), angle);
#endif
	}
	else {
		Logger::log() << Logger::Debug << "could not parse " << in.toStdString() << Logger::endl;
		return Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
	}
}

bool DummyController::loadSimulation(std::string const & fileName)
{
	Logger::In in("DummyController");
	QDomDocument doc;

	QFile file(QString(osgDB::findDataFile(fileName).c_str()));

	if (!file.open(QIODevice::ReadOnly)) {
		Logger::log() << Logger::Debug << "could not open file " << fileName << Logger::endl;
		return false;
	}

	if (!doc.setContent(&file))	{
		Logger::log() << Logger::Debug << "could not parse file " << fileName << Logger::endl;
		file.close();
		return false;
	}

	pimpl->clearObjects();
	file.close();
	aGraph const & graph = theRNDFGraph::instance().getBoostGraph();
	QDomNodeList list = doc.elementsByTagName("rndf");
	/*	MARK

		if (list.length()==1)
		{
			std::string fname = list.item(0).firstChild().nodeValue().toStdString();
			RNDFReader reader;
			std::auto_ptr<RNDFData> rndfData;
			rndfData = std::auto_ptr<RNDFData>(reader.loadRNDF(fname.c_str()));
			if (!rndfData.get())
			{
				Logger::log() << Logger::Error << "failed loading RNDF" << Logger::endl;
			}
			else
			{
				RNDFGraph & rndfGraph = theRNDFGraph::instance();
				rndfGraph.buildGraph(*rndfData);
				Logger::log() << Logger::Debug << "Simulator - parsed RNDF:" << fname << Logger::endl;
			}
		}
	*/

	mShortestPath.calcAPSP();
	list = doc.elementsByTagName("simobject");
	Logger::log() << Logger::Debug << "reading xml file, got " << list.length() << " simobject nodes " << Logger::endl;

	for (unsigned int i = 0; i < list.length(); i++) {
		QDomNode node = list.item(i);
		QDomNodeList subList = node.childNodes();
		uint instCount = node.attributes().namedItem("instances").nodeValue().toInt();

		if (instCount == 0) {
			instCount = 1;
		}

		for (uint inst = 0; inst < instCount; inst++) {
			// For each simobject we create one
			SimObject * simobj = new SimObject();

			for (uint j = 0; j < subList.length(); j++) {
				if (QDomNode::ElementNode ==  subList.item(j).nodeType()) {
					// if we get a path object we need to parse all its position nodes
					if (subList.item(j).nodeName() == "path") {
						string option = subList.item(j).attributes().namedItem("option").nodeValue().toStdString();

// 					DLOG("Die Option hat folgenden Inhalt: " << option);
						if (option.compare("jump") == 0) {
							option = "jump";
							simobj->option = option;
							DLOG("option was successfully set to jump ");
						}
						else {
							DLOG("ERROR: option was not successfully set (not used or spelled wrong)");
							simobj->option = "";
						}

						QDomNodeList pathList = subList.item(j).childNodes();
						property_map<aGraph, vertex_data_t>::const_type vertexDataMap = get(vertex_data_t(), graph);
						std::string lastCheckpoint;
						std::string curCheckpoint;
						vertex_descr lastVec = theRNDFGraph::instance().getVertex("1.1.1"); // just a default value

						for (uint p = 0; p < pathList.length(); p++) {

							if (QDomNode::ElementNode ==  pathList.item(p).nodeType()) {
								if (pathList.item(p).nodeName() == "waypoint") {
									flt spd = pathList.item(p).attributes().namedItem("speed").nodeValue().toFloat();

									Vec3 coords;
									coords = parseVector(pathList.item(p).firstChild().nodeValue());

									if (coords.norm() == 0.0) {
#if 1
										std::string posRNDFWaypoint = pathList.item(p).firstChild().nodeValue().toStdString();

										vertex_descr vec = theRNDFGraph::instance().getVertex(posRNDFWaypoint);

										if (boost::graph_traits<aGraph>::null_vertex() != vec) {
											property_map<aGraph, vertex_data_t>::const_type
											vertexDataMap = get(vertex_data_t(), graph);
											Vec2 const & pos = vertexDataMap[vec].pos;
											coords = ::math::zeroExtend(pos);

//											std::cout << "NEW POS: " << coords << std::endl;
										}
										else {
											Logger::log() << Logger::Error << "reading <waypoint> : Could not find waypoint: " <<  posRNDFWaypoint << Logger::endl;
										}

#endif
									}
									else {
//                                    	std::cout << "global coords were set: " << coords << std::endl;
									}

									simobj->addWaypoint(coords, spd);
								}

								else if (pathList.item(p).nodeName() == "checkpoint") {

									flt spd = pathList.item(p).attributes().namedItem("speed").nodeValue().toFloat();
									curCheckpoint = pathList.item(p).firstChild().nodeValue().toStdString();
									// get the Waypoint position of a checkpoint id
									vector <vertex_descr> theCheckpoints = theRNDFGraph::instance().checkpoints();
									int wp;
									std::string wpStr;

									for (size_t i = 0; i < theCheckpoints.size(); i++) {
										VertexData const & vBla = vertexDataMap[theCheckpoints.at(i)];
										wp = atoi(curCheckpoint.c_str());

										if (wp == vBla.checkpoint) {
											ostringstream tempStr;
											tempStr << vBla.l0 << "." << vBla.l1 << "." << vBla.id;
											wpStr = tempStr.str();
											break;
										}
									}

									// get connections between two (p>0) inputs (e.g. checkpoints)
									vertex_descr vec = theRNDFGraph::instance().getVertex(wpStr);
									DLOG("######################################");

									if (p > 0) {
										std::pair<edge_descr, bool> theEdge = edge(lastVec, vec, graph);
										edge_descr const & theEdge_descr = theEdge.first;
										bool theEdgeBool = theEdge.second;
										bool edgeValid = true;

										// if there's nothing between two checkpoints (e.g. a crossing) then just drive on spline
										if (theEdgeBool) {
											DLOG("1. step: there's an direct edge between " << lastCheckpoint << " and " << curCheckpoint);
											DLOG("current position" << vertexDataMap[lastVec].l0 << "." << vertexDataMap[lastVec].l1 << "." << vertexDataMap[lastVec].id);
											DLOG("and goal is " << vertexDataMap[vec].l0 << "." << vertexDataMap[vec].l1 << "." << vertexDataMap[vec].id);
											// changing for the next one
											lastVec = vec;
											lastCheckpoint = curCheckpoint;
											std::vector<Vec3> drawEdge = addInterpolatedEdge(theEdge_descr);

											for (uint i = 0; i < drawEdge.size(); i++) {
												simobj->addWaypoint(drawEdge.at(i), spd, "");
											}
										}
										else {
											// if there's no direct connection between the 2 checkpoints, look for the shortest path
											// and find a way over crossings etc.
											bool reloop = true;

											while (reloop) {
												DLOG("-------------------------------------------");
												vertex_descr vLastVec = lastVec; // just to initialise
												edge_descr minE; // just to initialise, not for usage
												DLOG("1. step: there's no direct edge, looking for outgoing edges from current position");
												DLOG("current position" << vertexDataMap[lastVec].l0 << "." << vertexDataMap[lastVec].l1 << "." << vertexDataMap[lastVec].id);
												DLOG("and goal is " << vertexDataMap[vec].l0 << "." << vertexDataMap[vec].l1 << "." << vertexDataMap[vec].id);
												BOOST_FOREACH(edge_descr e, out_edges(lastVec, graph)) {
													std::vector<Vec3> drawEdge = addInterpolatedEdge(e);

													for (uint i = 0; i < drawEdge.size(); i++) {
														simobj->addWaypoint(drawEdge.at(i), spd, "");
													}

													vertex_descr found = target(e, graph);
													VertexData const & vFound = vertexDataMap[found];
													VertexData const & vVec = vertexDataMap[lastVec];
													DLOG("step 2: current position (WP) " << vVec.l0 << "." << vVec.l1 << "." << vVec.id
														 << " name: " << vVec.name);
													DLOG("finding following out_edges (WP) " << vFound.l0 << "." << vFound.l1 << "."
														 << vFound.id << " vF: " << vFound.name);
													flt minDist = numeric_limits<flt>::infinity();
													vertex_descr minOutFound = lastVec; //just to initialise, not for use
													VertexData  vMinOutFound;
													edge_descr theDrawEdge = e;

													if (vFound.checkpoint == vertexDataMap[vec].checkpoint) {
														reloop = false;
														continue;
													}

													// out position is now at an exit (found) and we're looking for the shortestpath of the outgoing edges
													// and we're taking the entry of the edge with the shortest path
													BOOST_FOREACH(edge_descr eOut, out_edges(found, graph)) {
														vertex_descr outFound = target(eOut, graph);
														VertexData const & vOutFound = vertexDataMap[outFound];
														DLOG("step 3: current position is exit (WP):" << vFound.l0 << "." << vFound.l1 << "." << vFound.id);
														DLOG("finding following out_edges: (e.g. lanes on a xing)" << vOutFound.l0 << "." << vOutFound.l1 << "." << vOutFound.id);
														DLOG("with distance" << mShortestPath.getLengthOfPath(outFound, vec));

														// find the shortest one
														if (mShortestPath.getLengthOfPath(outFound, vec) < minDist) {
															theDrawEdge = eOut;
															minOutFound = outFound;
															vMinOutFound = vOutFound;
															minDist = mShortestPath.getLengthOfPath(outFound, vec);
														}
													}
													DLOG("step 4: taking entry point with shortest path: " << vMinOutFound.l0 << "." << vMinOutFound.l1 << "." << vMinOutFound.id);
													std::vector<Vec3> drawEdge2 = addInterpolatedEdge(theDrawEdge);

													for (uint i = 0; i < drawEdge2.size(); i++) {
														simobj->addWaypoint(drawEdge2.at(i), spd, "");
													}

													//if the found entry point is not our expected goal (vec), than go on ... looking for the shortest path
// 													DLOG("step 5: current position (WP): " << vertexDataMap[source(theDrawEdge,graph)].l0  << "." << vertexDataMap[source(theDrawEdge,graph)].l1 << "." << vertexDataMap[source(theDrawEdge,graph)].id);
													DLOG("step 5: current position (WP): " << vertexDataMap[minOutFound].l0  << "." << vertexDataMap[minOutFound].l1 << "." << vertexDataMap[minOutFound].id);
													DLOG("our goal (CP) was the WP: " << vertexDataMap[vec].l0 << "." << vertexDataMap[vec].l1 << "." << vertexDataMap[vec].id);
													flt minDistE = numeric_limits<flt>::infinity();

// 													if(vertexDataMap[source(theDrawEdge,graph)].checkpoint != vertexDataMap[vec].checkpoint) {
													if (vertexDataMap[minOutFound].checkpoint != vertexDataMap[vec].checkpoint) {
														DLOG("step 6: Checkpoint not reached yet - looking for outgoing edges and shortest path to goal (CP)");
														BOOST_FOREACH(edge_descr theE, out_edges(minOutFound, graph)) {
															vertex_descr minVoutFound = target(theE, graph);
															DLOG("looking for the edge to CP: " << vertexDataMap[vec].l0 << "." << vertexDataMap[vec].l1 << "." << vertexDataMap[vec].id << ") finden.");
															DLOG("our current position: " << vertexDataMap[minOutFound].l0 << "." << vertexDataMap[minOutFound].l1 << "." << vertexDataMap[minOutFound].id);
															DLOG("finding following outgoing edge(s) " << vertexDataMap[target(theE, graph)].l0 << "." << vertexDataMap[target(theE, graph)].l1 << "." << vertexDataMap[target(theE, graph)].id);

															if (mShortestPath.getLengthOfPath(minVoutFound, vec) < minDistE) {
																DLOG("current shortest path length: " << minDistE);
																minDistE = mShortestPath.getLengthOfPath(minVoutFound, vec);
																DLOG("updated current shortest path length: " << minDistE);
																minE = theE;
															}
														}

														if (vertexDataMap[target(minE, graph)].checkpoint == vertexDataMap[vec].checkpoint) {
															reloop = false;
															lastVec = vec;
														}
														else {
															DLOG("step 7: goal not reached, starting again looking for exit, entry...");
															lastVec = target(minE, graph);
														}

														// calculating the way from entry to checkpoint or going on with next exit, reloop
														std::vector<Vec3> drawEdge3 = addInterpolatedEdge(minE);

														for (uint i = 0; i < drawEdge3.size(); i++) {
															simobj->addWaypoint(drawEdge3.at(i), spd, "");
														}
													}
													else {
														reloop = false;
														lastVec = vec;
														DLOG("step 6: checkpoint reached, finished");
													}
												}
												DLOG("Goal was (WP): " << vertexDataMap[vec].l0 << "." << vertexDataMap[vec].l1 << "." << vertexDataMap[vec].id);
												DLOG("current position (WP): " << vertexDataMap[target(minE, graph)].l0  << "." << vertexDataMap[target(minE, graph)].l1 << "." << vertexDataMap[target(minE, graph)].id);
											}
										}
									}

									lastCheckpoint = curCheckpoint;
									lastVec = vec;
								}

								else if (pathList.item(p).nodeName() == "rndfpoint") {
									flt spd = pathList.item(p).attributes().namedItem("speed").nodeValue().toFloat();
									std::string lastRNDFWaypoint = pathList.item(p).firstChild().nodeValue().toStdString();
									vertex_descr vec = theRNDFGraph::instance().getVertex(lastRNDFWaypoint);

									if (p > 0) {
										if (boost::graph_traits<aGraph>::null_vertex() != vec) {
											property_map<aGraph, vertex_data_t>::const_type vertexDataMap = get(vertex_data_t(), graph);
											Vec2 const & pos = vertexDataMap[vec].pos;
											simobj->addWaypoint(Vec3(pos(0), pos(1), 0.0f), spd, "");
											DLOG("adding waypoint for rndfpoint '" << lastRNDFWaypoint << "' at position " << pos(0) << ", " << pos(1) << ".");
											//std::cout << "getting rndfpoint: " << lastRNDFWaypoint << std::endl;
										}
										else {
											Logger::log() << Logger::Error << "could not find waypoint: " <<  lastRNDFWaypoint << Logger::endl;
										}
									}

									lastVec = vec;
								}
								else if (pathList.item(p).nodeName() == "randomrndf") {
									flt spd = pathList.item(p).attributes().namedItem("speed").nodeValue().toFloat();
									simobj->addWaypoint(Vec3(0.0f, 0.0f, 0.0f), spd, "1.1.1");
								}
								else if (pathList.item(p).nodeName() == "enu") {
									flt spd = pathList.item(p).attributes().namedItem("speed").nodeValue().toFloat();
									flt x = pathList.item(p).attributes().namedItem("x").nodeValue().toFloat();
									flt y = pathList.item(p).attributes().namedItem("y").nodeValue().toFloat();
									flt z = pathList.item(p).attributes().namedItem("z").nodeValue().toFloat();
									simobj->addWaypoint(Vec3(x, y, z), spd, "");
									DLOG("adding waypoint for enu at position " << x << ", " << y << ".");
								}
							}
						}
					}
					else if (subList.item(j).nodeName() == "position") {
						std::string posRNDFWaypoint = subList.item(j).firstChild().nodeValue().toStdString();

						vertex_descr vec = theRNDFGraph::instance().getVertex(posRNDFWaypoint);

						Vec3 simPos;
						simPos = parseVector(subList.item(j).firstChild().nodeValue());

						if (simPos.norm() == 0.0) {

							if (boost::graph_traits<aGraph>::null_vertex() != vec) {
								property_map<aGraph, vertex_data_t>::const_type
								vertexDataMap = get(vertex_data_t(), graph);
								Vec2 const & pos = vertexDataMap[vec].pos;
								simPos = ::math::zeroExtend(pos);

								//							std::cout << "NEW POS: " << simobj->position << std::endl;
							}
							else {
								Logger::log() << Logger::Error << "reading <position> : Could not find waypoint: " <<  posRNDFWaypoint << Logger::endl;
							}
						}

						simobj->position = simPos;
					}
					else if (subList.item(j).nodeName() == "material") {
						simobj->material = parseMaterial(subList.item(j).firstChild().nodeValue());
					}
					else if (subList.item(j).nodeName() == "orientation") {
						simobj->orientation = parseQuaternion(subList.item(j).firstChild().nodeValue());
					}
					else if (subList.item(j).nodeName() == "rndfpoint") {
						std::string lastRNDFWaypoint = subList.item(j).firstChild().nodeValue().toStdString();
						vertex_descr vec = theRNDFGraph::instance().getVertex(lastRNDFWaypoint);

						if (boost::graph_traits<aGraph>::null_vertex() != vec) {
							property_map<aGraph, vertex_data_t>::const_type
							vertexDataMap = get(vertex_data_t(), graph);
							Vec2 const & pos = vertexDataMap[vec].pos;
							simobj->model = "";
							simobj->name = "";
							simobj->position = ::math::zeroExtend(pos);
						}
						else {
							Logger::log() << Logger::Error << "reading <rndfpoint> : Could not find waypoint: " <<  lastRNDFWaypoint << Logger::endl;
						}
					}
					else if (subList.item(j).nodeName() == "size") {
						simobj->size = parseVector(subList.item(j).firstChild().nodeValue());
					}
					else if (subList.item(j).nodeName() == "model") {
						simobj->model = subList.item(j).firstChild().nodeValue().toStdString();
					}
					else if (subList.item(j).nodeName() == "modelscale") {
						simobj->modelscale = parseVector(subList.item(j).firstChild().nodeValue());
					}
					else if (subList.item(j).nodeName() == "name") {
						simobj->name = subList.item(j).firstChild().nodeValue().toStdString();
					}
				}
			}

			pimpl->objects.push_back(simobj);
		}
	}

	Logger::log() << Logger::Debug << "finished loading file, created " << pimpl->objects.size() << " obstacles " << Logger::endl;
	return true;
}


std::vector<Vec3> DummyController::addInterpolatedEdge(edge_descr const & edge)
{
	aGraph const & graph = theRNDFGraph::instance().getBoostGraph();
	property_map<aGraph, edge_data_t>::const_type edgeDataMap = get(edge_data_t(), graph);
	property_map<aGraph, vertex_data_t>::const_type vertexDataMap = get(vertex_data_t(), graph);
	std::vector<Vec3> retVector;

	EdgeData  curEdgeData = edgeDataMap[edge];
	vertex_descr src = source(edge, graph),
				 dst = target(edge, graph);
	VertexData const & vSrc = vertexDataMap[src],
					   & vDst = vertexDataMap[dst];

	// Vec2 srcPos = vSrc.pos, trgPos = vDst.pos;
	LaneSpline const & spline = *curEdgeData.laneSpline();
	flt edgeLength = abs(curEdgeData.targetParam - curEdgeData.sourceParam);
	Vec2 dir, point;
	int j = 0;
	flt stepSize = round(edgeLength) * 2;

	for (flt i = curEdgeData.sourceParam; true; i += edgeLength / stepSize) {

//		std::cout.precision(64);

		if (i >= curEdgeData.targetParam || j > stepSize) {
			break;
		}

//		std::cout << "Dummy::addIntpolEdge() -- i: " << i << " eLength:"
//				<< edgeLength << " tgt: " << curEdgeData.targetParam
//				<< "src: " << curEdgeData.sourceParam << std::endl;

		dir = normalized(spline.firstDerivative(i));
		point = spline(i);
		// adding waypoint for the simulated object
		retVector.push_back(Vec3(point(0), point(1), 0.0f));
//     simobj->addWaypoint(Vec3(point(0), point(1), 0.0f), spd, "");
		j++;

	}

	return retVector;
}
