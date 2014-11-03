#include "XingInfo.h"
#include <rtt/Logger.hpp>
#include <boost/foreach.hpp>
#include <util/Templates.h>
#include <patterns/Singleton.h>
#include <boost/foreach.hpp>
#include <set>

// #define VERBOSE

#if defined(VERBOSE)
#define DLOG(X)	Logger::log() << Logger::Info << X << Logger::endl
#else
#define DLOG(X) /**/
#endif

using namespace boost;
using namespace std;
using namespace ::math;
using namespace ::aa::modules::models::rndf;
using RTT::Logger;


template < typename Pair, typename Order = std::less<typename Pair::first_type> >
struct LessCrappy {
	bool operator()(Pair const & a, Pair const & b) const {
		return Order()(a.first, b.first);
	}
};


XingInfo::XingInfo()
	: mRNDFGraph(patterns::Singleton<RNDFGraph>::instance())
{}

XingInfo::~XingInfo()
{}

void sampleEdge(std::vector<std::pair<flt, Vec2> > & samples, EdgeData const & edgeData, flt stepWidth = 0.1f)
{
	flt const from = std::min(edgeData.sourceParam, edgeData.targetParam);
	flt const to = std::max(edgeData.sourceParam, edgeData.targetParam);
	flt f;

	for (f = from; f < to; f += stepWidth) {
		samples.push_back(std::make_pair(f, (*edgeData.laneSpline())(f)));
	}

	samples.push_back(std::make_pair(to, (*edgeData.laneSpline())(to)));
}

XingInfo::map_type const & XingInfo::getXingMap() const
{
	return mXingMapShared;
}

boost::shared_ptr<Xing> XingInfo::getXing(edge_descr const & xingEdge) const
{
	map_type::const_iterator mapIterShared = mXingMapShared.find(xingEdge);

	if (mapIterShared  != mXingMapShared.end()) {
		return mapIterShared->second;
	}

	return boost::shared_ptr<Xing>();
}

bool XingInfo::getNearestXing(boost::shared_ptr<Xing> & xingSh_ptr, Vec2 const  & pos) const
{
	flt minDist = std::numeric_limits<flt>::max();

	for (map_type::const_iterator it = mXingMapShared.begin();
			it != mXingMapShared.end(); it++) {
		flt dist = ssd(it->second->xingCenter, pos);

		if (dist < minDist) {
			xingSh_ptr = it->second;
			minDist = dist;
		}
	}

	return minDist < std::numeric_limits<flt>::max();
}

bool XingInfo::computeXings()
{
	Logger::In in("XingInfo");

	aGraph const & rGraph = mRNDFGraph.getBoostGraph();
	property_map<aGraph, edge_data_t>::const_type edgeDataMap = get(edge_data_t(), rGraph);
	property_map<aGraph, vertex_data_t>::const_type vertexDataMap = get(vertex_data_t(), rGraph);

	unsigned int xingID = 0;
	mVertexXing.clear();
	mXingMapShared.clear();

	// container
	set< pair<vertex_descr, bool> , LessCrappy< pair<vertex_descr, bool> > > vertsV; // bool indicates if v is handled as Exit or not
	set< pair<vertex_descr, bool> , LessCrappy< pair<vertex_descr, bool> > > resolvedV; // bool indicates if v is handled as Exit or not
	set< edge_descr, EdgeOrder > edgesE;

	// Main Iteration over ALL Edges
	BOOST_FOREACH(edge_descr const & e, edges(rGraph)) {
		// normally not neccessary, but in SwRI everything is possible...
		if (vertexDataMap[target(e, rGraph)].vertexType.isSet(VertexData::IN_ZONE)
				|| vertexDataMap[target(e, rGraph)].vertexType.isSet(VertexData::PERIMETER_POINT)
				|| vertexDataMap[target(e, rGraph)].vertexType.isSet(VertexData::PARKINGSPOT)

				|| vertexDataMap[source(e, rGraph)].vertexType.isSet(VertexData::IN_ZONE)
				|| vertexDataMap[source(e, rGraph)].vertexType.isSet(VertexData::PERIMETER_POINT)
				|| vertexDataMap[source(e, rGraph)].vertexType.isSet(VertexData::PARKINGSPOT)
		   )	{
			continue;
		}

		// skipping NOConnectionedges, already inserted edges, UTURN and ZONE edges
		if (!edgeDataMap[e].isConnection
				|| edgeDataMap[e].isUTurn
				|| edgeDataMap[e].inZone
				|| edgeDataMap[e].isMirrored
		   )	{
			continue;
		}

		//check if we already inserted this edge
		if (mXingMapShared.find(e) != mXingMapShared.end()) {
			continue;
		}

		// found a NEW connection edge - take inital Vertex and add to V
		vertex_descr vInit = source(e, rGraph);
		vertsV.insert(std::make_pair(vInit, true));


		// WHILE V not empty : take v € V and add all connected v to V and connectionEdges *(o/i)e to E
		while (!vertsV.empty()) 	{

			// move vTemp from V to R
			set< pair<vertex_descr, bool> , LessCrappy< pair<vertex_descr, bool> > >::iterator vSetIter = vertsV.begin();
			vertex_descr vTemp = vSetIter->first;
			bool vExit = vSetIter->second;

			resolvedV.insert(*vSetIter);

			// check entry-/exitType to iter over in-/outGoing Edges
			bool isExit  = vertexDataMap[vTemp].vertexType.isSet(VertexData::EXIT);
			bool isEntry = vertexDataMap[vTemp].vertexType.isSet(VertexData::ENTRY);

			if (isExit && vExit) {
// cout<<"   EX ";
				// iter over outgoingEdges (*oe) from vInit and find following edges (e)
				BOOST_FOREACH(edge_descr const & oe, out_edges(vTemp, rGraph)) {
					if (edgeDataMap[oe].isUTurn || edgeDataMap[oe].inZone || edgeDataMap[oe].isMirrored) {
						continue;
					}

					if (vertexDataMap[ target(oe, rGraph)].vertexType.isSet(VertexData::IN_ZONE)
							|| vertexDataMap[ target(oe, rGraph)].vertexType.isSet(VertexData::PERIMETER_POINT)
							|| vertexDataMap[ target(oe, rGraph)].vertexType.isSet(VertexData::PARKINGSPOT)) {
						continue;
					}

					if (resolvedV.find(std::make_pair(target(oe, rGraph), false)) != resolvedV.end()) {
						continue;
					}

					if (resolvedV.find(std::make_pair(target(oe, rGraph), true)) != resolvedV.end()) {
						continue;
					}

// cout<<" +["<<vertexDataMap[target( oe, rGraph)].name<<"] ";

					// mark Entry as Entry for Entry-Exit-WP
					vertsV.insert(std::make_pair(target(oe, rGraph), false));
					edgesE.insert(oe);
				}
			}

			if (isEntry && !vExit) {
// cout<<"EN ";
// 				iter over incoming (*ie) from vInit and find following edges (e)
				BOOST_FOREACH(edge_descr const & ie, in_edges(vTemp, rGraph)) {
					if (edgeDataMap[ie].isUTurn || edgeDataMap[ie].inZone || edgeDataMap[ie].isMirrored) {
						continue;
					}

					if (vertexDataMap[ source(ie, rGraph)].vertexType.isSet(VertexData::IN_ZONE)
							|| vertexDataMap[ source(ie, rGraph)].vertexType.isSet(VertexData::PERIMETER_POINT)
							|| vertexDataMap[ source(ie, rGraph)].vertexType.isSet(VertexData::PARKINGSPOT)) {
						continue;
					}

					if (resolvedV.find(std::make_pair(source(ie, rGraph), true)) != resolvedV.end()) {
						continue;
					}

					if (resolvedV.find(std::make_pair(source(ie, rGraph), false)) != resolvedV.end()) {
						continue;
					}

// cout<<" +["<<vertexDataMap[ source(*ie, rGraph) ].name<<"] ";

					// mark Entry as Exit for Entry-Exit-WP
					vertsV.insert(std::make_pair(source(ie, rGraph), true));
					edgesE.insert(ie);
				}
			}

// cout<<"\n-["<<vertexDataMap[vTemp].name<<"] "<<endl;
			vertsV.erase(vSetIter);
		}

// COMMENT : WE NEED THEM TO DISTINGUISH CRAPPY XINGS FROM NORMAL XINGS
// 		if (resolvedV.size() <= 2) {
// 			resolvedV.clear();
// 			vertsV.clear();
// 			edgesE.clear();
// 			continue;
// 		}


		// edge/vertex scanning complete: now build Xing
		xingID++;
		DLOG("[-----START--- #" << xingID << ". Xing--------]");

		boost::shared_ptr<Xing> xing(new Xing());
		// TODO : sort R into Lanes
		Xing::Lane tempLane;

		while (!resolvedV.empty()) {
// 			tempLane.clear();

			set< pair<vertex_descr, bool> , LessCrappy< pair<vertex_descr, bool> > >::iterator vResolvedIter = resolvedV.begin();
			vertex_descr vTemp = (*resolvedV.begin()).first;
			bool vHandledAsExit = (*resolvedV.begin()).second;
			resolvedV.erase(vResolvedIter);
			Vec2 tempDir, tempPos;
			bool vDirRes = mRNDFGraph.trackDir(tempDir, vTemp);
			assert(vDirRes);
			XingWP wp;
			wp.wp = vTemp;
			wp.pos = vertexDataMap[vTemp].pos;
			wp.dir = tempDir;
			wp.isExit =  vertexDataMap[vTemp].vertexType.isSet(VertexData::EXIT) && vHandledAsExit;
			wp.isEntry = vertexDataMap[vTemp].vertexType.isSet(VertexData::ENTRY) && !vHandledAsExit;
			wp.isBoth =  vertexDataMap[vTemp].vertexType.isSet(VertexData::ENTRY) && vertexDataMap[vTemp].vertexType.isSet(VertexData::EXIT);
			wp.isStop =  vertexDataMap[vTemp].vertexType.isSet(VertexData::STOP_SIGN) || vertexDataMap[vTemp].vertexType.isSet(VertexData::GIVE_WAY) ;

			// invert EntryDirs and set correct dir to "normal" WPs;
			if (wp.isExit || vHandledAsExit) {
				wp.dir *= -1.f;
			}


			tempLane.push_back(wp);

			DLOG("| * WP : " << vertexDataMap[vTemp].name
				 << " [" << vertexDataMap[vTemp].pos[0]
				 << "," << vertexDataMap[vTemp].pos[1]
				 << "]"
				 << ", exit, entry, both : ["
				 << wp.isExit << "," << wp.isEntry << "," << wp.isBoth
				);

// 			// sameLanesearch
// 			bool isExit = vertexDataMap[vTemp].vertexType.isSet(VertexData::EXIT);
// 			edge_descr
// 				trackEdge,
// 				neighbourEdge,
// 				l,r;
// 			Vec2
// 				neighbourPos, curPos, checkPos, lPos, rPos;
// 			flt
// 				closestParm, closestSquaredDist;
//
// 			if ( isExit )
// 				trackEdge = *in_edges(vTemp, rGraph).first;
// 			else
// 				trackEdge = *out_edges(vTemp, rGraph).first;
//
// // 			mRNDFGraph.getDirectNeighbouredEdge( curPos, lastEdge, true, neighbourEdge, neighbourPos,
// // 				 									 closestParm, closestSquaredDist);
//
// 			// TODO : left und right NEdge.
//
// 			checkPos = vertexDataMap[vTemp].pos + 0.05f * tempDir; //HACK :to assure there is a neighbour lane
// 			pair <flt, flt> ne = mRNDFGraph.getDirectNeighbouredEdge( checkPos, trackEdge, l, r, lPos, rPos );
//
//  			if ( ne.first >= 0.0f)
// 				neighbourEdge = l;
//  			if ( ne.second >= 0.0f)
// 				neighbourEdge = r;
//
// 			if ( ne.first >= 0.0f || ne.second >= 0.0f ) {
//
// 				optional<edge_descr> edgeOpt;
// 				if ( edgeDataMap[neighbourEdge].isMirrored ) {
// 					edgeOpt = mRNDFGraph.getMirroredEdge(neighbourEdge);
// 					if (edgeOpt)
// 						neighbourEdge = edgeOpt.get();
// 				}
//
// 				bool found = false;
// 				if ( resolvedV.find(source(neighbourEdge, rGraph)) != resolvedV.end() ) {
// 					found = true;
// 					vTemp = source(neighbourEdge, rGraph);
// 				}
// 				if ( resolvedV.find(target(neighbourEdge, rGraph)) != resolvedV.end() ) {
// 					found = true;
// 					vTemp = target(neighbourEdge, rGraph);
// 				}
//
// 				if (found) {
// 					resolvedV.erase(vTemp);
// 					vDirRes = mRNDFGraph.trackDir(tempDir, vTemp );
// 					assert(vDirRes);
// 					tempWP.wp = vTemp;
// 					tempWP.dir = tempDir;
// 					tempWP.isExit = vertexDataMap[vTemp].vertexType.isSet(VertexData::EXIT);
// 					tempWP.isStop = vertexDataMap[vTemp].stopSign;
// 					tempLane.push_back( tempWP );
// 					cout << "| * WP : " << vertexDataMap[vTemp].name
// 						<< " [" << vertexDataMap[vTemp].pos[0]
// 						<< ","<< vertexDataMap[vTemp].pos[1] << "]" << endl;
//
// 				}
// 			}

// 			tempXing.xingLanes.push_back(tempLane);
		}

		xing->xingLanes.push_back(tempLane);

// 		DLOG("| number : " << numV);

		// setting Xing information
		xing->xingID = xingID;
		xing->mCrossingEdges.insert(xing->mCrossingEdges.begin(),
									edgesE.begin(), edgesE.end());

		// adding all xingEdges into xingMap
		for (set< edge_descr, EdgeOrder >::iterator setIter = edgesE.begin(); setIter != edgesE.end(); setIter++) {
			mXingMapShared.insert(std::make_pair(*setIter, xing));
			DLOG("| edge " << edgeDataMap[*setIter].name); /* << " with id : " << tempID);*/
		}

		// adding all vertices into vertexXingMap
		for (unsigned int l = 0; l < xing->xingLanes.size(); ++l) {
			for (unsigned int wp = 0; wp < xing->xingLanes[l].size(); wp++) {
				vertex_descr temp = xing->xingLanes[l][wp].wp;
				mVertexXing.insert(std::make_pair(temp , xing));
			}
		}

		DLOG("[--- END of " << xingID << ". Xing with " << xing->xingLanes.size() <<
			 "Lanes)---] \n");

		// FINISHED THIS XING ... mainlooping
		edgesE.clear();
		resolvedV.clear();
		tempLane.clear();
	}

	// MERGING : whenever a Node which is not an EXIT or an ENTRY occures in more than one Xing -> merge them
	// TODO : use information about XingCenter for merging operation
	DLOG("[ STARTING mergeprocess ]");
	set< vertex_descr > mergedVerts;

	for (mmap_type::iterator vxIter = mVertexXing.begin(); vxIter != mVertexXing.end(); vxIter++) {
		vertex_descr vTemp = vxIter->first ;

		// TODO : check if a count bigger two is possible
		if (mVertexXing.count(vTemp) == 2u)  {
// 			cout << " found a Node :" << mVertexXing.count( vTemp ) << " times !" << endl;

			if (vertexDataMap[vTemp].vertexType.isSet(VertexData::ENTRY) || vertexDataMap[vTemp].vertexType.isSet(VertexData::EXIT)) {
				continue;
			}

			uint numV = 0;
			Vec2 xingCenter(0, 0);
			Xing::Lane tempLane;
			boost::shared_ptr<Xing> mergedXing(new Xing());

			// make sure that every Vertex is only added once in the merged Xing
			mergedVerts.clear();

			//take both occurances
			mmap_type::iterator vxLower = mVertexXing.lower_bound(vTemp);
			mmap_type::iterator vxUpper = mVertexXing.upper_bound(vTemp);
			mmap_type::iterator vxTemp  = mVertexXing.upper_bound(vTemp);
			vxTemp--;

			unsigned int fstID = vxLower->second->xingID, sndID = vxTemp->second->xingID;

			DLOG("[-----START-MERGING-- #" << fstID << ". AND " << sndID
				 << ". Xing [" << vertexDataMap[vxLower->first].name << "]");

			for (mmap_type::iterator vxCrap = vxLower; vxCrap != vxUpper; vxCrap++) {
				vector< vector< XingWP > > const & lanes = vxCrap->second->xingLanes;

				for (unsigned int l = 0; l < lanes.size(); l++) {
					for (unsigned int wp = 0; wp < lanes[l].size(); wp++) {
						// kickk every flt blue ass
						if (mVertexXing.count(lanes[l][wp].wp) == 2u
								&& !vertexDataMap[lanes[l][wp].wp].vertexType.isSet(VertexData::EXIT)
								&& !vertexDataMap[lanes[l][wp].wp].vertexType.isSet(VertexData::ENTRY)) {
							DLOG(" KICK " << vertexDataMap[lanes[l][wp].wp].name << " !");
							continue;
						}

						if (mergedVerts.find(lanes[l][wp].wp) !=  mergedVerts.end()) {
							continue;
						}

						mergedVerts.insert(lanes[l][wp].wp);
						tempLane.push_back(lanes[l][wp]);
						xingCenter += vertexDataMap[ lanes[l][wp].wp ].pos;
						xingID = vxCrap->second->xingID + 10000; // taking last ID + shift
						DLOG("| * WP : " << vertexDataMap[ lanes[l][wp].wp ].name
							 << " [" << vertexDataMap[ lanes[l][wp].wp ].pos[0]
							 << "," << vertexDataMap[ lanes[l][wp].wp ].pos[1] << "]");

					}
				}
			}

			//deleting from mVertexXing and ajusting iteartor ...
			mVertexXing.erase(vxLower, vxUpper);
			vxIter = vxUpper;

			//merge into tempXing
			xingCenter /= flt(numV);
			mergedXing->xingCenter = xingCenter;
			mergedXing->xingID = xingID;
			mergedXing->xingLanes.push_back(tempLane);
			DLOG("[--- END of MERGING " << xingID << ". NEWMERGEDXing with " << mergedXing->xingLanes.size() <<
				 " lanes, center (" << xingCenter << ")---] \n");


			//replace every occurence in mXingMapShared with newMergedXing

			for (map_type::iterator edgeMapIter = mXingMapShared.begin(); edgeMapIter != mXingMapShared.end(); ++edgeMapIter) {
				if ((edgeMapIter->second->xingID == fstID) || (edgeMapIter->second->xingID == sndID)) {
					edgeMapIter->second = mergedXing;
				}
			}

			//replace every occurence in mvERTEXmAP with newMergedXing
			for (mmap_type::iterator vertexMapIter = mVertexXing.begin(); vertexMapIter != mVertexXing.end(); ++vertexMapIter) {
				if ((vertexMapIter->second->xingID == fstID) || (vertexMapIter->second->xingID == sndID)) {
					vertexMapIter->second = mergedXing;
				}
			}
		}
	}

	DLOG("FINISHED Merging XINGS");

	// kick EVERY occurance of blue bastards
	for (mmap_type::iterator vxIter = mVertexXing.begin(); vxIter != mVertexXing.end();) {
		vertex_descr vTemp = vxIter->first ;

		// assure were only handle blue ones -> they can only belong to ONE Xing
		if (vertexDataMap[vTemp].vertexType.isSet(VertexData::ENTRY) || vertexDataMap[vTemp].vertexType.isSet(VertexData::EXIT)) {
			++vxIter;
			continue;
		}

		boost::shared_ptr<Xing> xingPtr = vxIter->second;

		if (out_degree(vTemp, rGraph) == 0 || in_degree(vTemp, rGraph) == 0) {
			RTT::log(RTT::Warning) << "Vertex " << vertexDataMap[vTemp].name << " on crossing " <<
								   xingPtr->xingID << " is terminal, but neither entry nor exit" <<  RTT::endlog();
			mVertexXing.erase(vxIter++);
			continue;
		}

		assert((out_degree(vTemp, rGraph) > 0) && (in_degree(vTemp, rGraph) > 0));
		edge_descr inEdge = *in_edges(vTemp, rGraph).first;
		edge_descr outEdge = *out_edges(vTemp, rGraph).first;

		assert(inEdge != edge_descr() && outEdge != edge_descr());

		vertex_descr inV = source(inEdge , rGraph);
		vertex_descr outV = target(outEdge , rGraph);

		uint tempID = xingPtr->xingID;
		std::multimap< vertex_descr, boost::shared_ptr<Xing> >::iterator inIter = mVertexXing.find(inV);
		std::multimap< vertex_descr, boost::shared_ptr<Xing> >::iterator outIter = mVertexXing.find(outV);

		if (inIter != mVertexXing.end() && outIter != mVertexXing.end()) {
			uint inID = inIter->second->xingID;
			uint outID = outIter->second->xingID;

			if ((tempID == inID) && (tempID == outID)) {
				// killing every last BLUE motherfucker, and erase his history
				unsigned int numV = 0;

				for (unsigned int l = 0; l < xingPtr->xingLanes.size(); l++) {
					for (unsigned int wp = 0; wp < xingPtr->xingLanes[l].size(); wp++) {

						if (vTemp == xingPtr->xingLanes[l][wp].wp) {
							DLOG("!!!!  ELIMINATING  IN-CENTER-BLUE-NODE FROM VMAP AND EMAP !!!!!");
							xingPtr->xingLanes[l].erase(xingPtr->xingLanes[l].begin() + wp);
							wp--;
							continue;
						}

						numV++;
					}
				}
			}
		}

		++vxIter;
	}

	std::set<boost::shared_ptr<Xing> > done;

	BOOST_FOREACH(map_type::value_type & x, mXingMapShared) {
		boost::shared_ptr<Xing> xing(x.second);

		// computing xingCenter
		if (done.count(xing) > 0) {
			continue;
		}

		done.insert(xing);

		Vec2 minp(std::numeric_limits< flt >::max(), std::numeric_limits< flt >::max()), maxp(-minp);
		Vec2 xingCenter(0, 0);

		for (unsigned int l = 0; l < xing->xingLanes.size(); ++l) {
			for (unsigned int wp = 0; wp < xing->xingLanes[l].size(); wp++) {
				VertexData const & vertexData(vertexDataMap[xing->xingLanes[l][wp].wp]);

				if (vertexData.vertexType.isSet(VertexData::ENTRY) || vertexData.vertexType.isSet(VertexData::EXIT)) {
					for (unsigned int d = 0; d < 2; ++d) {
						minp(d) = std::min(minp(d), vertexData.pos(d));
						maxp(d) = std::max(maxp(d), vertexData.pos(d));
					}
				}
			}
		}

		xing->xingCenter = flt(0.5) * (minp + maxp);

		typedef std::pair<flt, Vec2> ParamPoint;
		std::vector<ParamPoint> samplesA, samplesB;

		// Find overlapping edges
		// TODO: Honour the lane-size
		for (unsigned int i = 0; i < xing->mCrossingEdges.size(); ++i) {
			edge_descr const & edgeA(xing->mCrossingEdges[i]);
			EdgeData const & edgeDataA(edgeDataMap[edgeA]);
			samplesA.clear();
			sampleEdge(samplesA, edgeDataA);

			for (unsigned int j = i + 1; j < xing->mCrossingEdges.size(); ++j) {
				edge_descr const & edgeB(xing->mCrossingEdges[j]);
				EdgeData const & edgeDataB(edgeDataMap[edgeB]);
				samplesB.clear();
				sampleEdge(samplesB, edgeDataB);
				flt minDist2(std::numeric_limits<flt>::max());
				ParamPoint minA, minB;
				BOOST_FOREACH(ParamPoint const & ppA, samplesA) {
					BOOST_FOREACH(ParamPoint const & ppB, samplesB) {
						flt const dist2 = ssd(ppA.second, ppB.second);

						if (dist2 < minDist2) {
							minDist2 = dist2;
							minA = ppA;
							minB = ppB;
						}
					}
				}

				if (minDist2 <= 4.0) {
					xing->mIntersectingEdges[edgeA].push_back(Xing::EdgeIntersection(edgeB, minB.first, minA.first));
					xing->mIntersectingEdges[edgeB].push_back(Xing::EdgeIntersection(edgeA, minA.first, minB.first));
				}
			}
		}
	}

	return true;
}

