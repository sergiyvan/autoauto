#include "RndfGenerator.h"
#include "RndfReader.h"

#include <rtt/Logger.hpp>
#include <iostream>
#include <fstream>
#include <cstring>
#include <vector>
#include "math.h"
#include <aaconfig.h>
#include <boost/filesystem/path.hpp>
#include <math/AutoMath.h>
#include <rtt/OperationCaller.hpp>
#include <util/TaskContextFactory.h>
#include <boost/date_time/gregorian/gregorian.hpp>

namespace aa
{
namespace modules
{
namespace models
{
namespace rndf
{
namespace io
{

using namespace ::math;

REGISTERTASKCONTEXT(RNDFGenerator);

typedef patterns::Singleton<RNDFGraph> theRNDFGraph;

using namespace std;
using namespace math;
using namespace RTT;
using namespace boost::gregorian;

RNDFGenerator::RNDFGenerator(std::string const & name)
	: util::RtTaskContext(name)
	// default values set to match tempelhof course, please backup if you are going to change them
	, mStraightStartLat("StraightStartLat", "latitude of start point of straight segment", 52.474568) //52.480031
	, mStraightStartLong("StraightStartLong", "longitude of start point of straight segment", 13.393655) //13.391667
	, mStraightEndLat("StraightEndLat", "latitude of end point of straight segment", 52.475300) //52.481253
	, mStraightEndLong("StraightEndLong", "longitude of end point of straight segment", 13.410648) //13.391667

	, mCircleLat("CircleLat", "latitude of circle center", 52.480031)
	, mCircleLong("CircleLong", "longitude of circle center", 13.391667)
	, mCircleRadius("CircleRadius", "radius of circle in meters", 50.0)

	, mEightLat("EightLat", "latitude of eight center", 52.480031)
	, mEightLong("EightLong", "longitude of eight center", 13.391667)
	, mEightRadius("EightRadius", "radius of eight circles in meters", 25.0)
	, mEightSwivel("EightSwivel", "angle at which to swivel in degrees (0-80)", 20.0)
	, mEightRotAngle("EightRotAngle", "rotating angle of complete eight course", 45.0)

	, mMultilaneLat("MultilaneLat", "latitude of crossing of multilane course", 52.480031)
	, mMultilaneLong("MultilaneLong", "longitude of crossing of multilane course", 13.391667)
	, mMultilaneSegmentLength("MultilaneSegmentLength", "segment length of each segment in meters", 20.0)
	, mMultilaneLanes("MultilaneLanes", "number of lanes in each direction", 4)

	, mSampleSizeDegree("SampleSizeDegree", "sample size when iterating over degrees", 5.0)
	, mSampleSizeUnit("SampleSizeUnit", "sample size when iterating over units (meters)", 2.0)
{
	addProperty(mStraightStartLat);
	addProperty(mStraightStartLong);
	addProperty(mStraightEndLat);
	addProperty(mStraightEndLong);

	addProperty(mCircleLat);
	addProperty(mCircleLong);
	addProperty(mCircleRadius);

	addProperty(mEightLat);
	addProperty(mEightLong);
	addProperty(mEightRadius);
	addProperty(mEightSwivel);
	addProperty(mEightRotAngle);

	addProperty(mMultilaneLat);
	addProperty(mMultilaneLong);
	addProperty(mMultilaneSegmentLength);
	addProperty(mMultilaneLanes);

	addProperty(mSampleSizeDegree);
	addProperty(mSampleSizeUnit);

	addOperation("generateAndLoadDefaultStraightRNDF", &RNDFGenerator::generateAndLoadDefaultStraightRNDF, this, RTT::ClientThread).doc("generateAndLoadDefaultStraightRNDF").arg("fileName", "filename for RNDF");

	addOperation("generateAndLoadDefaultCircleRNDF", &RNDFGenerator::generateAndLoadDefaultCircleRNDF, this, RTT::ClientThread).doc("generateAndLoadCircleRNDF").arg("fileName", "filename for RNDF");

	addOperation("generateAndLoadDefaultEightRNDF", &RNDFGenerator::generateAndLoadDefaultEightRNDF, this, RTT::ClientThread).doc("generateAndLoadEightRNDF").arg("fileName", "filename for RNDF");

	addOperation("generateAndLoadDefaultMultilaneRNDF", &RNDFGenerator::generateAndLoadDefaultMultilaneRNDF, this, RTT::ClientThread).doc("generateAndLoadDefaultMultilaneRNDF").arg("fileName", "filename for RNDF");
}

RNDFGenerator::~RNDFGenerator()
{}

bool RNDFGenerator::startHook()
{
	return true;
}

void RNDFGenerator::updateHook()
{
}

void RNDFGenerator::stopHook()
{
}

bool RNDFGenerator::generateAndLoadDefaultStraightRNDF(std::string const & fname)
{
	return generateAndLoadStraightRNDF(mStraightStartLat, mStraightStartLong, mStraightEndLat, mStraightEndLong, fname);
}

bool RNDFGenerator::generateAndLoadDefaultCircleRNDF(std::string const & fname)
{
	return generateAndLoadCircleRNDF(mCircleLat, mCircleLong, mCircleRadius, fname);
}

bool RNDFGenerator::generateAndLoadDefaultEightRNDF(std::string const & fname)
{
	return generateAndLoadEightRNDF(mEightLat, mEightLong, mEightRadius, mEightSwivel, mEightRotAngle, fname);
}

bool RNDFGenerator::generateAndLoadDefaultMultilaneRNDF(std::string const & fname)
{
	return generateAndLoadMultilaneRNDF(mMultilaneLat, mMultilaneLong, mMultilaneSegmentLength, mMultilaneLanes, fname);
}



bool RNDFGenerator::generateAndLoadStraightRNDF(flt startPointLat, flt startPointLong, flt endPointLat, flt endPointLong, std::string const & fname)
{
	Logger::In in("RNDFGenerator::generateAndLoadStraightRNDF");

	mMutex.lock();
	mRNDFData = generateStraightRNDF(Vec2(startPointLat, startPointLong), Vec2(endPointLat, endPointLong), fname);

	if (!mRNDFData.get()) {
		Logger::log() << Logger::Error << "failed loading RNDF \"" << fname << '"' << Logger::endl;
	}
	else {
		RNDFGraph & rndfGraph = theRNDFGraph::instance();
		rndfGraph.resetGraph();
		rndfGraph.buildGraph(mRNDFData);
	}

	Logger::log() << Logger::Debug << "Circle RNDF " << fname << " succesfully loaded!" << Logger::endl;
	mMutex.unlock();
	return true;
}


bool RNDFGenerator::generateAndLoadCircleRNDF(flt midPointLat, flt midPointLong, flt radius, std::string const & fname)
{
	Logger::In in("RNDFGenerator::generateAndLoadCircleRNDF");

	mMutex.lock();
	mRNDFData = generateCircleRNDF(Vec2(midPointLat, midPointLong), radius, fname);

	if (!mRNDFData.get()) {
		Logger::log() << Logger::Error << "failed loading RNDF \"" << fname << '"' << Logger::endl;
	}
	else {
		RNDFGraph & rndfGraph = theRNDFGraph::instance();
		rndfGraph.resetGraph();
		rndfGraph.buildGraph(mRNDFData);
	}

	Logger::log() << Logger::Debug << "Circle RNDF " << fname << " succesfully loaded!" << Logger::endl;
	mMutex.unlock();
	return true;
}


bool RNDFGenerator::generateAndLoadEightRNDF(flt midPointLat, flt midPointLong, flt radius, flt swivel, flt rotAngle, std::string const & fname)
{
	Logger::In in("RNDFGenerator::generateAndLoadEightRNDF");

	mMutex.lock();
	mRNDFData = generateEightRNDF(Vec2(midPointLat, midPointLong), radius, swivel, rotAngle, fname);

	if (!mRNDFData.get()) {
		Logger::log() << Logger::Error << "failed loading RNDF \"" << fname << '"' << Logger::endl;
	}
	else {
		RNDFGraph & rndfGraph = theRNDFGraph::instance();
		rndfGraph.resetGraph();
		rndfGraph.buildGraph(mRNDFData);
	}

	Logger::log() << Logger::Debug << "Eight RNDF " << fname << " succesfully loaded!" << Logger::endl;
	mMutex.unlock();
	return true;
}


bool RNDFGenerator::generateAndLoadMultilaneRNDF(flt midPointLat, flt midPointLong, flt segmentLength, int lanes, std::string const & fname)
{
	Logger::In in("RNDFGenerator::generateAndLoadMultilaneRNDF");

	mMutex.lock();
	mRNDFData = generateMultilaneRNDF(Vec2(midPointLat, midPointLong), segmentLength, lanes, fname);

	if (!mRNDFData.get()) {
		Logger::log() << Logger::Error << "failed loading RNDF \"" << fname << '"' << Logger::endl;
	}
	else {
		RNDFGraph & rndfGraph = theRNDFGraph::instance();
		rndfGraph.resetGraph();
		rndfGraph.buildGraph(mRNDFData);
	}

	Logger::log() << Logger::Debug << "Multilane RNDF " << fname << " succesfully loaded!" << Logger::endl;
	mMutex.unlock();
	return true;
}



boost::shared_ptr<RNDFData> RNDFGenerator::generateStraightRNDF(Vec2 startPoint, Vec2 endPoint, std::string const & fname)
{
	Logger::In in("RNDFGenerator::generateStraightRNDF");

	// write out straight RNDF to file
	boost::filesystem::path resource_path(cmake_resource_dir);

	string filename = resource_path.string() + "/" + fname;

	ofstream out(filename.c_str(), ios::out & ios::binary & ios::trunc);

	if (!out) {
		Logger::log() << Logger::Error << "Could not write out file to " << filename << Logger::endl;
		return boost::shared_ptr<RNDFData>();
	}

	// Straight template points
	vector<Vec2> wp_forward;
	vector<Vec2> wp_backward;

	// use formula with earth's average meridional radius M = 6,367,449 m
	int M = 6367449, a = 6378137, b = 6356752;//FIXME there was a .3, but b is an integer;
	Vec2 midPoint = (endPoint - startPoint) / flt(2.0);
	flt phi = midPoint[0] * D2R;
	flt x = PI / 180.0 * cos(phi) * sqrt((pow(a, 4) * pow(cos(phi), 2) + pow(b, 4) * pow(sin(phi), 2)) / (pow(a * cos(phi), 2) + pow(b * sin(phi), 2)));


	flt ydiff = (endPoint[0] - startPoint[0]) * x * cos(phi);
	flt xdiff = (endPoint[1] - startPoint[1]) * x;
	flt m = ydiff / (xdiff == 0 ? epsilon : xdiff);
	flt l = sqrt(ydiff * ydiff + xdiff * xdiff);

	//rougher sampling
	flt straightSampleSize = 5.0f;

	// create forward waypoints
	for (flt i = 0; i < l; i += straightSampleSize) {
		Vec2 straightPoint((endPoint[0] - startPoint[0])*i / l, (endPoint[1] - startPoint[1])*i / l);
		wp_forward.push_back(startPoint + straightPoint);
	}

	wp_forward.push_back(endPoint);

	flt w = 7 / x;
	flt wx = sqrt(w * w / (1 + 1 / m / m));
	Vec2 shiftVec;

	if (xdiff == 0) {
		shiftVec = Vec2(0, w);
	}
	else {
		shiftVec = Vec2(-wx / m, wx);
	}

	// map backward waypoints left to forward waypoints
	for (size_t i = 0; i < wp_forward.size(); ++i) {
		wp_backward.push_back(wp_forward[wp_forward.size() - i - 1] + shiftVec);
	}

	vector< vector<Vec2> > waypoints;
	waypoints.push_back(wp_forward);
// 	waypoints.push_back(wp_backward);

	out << "RNDF_name	TestStraight" << endl;
	out << "num_segments	1" << endl;
	out << "num_zones	0" << endl;
	out << "format_version	1.0" << endl;
	date today(day_clock::local_day());
	out << "creation_date	" << today.month().as_number() << "/" << today.day() << "/" << today.year() << endl;


	out << "segment	1" << endl;
	out << "num_lanes	1" << endl;
// 	out << "num_lanes	2" << endl;
	out << "segment_name	StraightForward" << endl;

	out << "lane	1.1" << endl;
	out << "num_waypoints	" << waypoints[0].size() << endl;
	out << "lane_width	12" << endl;
	out << "left_boundary	solid_white" << endl;
	out << "right_boundary	solid_white" << endl;

	out << "checkpoint	1.1.1	1" << endl;
	out << "checkpoint	1.1." << waypoints[0].size() / 4 << "	2" << endl;
	out << "checkpoint	1.1." << 2 * waypoints[0].size() / 4 << "	3" << endl;
	out << "checkpoint	1.1." << 3 * waypoints[0].size() / 4 << "	4" << endl;
	out << "checkpoint	1.1." << waypoints[0].size() << "	5" << endl;

	out.precision(10);

	for (size_t i = 0; i < waypoints[0].size(); ++i) {
		out << "1.1." << (i + 1) << " " << waypoints[0][i] << endl;
	}

	out << "end_lane" << endl;

	/*
	out << "lane	1.2" << endl;
	out << "num_waypoints	" << waypoints[1].size() << endl;
	out << "lane_width	12" << endl;
	out << "left_boundary	solid_white" << endl;
	out << "right_boundary	solid_white" << endl;

	out.precision(10);
	for(int i=0; i < waypoints[1].size(); ++i)
	{
		out << "1.2." << (i+1) << " " << waypoints[1][i] << endl;
	}

	out << "end_lane" << endl;
	*/

	out << "end_segment" << endl;

	out << "end_file" << endl;

	out.close();
	Logger::log() << Logger::Debug << "Straight RNDF " << fname << " succesfully generated!" << Logger::endl;


	// generate MDF
	string mdf_fname = fname;
	size_t result = mdf_fname.find_last_of('.');

	if (std::string::npos != result) {
		mdf_fname.erase(result);
	}

	mdf_fname.append(".mdf");

	string filenameMDF = resource_path.string() + "/" + mdf_fname;

	ofstream outMDF(filenameMDF.c_str(), ios::out & ios::binary & ios::trunc);

	boost::filesystem::path rndf_fp(filename);
	string rndf_f(rndf_fp.filename().c_str());
	result = rndf_f.find_last_of('.');

	if (std::string::npos != result) {
		rndf_f.erase(result);
	}

	if (!outMDF) {
		Logger::log() << Logger::Error << "Could not write out file to " << mdf_fname << Logger::endl;
		return boost::shared_ptr<RNDFData>();
	}

	outMDF << "MDF_name	TestStraight" << endl;
	outMDF << "RNDF	" << rndf_f << endl;
	outMDF << "format_version	1.0" << endl;
	outMDF << "creation_date	" << today.month().as_number() << "/" << today.day() << "/" << today.year() << endl;

	outMDF << "checkpoints" << endl;
	outMDF << "num_checkpoints	5" << endl;
	outMDF << "1" << endl;
	outMDF << "2" << endl;
	outMDF << "3" << endl;
	outMDF << "4" << endl;
	outMDF << "5" << endl;
	outMDF << "end_checkpoints" << endl;

	outMDF << "speed_limits" << endl;
	outMDF << "num_speed_limits	1" << endl;
	outMDF << "1	6.21371192	18.6411358" << endl;
	outMDF << "end_speed_limits" << endl;
	outMDF << "end_file" << endl;


	outMDF.close();
	Logger::log() << Logger::Debug << "Straight MDF " << mdf_fname << " succesfully generated!" << Logger::endl;


	// read RNDF file to RNDFData
	RNDFReader rndfreader;
	return rndfreader.loadRNDF(filename);
}


boost::shared_ptr<RNDFData> RNDFGenerator::generateCircleRNDF(Vec2 midPoint, flt radius, std::string const & fname)
{
	Logger::In in("RNDFGenerator::generateCircleRNDF");

	// write out circle RNDF to file
	boost::filesystem::path resource_path(cmake_resource_dir);

	string filename = resource_path.string() + "/" + fname;

	ofstream out(filename.c_str(), ios::out & ios::binary & ios::trunc);

	if (!out) {
		Logger::log() << Logger::Error << "Could not write out file to " << filename << Logger::endl;
		return boost::shared_ptr<RNDFData>();
	}

	// Circle template points
	vector<Vec2> waypoints;

	// use formula with earth's average meridional radius M = 6,367,449 m
	int M = 6367449, a = 6378137, b = 6356752;//FIXME there was a .3, but b is an integer;
	flt phi = midPoint[0] * D2R;
	flt x = PI / 180.0 * cos(phi) * sqrt((pow(a, 4) * pow(cos(phi), 2) + pow(b, 4) * pow(sin(phi), 2)) / (pow(a * cos(phi), 2) + pow(b * sin(phi), 2)));

	for (flt i = 1, j = 1; i <= 360 + mSampleSizeDegree.get(); i += mSampleSizeDegree.get(), ++j) {
		Vec2 circlePoint(radius / x * cos(i * D2R), radius / x * sin(i * D2R) / cos(phi));
		waypoints.push_back(midPoint + circlePoint);
	}

	out << "RNDF_name	TestCircle" << endl;
	out << "num_segments	1" << endl;
	out << "num_zones	0" << endl;
	out << "format_version	1.0" << endl;
	date today(day_clock::local_day());
	out << "creation_date	" << today.month().as_number() << "/" << today.day() << "/" << today.year() << endl;

	out << "segment	1" << endl;
	out << "num_lanes	1" << endl;
	out << "segment_name	Circle" << endl;

	out << "lane	1.1" << endl;
	out << "num_waypoints	" << waypoints.size() << endl;
	out << "lane_width	12" << endl;
	out << "left_boundary	solid_white" << endl;
	out << "right_boundary	solid_white" << endl;
	out << "checkpoint	1.1.1	1" << endl;

	out.precision(10);

	for (size_t i = 0; i < waypoints.size(); ++i) {
		out << "1.1." << (i + 1) << " " << waypoints[i] << endl;
	}

	out << "end_lane" << endl;
	out << "end_segment" << endl;
	out << "end_file" << endl;

	out.close();

	Logger::log() << Logger::Debug << "Circle RNDF " << fname << " succesfully generated!" << Logger::endl;


	// generate MDF
	string mdf_fname = fname;
	size_t result = mdf_fname.find_last_of('.');

	if (std::string::npos != result) {
		mdf_fname.erase(result);
	}

	mdf_fname.append(".mdf");

	string filenameMDF = resource_path.string() + "/" + mdf_fname;

	ofstream outMDF(filenameMDF.c_str(), ios::out & ios::binary & ios::trunc);

	boost::filesystem::path rndf_fp(filename);
	string rndf_f(rndf_fp.filename().c_str());
	result = rndf_f.find_last_of('.');

	if (std::string::npos != result) {
		rndf_f.erase(result);
	}

	if (!outMDF) {
		Logger::log() << Logger::Error << "Could not write out file to " << mdf_fname << Logger::endl;
		return boost::shared_ptr<RNDFData>();
	}

	outMDF << "MDF_name	TestCircle" << endl;
	outMDF << "RNDF	" << rndf_f << endl;
	outMDF << "format_version	1.0" << endl;
	outMDF << "creation_date	" << today.month().as_number() << "/" << today.day() << "/" << today.year() << endl;

	outMDF << "checkpoints" << endl;
	outMDF << "num_checkpoints	1" << endl;
	outMDF << "1" << endl;
	outMDF << "end_checkpoints" << endl;

	outMDF << "speed_limits" << endl;
	outMDF << "num_speed_limits	1" << endl;
	outMDF << "1	6.21371192	18.6411358" << endl;
	outMDF << "end_speed_limits" << endl;
	outMDF << "end_file" << endl;

	outMDF.close();
	Logger::log() << Logger::Debug << "Circle MDF " << mdf_fname << " succesfully generated!" << Logger::endl;


	// read RNDF file to RNDFData
	RNDFReader rndfreader;
	return rndfreader.loadRNDF(filename);
}

boost::shared_ptr<RNDFData> RNDFGenerator::generateEightRNDF(Vec2 midPoint, flt radius, flt swivel, flt rotAngle, std::string const & fname)
{
	Logger::In in("RNDFGenerator::generateEightRNDF");

	// write out circle RNDF to file
	boost::filesystem::path resource_path(cmake_resource_dir);

	string filename = resource_path.string() + "/" + fname;

	ofstream out(filename.c_str(), ios::out & ios::binary & ios::trunc);

	if (!out) {
		Logger::log() << Logger::Error << "Could not write out file to " << filename << Logger::endl;
		return boost::shared_ptr<RNDFData>();
	}


	// Eight template points
	vector<Vec2> circle1, circle2;
	flt alpha = swivel * D2R;
	flt beta = (90 - swivel) * D2R;
	flt l = radius * sin(alpha);
	flt m = l * tan(alpha);
	flt t = sqrt(m * m + l * l);
	flt d = m + radius * cos(alpha) - radius;

	// initial straight segment (unit sampled)
// 	for(flt i=0; i > -m; i-=mSampleSizeUnit.get()) {
// 		temppoints.push_back( Vec2( i , (l/-m)*i ) );
// 	}

	// circle 1 segment (degree sampled)
	for (flt i = swivel; i < 360 - swivel; i += mSampleSizeDegree.get()) {
		circle1.push_back(Vec2(radius * cos(i * D2R) - radius - d , radius * sin(i * D2R)));
	}

	// middle straight segment (unit sampled)
// 	for(flt i=-m; i < m; i+=mSampleSizeUnit.get()) {
// 		temppoints.push_back( Vec2( i , l/m*i ) );
// 	}

	// circle 2 segment (degree sampled)
	for (flt i = 540 - swivel; i > 180 + swivel; i -= mSampleSizeDegree.get()) {
		circle2.push_back(Vec2(radius * cos(i * D2R) + radius + d , radius * sin(i * D2R)));
	}

	// end straight segment (unit sampled)
// 	for(flt i=m; i > 0; i-=mSampleSizeUnit.get()) {
// 		temppoints.push_back( Vec2( i , -l/m*i ) );
// 	}

	// rotate
	for (size_t i = 0; i < circle1.size(); ++i) {
		flt x = circle1[i][0];
		flt y = circle1[i][1];
		circle1[i][0] = x * cos(rotAngle * D2R) - y * sin(rotAngle * D2R);
		circle1[i][1] = x * sin(rotAngle * D2R) + y * cos(rotAngle * D2R);
	}

	for (size_t i = 0; i < circle2.size(); ++i) {
		flt x = circle2[i][0];
		flt y = circle2[i][1];
		circle2[i][0] = x * cos(rotAngle * D2R) - y * sin(rotAngle * D2R);
		circle2[i][1] = x * sin(rotAngle * D2R) + y * cos(rotAngle * D2R);
	}

	vector<Vec2> waypoints1, waypoints2;

	// use formula with earth's average meridional radius M = 6,367,449 m
	int M = 6367449, a = 6378137, b = 6356752;//FIXME there was a .3, but b is an integer;
	flt phi = midPoint[0] * D2R;
	flt x = PI / 180.0 * cos(phi) * sqrt((pow(a, 4) * pow(cos(phi), 2) + pow(b, 4) * pow(sin(phi), 2)) / (pow(a * cos(phi), 2) + pow(b * sin(phi), 2)));

	for (size_t i = 0; i < circle1.size(); ++i) {
		Vec2 eightPoint(circle1[i][1] / x, circle1[i][0] / x / cos(phi));
		waypoints1.push_back(midPoint + eightPoint);
	}

	for (size_t i = 0; i < circle2.size(); ++i) {
		Vec2 eightPoint(circle2[i][1] / x, circle2[i][0] / x / cos(phi));
		waypoints2.push_back(midPoint + eightPoint);
	}

	out << "RNDF_name	TestEight" << endl;
	out << "num_segments	2" << endl;
	out << "num_zones	0" << endl;
	out << "format_version	1.0" << endl;
	date today(day_clock::local_day());
	out << "creation_date	" << today.month().as_number() << "/" << today.day() << "/" << today.year() << endl << endl;

	out << "segment	1" << endl;
	out << "num_lanes	1" << endl;
	out << "segment_name	Eight1" << endl << endl;

	out << "lane	1.1" << endl;
	out << "num_waypoints	" << waypoints1.size() << endl;
	out << "lane_width	12" << endl;
	out << "left_boundary	solid_white" << endl;
	out << "right_boundary	solid_white" << endl;
	out << "checkpoint	1.1.1	1" << endl;
	out << "checkpoint	1.1." << waypoints1.size() << "	2" << endl;
	out << "exit	1.1." << waypoints1.size() << "	2.1.1" << endl;

	out.precision(10);

	for (size_t i = 0; i < waypoints1.size(); ++i) {
		out << "1.1." << (i + 1) << " " << waypoints1[i] << endl;
	}

	out << "end_lane" << endl;
	out << "end_segment" << endl << endl;

	out << "segment	2" << endl;
	out << "num_lanes	1" << endl;
	out << "segment_name	Eight2" << endl << endl;

	out << "lane	2.1" << endl;
	out << "num_waypoints	" << waypoints2.size() << endl;
	out << "lane_width	12" << endl;
	out << "left_boundary	solid_white" << endl;
	out << "right_boundary	solid_white" << endl;
	out << "checkpoint	2.1.1	3" << endl;
	out << "checkpoint	2.1." << waypoints2.size() << "	4" << endl;
	out << "exit	2.1." << waypoints2.size() << "	1.1.1" << endl;

	out.precision(10);

	for (size_t i = 0; i < waypoints2.size(); ++i) {
		out << "2.1." << (i + 1) << " " << waypoints2[i] << endl;
	}

	out << "end_lane" << endl << endl;

	out << "end_segment" << endl;
	out << "end_file" << endl;

	out.close();

	Logger::log() << Logger::Debug << "Eight RNDF " << fname << " succesfully generated!" << Logger::endl;



	// generate MDF
	string mdf_fname = fname;
	size_t result = mdf_fname.find_last_of('.');

	if (std::string::npos != result) {
		mdf_fname.erase(result);
	}

	mdf_fname.append(".mdf");

	string filenameMDF = resource_path.string() + "/" + mdf_fname;

	ofstream outMDF(filenameMDF.c_str(), ios::out & ios::binary & ios::trunc);

	boost::filesystem::path rndf_fp(filename);
	string rndf_f(rndf_fp.filename().c_str());
	result = rndf_f.find_last_of('.');

	if (std::string::npos != result) {
		rndf_f.erase(result);
	}

	if (!outMDF) {
		Logger::log() << Logger::Error << "Could not write out file to " << mdf_fname << Logger::endl;
		return boost::shared_ptr<RNDFData>();
	}

	outMDF << "MDF_name	TestEight" << endl;
	outMDF << "RNDF	" << rndf_f << endl;
	outMDF << "format_version	1.0" << endl;
	outMDF << "creation_date	" << today.month().as_number() << "/" << today.day() << "/" << today.year() << endl;

	outMDF << "checkpoints" << endl;
	outMDF << "num_checkpoints	5" << endl;
	outMDF << "1" << endl;
	outMDF << "2" << endl;
	outMDF << "3" << endl;
	outMDF << "4" << endl;
	outMDF << "1" << endl;
	outMDF << "end_checkpoints" << endl;

	outMDF << "speed_limits" << endl;
	outMDF << "num_speed_limits	1" << endl;
	outMDF << "1	6.21371192	18.6411358" << endl;
	outMDF << "end_speed_limits" << endl;
	outMDF << "end_file" << endl;

	outMDF.close();
	Logger::log() << Logger::Debug << "Eight MDF " << mdf_fname << " succesfully generated!" << Logger::endl;


	// read RNDF file to RNDFData
	RNDFReader rndfreader;
	return rndfreader.loadRNDF(filename);
}



boost::shared_ptr<RNDFData> RNDFGenerator::generateMultilaneRNDF(Vec2 midPoint, flt segmentLength, unsigned int lanes, std::string const & fname)
{
	Logger::In in("RNDFGenerator::generateMultilaneRNDF");

	// write out multilane RNDF to file
	boost::filesystem::path resource_path(cmake_resource_dir);

	string filename = resource_path.string() + "/" + fname;

	ofstream out(filename.c_str(), ios::out & ios::binary & ios::trunc);

	if (!out) {
		Logger::log() << Logger::Error << "Could not write out file to " << filename << Logger::endl;
		return boost::shared_ptr<RNDFData>();
	}

	// multilane template points (divided in segments)
	/*	   1     2
	 *	  ---------
	 *	3|   4|   5|
	 *     6     7
	 *	  ---------
	 *	8|   9|  10|
	 *	  ---------
	 *     11    12
	 */
	vector<vector<vector<Vec2> > > waypoints;

	// use formula with earth's average meridional radius M = 6,367,449 m
	int M = 6367449, a = 6378137, b = 6356752;//FIXME there was a .3, but b is an integer;
	flt phi = midPoint[0] * D2R;
	flt x = PI / 180.0 * cos(phi) * sqrt((pow(a, 4) * pow(cos(phi), 2) + pow(b, 4) * pow(sin(phi), 2)) / (pow(a * cos(phi), 2) + pow(b * sin(phi), 2)));

	flt laneWidth = 12 * FT_2_M * cos(phi);
	flt offset = lanes * laneWidth * 1.5f;

	// create waypoints for segment 1
	vector<vector<Vec2> > segment;

	for (unsigned int j = 0; j < lanes; j++) {
		vector<Vec2> forward;

		for (flt i = 0; i <= segmentLength; i += mSampleSizeUnit.get()) {
			forward.push_back(Vec2((j + 0.5f)*laneWidth + segmentLength + 2 * offset, -i - offset));
		}

		segment.push_back(forward);
	}

	for (unsigned int j = 0; j < lanes; j++) {
		vector<Vec2> backward;

		for (flt i = 0; i <= segmentLength; i += mSampleSizeUnit.get()) {
			backward.push_back(Vec2(-(j + 0.5f)*laneWidth + segmentLength + 2 * offset, -(segmentLength - i) - offset));
		}

		segment.push_back(backward);
	}

	waypoints.push_back(segment);

	// create waypoints for segment 2
	segment.clear();

	for (unsigned int j = 0; j < lanes; j++) {
		vector<Vec2> forward;

		for (flt i = 0; i <= segmentLength; i += mSampleSizeUnit.get()) {
			forward.push_back(Vec2((j + 0.5f)*laneWidth + segmentLength + 2 * offset, segmentLength - i + offset));
		}

		segment.push_back(forward);
	}

	for (unsigned int j = 0; j < lanes; j++) {
		vector<Vec2> backward;

		for (flt i = 0; i <= segmentLength; i += mSampleSizeUnit.get()) {
			backward.push_back(Vec2(-(j + 0.5f)*laneWidth + segmentLength + 2 * offset, i + offset));
		}

		segment.push_back(backward);
	}

	waypoints.push_back(segment);

	// create waypoints for segment 3
	segment.clear();

	for (unsigned int j = 0; j < lanes; j++) {
		vector<Vec2> forward;

		for (flt i = 0; i <= segmentLength; i += mSampleSizeUnit.get()) {
			forward.push_back(Vec2(i + offset, (j + 0.5f)*laneWidth - segmentLength - 2 * offset));
		}

		segment.push_back(forward);
	}

	for (unsigned int j = 0; j < lanes; j++) {
		vector<Vec2> backward;

		for (flt i = 0; i <= segmentLength; i += mSampleSizeUnit.get()) {
			backward.push_back(Vec2(segmentLength - i + offset, -(j + 0.5f)*laneWidth - segmentLength - 2 * offset));
		}

		segment.push_back(backward);
	}

	waypoints.push_back(segment);

	// create waypoints for segment 4
	segment.clear();

	for (unsigned int j = 0; j < lanes; j++) {
		vector<Vec2> forward;

		for (flt i = 0; i <= segmentLength; i += mSampleSizeUnit.get()) {
			forward.push_back(Vec2(i + offset, (j + 0.5f)*laneWidth));
		}

		segment.push_back(forward);
	}

	for (unsigned int j = 0; j < lanes; j++) {
		vector<Vec2> backward;

		for (flt i = 0; i <= segmentLength; i += mSampleSizeUnit.get()) {
			backward.push_back(Vec2(segmentLength - i + offset, -(j + 0.5f)*laneWidth));
		}

		segment.push_back(backward);
	}

	waypoints.push_back(segment);

	// create waypoints for segment 5
	segment.clear();

	for (unsigned int j = 0; j < lanes; j++) {
		vector<Vec2> forward;

		for (flt i = 0; i <= segmentLength; i += mSampleSizeUnit.get()) {
			forward.push_back(Vec2(i + offset, (j + 0.5f)*laneWidth + segmentLength + 2 * offset));
		}

		segment.push_back(forward);
	}

	for (unsigned int j = 0; j < lanes; j++) {
		vector<Vec2> backward;

		for (flt i = 0; i <= segmentLength; i += mSampleSizeUnit.get()) {
			backward.push_back(Vec2(segmentLength - i + offset, -(j + 0.5f)*laneWidth + segmentLength + 2 * offset));
		}

		segment.push_back(backward);
	}

	waypoints.push_back(segment);

	// create waypoints for segment 6
	segment.clear();

	for (unsigned int j = 0; j < lanes; j++) {
		vector<Vec2> forward;

		for (flt i = 0; i <= segmentLength; i += mSampleSizeUnit.get()) {
			forward.push_back(Vec2((j + 0.5f)*laneWidth, -i - offset));
		}

		segment.push_back(forward);
	}

	for (unsigned int j = 0; j < lanes; j++) {
		vector<Vec2> backward;

		for (flt i = 0; i <= segmentLength; i += mSampleSizeUnit.get()) {
			backward.push_back(Vec2(-(j + 0.5f)*laneWidth, -(segmentLength - i) - offset));
		}

		segment.push_back(backward);
	}

	waypoints.push_back(segment);

	// create waypoints for segment 7
	segment.clear();

	for (unsigned int j = 0; j < lanes; j++) {
		vector<Vec2> forward;

		for (flt i = 0; i <= segmentLength; i += mSampleSizeUnit.get()) {
			forward.push_back(Vec2((j + 0.5f)*laneWidth, segmentLength - i + offset));
		}

		segment.push_back(forward);
	}

	for (unsigned int j = 0; j < lanes; j++) {
		vector<Vec2> backward;

		for (flt i = 0; i <= segmentLength; i += mSampleSizeUnit.get()) {
			backward.push_back(Vec2(-(j + 0.5f)*laneWidth, i + offset));
		}

		segment.push_back(backward);
	}

	waypoints.push_back(segment);

	// create waypoints for segment 8
	segment.clear();

	for (unsigned int j = 0; j < lanes; j++) {
		vector<Vec2> forward;

		for (flt i = 0; i <= segmentLength; i += mSampleSizeUnit.get()) {
			forward.push_back(Vec2(-(segmentLength - i) - offset, (j + 0.5f)*laneWidth - segmentLength - 2 * offset));
		}

		segment.push_back(forward);
	}

	for (unsigned int j = 0; j < lanes; j++) {
		vector<Vec2> backward;

		for (flt i = 0; i <= segmentLength; i += mSampleSizeUnit.get()) {
			backward.push_back(Vec2(-i - offset, -(j + 0.5f)*laneWidth - segmentLength - 2 * offset));
		}

		segment.push_back(backward);
	}

	waypoints.push_back(segment);

	// create waypoints for segment 9
	segment.clear();

	for (unsigned int j = 0; j < lanes; j++) {
		vector<Vec2> forward;

		for (flt i = 0; i <= segmentLength; i += mSampleSizeUnit.get()) {
			forward.push_back(Vec2(-(segmentLength - i) - offset, (j + 0.5f)*laneWidth));
		}

		segment.push_back(forward);
	}

	for (unsigned int j = 0; j < lanes; j++) {
		vector<Vec2> backward;

		for (flt i = 0; i <= segmentLength; i += mSampleSizeUnit.get()) {
			backward.push_back(Vec2(-i - offset, -(j + 0.5f)*laneWidth));
		}

		segment.push_back(backward);
	}

	waypoints.push_back(segment);

	// create waypoints for segment 10
	segment.clear();

	for (unsigned int j = 0; j < lanes; j++) {
		vector<Vec2> forward;

		for (flt i = 0; i <= segmentLength; i += mSampleSizeUnit.get()) {
			forward.push_back(Vec2(-(segmentLength - i) - offset, (j + 0.5f)*laneWidth + segmentLength + 2 * offset));
		}

		segment.push_back(forward);
	}

	for (unsigned int j = 0; j < lanes; j++) {
		vector<Vec2> backward;

		for (flt i = 0; i <= segmentLength; i += mSampleSizeUnit.get()) {
			backward.push_back(Vec2(-i - offset, -(j + 0.5f)*laneWidth + segmentLength + 2 * offset));
		}

		segment.push_back(backward);
	}

	waypoints.push_back(segment);

	// create waypoints for segment 11
	segment.clear();

	for (unsigned int j = 0; j < lanes; j++) {
		vector<Vec2> forward;

		for (flt i = 0; i <= segmentLength; i += mSampleSizeUnit.get()) {
			forward.push_back(Vec2((j + 0.5f)*laneWidth - segmentLength - 2 * offset, -i - offset));
		}

		segment.push_back(forward);
	}

	for (unsigned int j = 0; j < lanes; j++) {
		vector<Vec2> backward;

		for (flt i = 0; i <= segmentLength; i += mSampleSizeUnit.get()) {
			backward.push_back(Vec2(-(j + 0.5f)*laneWidth - segmentLength - 2 * offset, -(segmentLength - i) - offset));
		}

		segment.push_back(backward);
	}

	waypoints.push_back(segment);

	// create waypoints for segment 12
	segment.clear();

	for (unsigned int j = 0; j < lanes; j++) {
		vector<Vec2> forward;

		for (flt i = 0; i <= segmentLength; i += mSampleSizeUnit.get()) {
			forward.push_back(Vec2((j + 0.5f)*laneWidth - segmentLength - 2 * offset, segmentLength - i + offset));
		}

		segment.push_back(forward);
	}

	for (unsigned int j = 0; j < lanes; j++) {
		vector<Vec2> backward;

		for (flt i = 0; i <= segmentLength; i += mSampleSizeUnit.get()) {
			backward.push_back(Vec2(-(j + 0.5f)*laneWidth - segmentLength - 2 * offset, i + offset));
		}

		segment.push_back(backward);
	}

	waypoints.push_back(segment);

	//rotate
// 	flt w = 180.f * D2R;
// 	for(int i=0; i < waypoints.size(); ++i) {
// 		for(int j=0; j < waypoints[i].size(); ++j) {
// 			for(int k=0; k < waypoints[i][j].size(); ++k) {
// 				flt u = waypoints[i][j][k][0];
// 				flt v = waypoints[i][j][k][1];
// 				waypoints[i][j][k][0] = u*cos(w) - v*sin(w);
// 				waypoints[i][j][k][1] = u*sin(w) + v*cos(w);
// 			}
// 		}
// 	}

	//transform coordinates from plain origin surface to earth surface
	for (size_t i = 0; i < waypoints.size(); ++i) {
		for (size_t j = 0; j < waypoints[i].size(); ++j) {
			for (size_t k = 0; k < waypoints[i][j].size(); ++k) {
				waypoints[i][j][k][0] = midPoint[0] + waypoints[i][j][k][0] / x;
				waypoints[i][j][k][1] = midPoint[1] + waypoints[i][j][k][1] / x / cos(phi);
			}
		}
	}

	out << "RNDF_name	TestMultilane" << endl;
	out << "num_segments	" << waypoints.size() << endl;
	out << "num_zones	0" << endl;
	out << "format_version	1.0" << endl;
	date today(day_clock::local_day());
	out << "creation_date	" << today.month().as_number() << "/" << today.day() << "/" << today.year() << endl << endl;

	size_t cp = 0;

	for (size_t s = 0; s < waypoints.size(); ++s) {
		out << "segment	" << (s + 1) << endl;
		out << "num_lanes	" << waypoints[s].size() << endl;
		out << "segment_name	segment" << (s + 1) << endl;

		for (size_t l = 0; l < waypoints[s].size(); ++l) {
			out << "lane	" << (s + 1) << "." << (l + 1) << endl;
			out << "num_waypoints	" << waypoints[s][l].size() << endl;
			out << "lane_width	12" << endl;
			out << "left_boundary	solid_white" << endl;
			out << "right_boundary	solid_white" << endl;

			out << "checkpoint	" << (s + 1) << "." << (l + 1) << "." << waypoints[s][l].size() / 2 << "	" << ++cp << endl;

			//define exit/entry points
			size_t endPoint = waypoints[s][l].size();

			switch (s + 1) {
			case 1:

				if (l < lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "3." << (l + 1 + lanes) << ".1" << endl;    //1->3
				}

				if (l >= lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "2." << (l + 1) << ".1" << endl;    //1->2
				}

				if (l >= lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "4." << (l + 1) << ".1" << endl;    //1->4
				}

				break;

			case 2:

				if (l < lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "1." << (l + 1) << ".1" << endl;    //2->1
				}

				if (l < lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "4." << (l + 1 + lanes) << ".1" << endl;    //2->4
				}

				if (l >= lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "5." << (l + 1) << ".1" << endl;    //2->5
				}

				break;

			case 3:

				if (l < lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "1." << (l + 1 + lanes) << ".1" << endl;    //3->1
				}

				if (l >= lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "6." << (l + 1) << ".1" << endl;    //3->6
				}

				if (l >= lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "8." << (l + 1) << ".1" << endl;    //3->8
				}

				break;

			case 4:

				if (l < lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "1." << (l + 1) << ".1" << endl;    //4->1
				}

				if (l < lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "2." << (l + 1 + lanes) << ".1" << endl;    //4->2
				}

				if (l >= lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "6." << (l + 1 - lanes) << ".1" << endl;    //4->6
				}

				if (l >= lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "7." << (l + 1) << ".1" << endl;    //4->7
				}

				if (l >= lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "9." << (l + 1) << ".1" << endl;    //4->9
				}

				break;

			case 5:

				if (l < lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "2." << (l + 1) << ".1" << endl;    //5->2
				}

				if (l >= lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "7." << (l + 1 - lanes) << ".1" << endl;    //5->7
				}

				if (l >= lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "10." << (l + 1) << ".1" << endl;    //5->10
				}

				break;

			case 6:

				if (l < lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "3." << (l + 1) << ".1" << endl;    //6->3
				}

				if (l < lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "8." << (l + 1 + lanes) << ".1" << endl;    //6->8
				}

				if (l >= lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "4." << (l + 1 - lanes) << ".1" << endl;    //6->4
				}

				if (l >= lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "7." << (l + 1) << ".1" << endl;    //6->7
				}

				if (l >= lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "9." << (l + 1) << ".1" << endl;    //6->9
				}

				break;

			case 7:

				if (l < lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "4." << (l + 1) << ".1" << endl;    //7->4
				}

				if (l < lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "6." << (l + 1) << ".1" << endl;    //7->6
				}

				if (l < lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "9." << (l + 1 + lanes) << ".1" << endl;    //7->9
				}

				if (l >= lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "5." << (l + 1 - lanes) << ".1" << endl;    //7->5
				}

				if (l >= lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "10." << (l + 1) << ".1" << endl;    //7->10
				}

				break;

			case 8:

				if (l < lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "3." << (l + 1) << ".1" << endl;    //8->3
				}

				if (l < lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "6." << (l + 1 + lanes) << ".1" << endl;    //8->6
				}

				if (l >= lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "11." << (l + 1) << ".1" << endl;    //8->11
				}

				break;

			case 9:

				if (l < lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "4." << (l + 1) << ".1" << endl;    //9->4
				}

				if (l < lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "6." << (l + 1) << ".1" << endl;    //9->6
				}

				if (l < lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "7." << (l + 1 + lanes) << ".1" << endl;    //9->7
				}

				if (l >= lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "11." << (l + 1 - lanes) << ".1" << endl;    //9->11
				}

				if (l >= lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "12." << (l + 1) << ".1" << endl;    //9->12
				}

				break;

			case 10:

				if (l < lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "5." << (l + 1) << ".1" << endl;    //10->5
				}

				if (l < lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "7." << (l + 1) << ".1" << endl;    //10->7
				}

				if (l >= lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "12." << (l + 1 - lanes) << ".1" << endl;    //10->12
				}

				break;

			case 11:

				if (l < lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "8." << (l + 1) << ".1" << endl;    //11->8
				}

				if (l >= lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "9." << (l + 1 - lanes) << ".1" << endl;    //11->9
				}

				if (l >= lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "12." << (l + 1) << ".1" << endl;    //11->12
				}

				break;

			case 12:

				if (l < lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "9." << (l + 1) << ".1" << endl;    //12->9
				}

				if (l < lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "11." << (l + 1) << ".1" << endl;    //12->11
				}

				if (l >= lanes) {
					out << "exit	" << (s + 1) << "." << (l + 1) << "." << endPoint << "	" << "10." << (l + 1 - lanes) << ".1" << endl;    //12->10
				}

				break;
			}



			out.precision(10);

			for (size_t i = 0; i < waypoints[s][l].size(); ++i) {
				out << (s + 1) << "." << (l + 1) << "." << (i + 1) << " " << waypoints[s][l][i] << endl;
			}

			out << "end_lane" << endl << endl;
		}

		out << "end_segment" << endl << endl;
	}

	out << "end_file" << endl;

	out.close();
	Logger::log() << Logger::Debug << "Multilane RNDF " << fname << " succesfully generated!" << Logger::endl;


	// generate MDF
	string mdf_fname = fname;
	size_t result = mdf_fname.find_last_of('.');

	if (std::string::npos != result) {
		mdf_fname.erase(result);
	}

	mdf_fname.append(".mdf");

	string filenameMDF = resource_path.string() + "/" + mdf_fname;

	ofstream outMDF(filenameMDF.c_str(), ios::out & ios::binary & ios::trunc);

	boost::filesystem::path rndf_fp(filename);
	string rndf_f(rndf_fp.filename().c_str());
	result = rndf_f.find_last_of('.');

	if (std::string::npos != result) {
		rndf_f.erase(result);
	}

	if (!outMDF) {
		Logger::log() << Logger::Error << "Could not write out file to " << mdf_fname << Logger::endl;
		return boost::shared_ptr<RNDFData>();
	}

	outMDF << "MDF_name	TestMultilane" << endl;
	outMDF << "RNDF	" << rndf_f << endl;
	outMDF << "format_version	1.0" << endl;
	outMDF << "creation_date	" << today.month().as_number() << "/" << today.day() << "/" << today.year() << endl;

	outMDF << "checkpoints" << endl;
	outMDF << "num_checkpoints	18" << endl;
	outMDF << "1" << endl;
	outMDF << "21" << endl;
	outMDF << "45" << endl;
	outMDF << "69" << endl;
	outMDF << "81" << endl;
	outMDF << "61" << endl;
	outMDF << "45" << endl;
	outMDF << "25" << endl;
	outMDF << "13" << endl;
	outMDF << "37" << endl;
	outMDF << "49" << endl;
	outMDF << "69" << endl;
	outMDF << "89" << endl;
	outMDF << "73" << endl;
	outMDF << "49" << endl;
	outMDF << "49" << endl;
	outMDF << "25" << endl;
	outMDF << "1" << endl;
	outMDF << "end_checkpoints" << endl;

	outMDF << "speed_limits" << endl;
	outMDF << "num_speed_limits	" << waypoints.size() << endl;

	for (unsigned int s = 0; s < waypoints.size(); ++s) {
		outMDF << (s + 1) << "	6.21371192	18.6411358" << endl;
	}

	outMDF << "end_speed_limits" << endl;
	outMDF << "end_file" << endl;


	outMDF.close();
	Logger::log() << Logger::Debug << "Multilane MDF " << mdf_fname << " succesfully generated!" << Logger::endl;


	// read RNDF file to RNDFData
	RNDFReader rndfreader;
	return rndfreader.loadRNDF(filename);
}

}
}
}
}
}
