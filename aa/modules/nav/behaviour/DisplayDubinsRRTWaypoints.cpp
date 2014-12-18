/*!
 * \file DisplayDubinsRRTWaypoints.cpp
 * \brief TODO Brief description of DisplayDubinsRRTWaypoints.
 */

#include "DisplayDubinsRRTWaypoints.h"

#include <util/TaskContextFactory.h>
#include <util/OrocosHelperFunctions.h>
#include <osg/Geode>
#include <osg/Array>
#include <osg/Geometry>
#include <osg/Point>


namespace aa
{
namespace modules
{
namespace nav
{
namespace behaviour
{

REGISTERTASKCONTEXT(DisplayDubinsRRTWaypoints);

//////////////////////////////////////////////////////
// Constructor:

DisplayDubinsRRTWaypoints::DisplayDubinsRRTWaypoints(std::string const & name)
	: gui::Painter3DTask(name)
	// TODO constructor
	// Read ports:
    , mWaypointsIn("WaypointsIn")
	// Write ports:

	// Properties:
    , mPointColor(::math::Vec3(1.0, 0.0, 0.0))
    , mLineColor(::math::Vec3(0.0, 1.0, 0.0))
    , mLineColor2(::math::Vec3(0.0, 0.0, 1.0))
    , mLineColor3(::math::Vec3(0.0, 1.0, 1.0))
    , mTransparency(0.5f)
    , mDrawLines(true)
	// Attributes:
	, mVertices(new osg::Vec3Array)
{
	// Read ports:

	// Event Ports:
    ports()->addPort(mWaypointsIn);
	// Non-event ports:

	// Write ports

	// Properties:
    addProperty("PointColor", mPointColor);
    addProperty("LineColor1", mLineColor).doc("LRL / RLR");
    addProperty("LineColor2", mLineColor).doc("LRL / RLR");
    addProperty("LineColor3", mLineColor).doc("LRL / RLR");
    addProperty("Transparency", mTransparency);
    addProperty("DrawLines", mDrawLines);

	// Attributes:

	// Methods:

	// Commands:

}

//////////////////////////////////////////////////////
// Destructor:

DisplayDubinsRRTWaypoints::~DisplayDubinsRRTWaypoints()
{
}

//////////////////////////////////////////////////////
// TaskContext interface

bool DisplayDubinsRRTWaypoints::startHook()
{
	return true;
}

void DisplayDubinsRRTWaypoints::updateHook()
{
}

void DisplayDubinsRRTWaypoints::stopHook()
{
}


////////////////////////////////////////////////////////
// Protected functions:

////////////////////////////////////////////////////////
// Private functions:


void DisplayDubinsRRTWaypoints::init3D(SceneNodePtr root)
{
    mLineGeode = new osg::Geode;
    root->addChild(mLineGeode);
    osg::StateSet * stateSet2 = mLineGeode->getOrCreateStateSet();
    stateSet2->setMode(GL_BLEND, osg::StateAttribute::ON);
    stateSet2->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    stateSet2->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    mLineGeode2 = new osg::Geode;
    root->addChild(mLineGeode2);
    stateSet2 = mLineGeode2->getOrCreateStateSet();
    stateSet2->setMode(GL_BLEND, osg::StateAttribute::ON);
    stateSet2->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    stateSet2->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    mLineGeode3 = new osg::Geode;
    root->addChild(mLineGeode3);
    stateSet2 = mLineGeode3->getOrCreateStateSet();
    stateSet2->setMode(GL_BLEND, osg::StateAttribute::ON);
    stateSet2->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    stateSet2->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    mPointGeode = new osg::Geode;
    root->addChild(mPointGeode);
    mPointGeode->setUpdateCallback(this);
	mGeometry = new osg::Geometry();
    mPointGeode->addDrawable(mGeometry);
	mGeometry->setVertexArray(mVertices);
    mDrawArrayLines = new osg::DrawArrays(osg::PrimitiveSet::POINTS);
	mGeometry->addPrimitiveSet(mDrawArrayLines);
	osg::ref_ptr<osg::Vec4Array> colourArray = new osg::Vec4Array();
    colourArray->push_back(osg::Vec4(mPointColor(0), mPointColor(1), mPointColor(2), mTransparency));
	mGeometry->setColorArray(colourArray.get());
	mGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
    osg::StateSet * stateSet = mPointGeode->getOrCreateStateSet();
	stateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
    stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    stateSet->setAttribute( new osg::Point( 3.0f ), osg::StateAttribute::ON );



}

void DisplayDubinsRRTWaypoints::operator()(osg::Node * node, osg::NodeVisitor * nv)
{
    std::vector<DubinsRRTWaypointPtr> waypoints;
    mWaypointsIn.read(waypoints);


    mLineGeode->removeDrawables(0,mLineGeode->getNumDrawables());
    mLineGeode2->removeDrawables(0,mLineGeode2->getNumDrawables());
    mLineGeode3->removeDrawables(0,mLineGeode3->getNumDrawables());


    if (mDrawLines) {
        //draw connections between rlr/lrl dc intersections
        osg::ref_ptr<osg::Vec3Array> vertices1(new osg::Vec3Array());
        for (int i=0;i<waypoints.size();i++) {
            if (waypoints[i]) {
                DubinsRRTWaypointPtr wp = waypoints[i];
                if (wp->prevWaypoint) {
                    if (wp->dubinsCurve.mType == DubinsType::LRL || wp->dubinsCurve.mType == DubinsType::RLR) {

                        if (wp->distToPrev > wp->dubinsCurve.mD1+wp->dubinsCurve.mD2) {
                            //we have 3 segments
                            math::Vec3 pos1 = wp->dubinsCurve.mStartPos;
                            vertices1->push_back(osg::Vec3(pos1(0), pos1(1), pos1(2)));
                            math::Vec3 dirPos1 = wp->dubinsCurve.mI1Pos;
                            vertices1->push_back(osg::Vec3(dirPos1(0), dirPos1(1), dirPos1(2)));
                            math::Vec3 pos = wp->dubinsCurve.mI1Pos;
                            vertices1->push_back(osg::Vec3(pos(0), pos(1), pos(2)));
                            math::Vec3 dirPos = wp->dubinsCurve.mI2Pos;
                            vertices1->push_back(osg::Vec3(dirPos(0), dirPos(1), dirPos(2)));
                            math::Vec3 pos2 = wp->dubinsCurve.mI2Pos;
                            vertices1->push_back(osg::Vec3(pos2(0), pos2(1), pos2(2)));
                            math::Vec3 dirPos2 = wp->position;
                            vertices1->push_back(osg::Vec3(dirPos2(0), dirPos2(1), dirPos2(2)));
                        } else if (wp->distToPrev > wp->dubinsCurve.mD1) {
                            //we have 2 segments
                            math::Vec3 pos1 = wp->dubinsCurve.mStartPos;
                            vertices1->push_back(osg::Vec3(pos1(0), pos1(1), pos1(2)));
                            math::Vec3 dirPos1 = wp->dubinsCurve.mI1Pos;
                            vertices1->push_back(osg::Vec3(dirPos1(0), dirPos1(1), dirPos1(2)));
                            math::Vec3 pos = wp->dubinsCurve.mI1Pos;
                            vertices1->push_back(osg::Vec3(pos(0), pos(1), pos(2)));
                            math::Vec3 dirPos = wp->position;
                            vertices1->push_back(osg::Vec3(dirPos(0), dirPos(1), dirPos(2)));
                        } else {
                            //we have 1 segments
                            math::Vec3 pos1 = wp->dubinsCurve.mStartPos;
                            vertices1->push_back(osg::Vec3(pos1(0), pos1(1), pos1(2)));
                            math::Vec3 dirPos = wp->position;
                            vertices1->push_back(osg::Vec3(dirPos(0), dirPos(1), dirPos(2)));
                        }
                    }
                }
            }

        }
        osg::ref_ptr<osg::Geometry> geometry1 = new osg::Geometry();
        geometry1->setVertexArray(vertices1);
        geometry1->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, vertices1->size()));
        osg::ref_ptr<osg::Vec4Array> colourArray1 = new osg::Vec4Array();
        colourArray1->push_back(osg::Vec4(mLineColor(0), mLineColor(1), mLineColor(2), mTransparency));
        geometry1->setColorArray(colourArray1.get());
        geometry1->setColorBinding(osg::Geometry::BIND_OVERALL);
        mLineGeode->addDrawable(geometry1);



        //draw connections between rsr/lsl dc intersections
        osg::ref_ptr<osg::Vec3Array> vertices2(new osg::Vec3Array());
        for (int i=0;i<waypoints.size();i++) {
            if (waypoints[i]) {
                DubinsRRTWaypointPtr wp = waypoints[i];
                if (wp->prevWaypoint) {
                    if (wp->dubinsCurve.mType == DubinsType::RSR || wp->dubinsCurve.mType == DubinsType::LSL) {

                        if (wp->distToPrev > wp->dubinsCurve.mD1+wp->dubinsCurve.mD2) {
                            //we have 3 segments
                            math::Vec3 pos1 = wp->dubinsCurve.mStartPos;
                            vertices2->push_back(osg::Vec3(pos1(0), pos1(1), pos1(2)));
                            math::Vec3 dirPos1 = wp->dubinsCurve.mI1Pos;
                            vertices2->push_back(osg::Vec3(dirPos1(0), dirPos1(1), dirPos1(2)));
                            math::Vec3 pos = wp->dubinsCurve.mI1Pos;
                            vertices2->push_back(osg::Vec3(pos(0), pos(1), pos(2)));
                            math::Vec3 dirPos = wp->dubinsCurve.mI2Pos;
                            vertices2->push_back(osg::Vec3(dirPos(0), dirPos(1), dirPos(2)));
                            math::Vec3 pos2 = wp->dubinsCurve.mI2Pos;
                            vertices2->push_back(osg::Vec3(pos2(0), pos2(1), pos2(2)));
                            math::Vec3 dirPos2 = wp->position;
                            vertices2->push_back(osg::Vec3(dirPos2(0), dirPos2(1), dirPos2(2)));
                        } else if (wp->distToPrev > wp->dubinsCurve.mD1) {
                            //we have 2 segments
                            math::Vec3 pos1 = wp->dubinsCurve.mStartPos;
                            vertices2->push_back(osg::Vec3(pos1(0), pos1(1), pos1(2)));
                            math::Vec3 dirPos1 = wp->dubinsCurve.mI1Pos;
                            vertices2->push_back(osg::Vec3(dirPos1(0), dirPos1(1), dirPos1(2)));
                            math::Vec3 pos = wp->dubinsCurve.mI1Pos;
                            vertices2->push_back(osg::Vec3(pos(0), pos(1), pos(2)));
                            math::Vec3 dirPos = wp->position;
                            vertices2->push_back(osg::Vec3(dirPos(0), dirPos(1), dirPos(2)));
                        } else {
                            //we have 1 segments
                            math::Vec3 pos1 = wp->dubinsCurve.mStartPos;
                            vertices2->push_back(osg::Vec3(pos1(0), pos1(1), pos1(2)));
                            math::Vec3 dirPos = wp->position;
                            vertices2->push_back(osg::Vec3(dirPos(0), dirPos(1), dirPos(2)));
                        }
                    }
                }
            }

        }
        osg::ref_ptr<osg::Geometry> geometry2 = new osg::Geometry();
        geometry2->setVertexArray(vertices2);
        geometry2->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, vertices2->size()));
        osg::ref_ptr<osg::Vec4Array> colourArray2 = new osg::Vec4Array();
        colourArray2->push_back(osg::Vec4(mLineColor2(0), mLineColor2(1), mLineColor2(2), mTransparency));
        geometry2->setColorArray(colourArray2.get());
        geometry2->setColorBinding(osg::Geometry::BIND_OVERALL);
        mLineGeode->addDrawable(geometry2);




        //draw connections between rsl/lsr dc intersections
        osg::ref_ptr<osg::Vec3Array> vertices3(new osg::Vec3Array());
        for (int i=0;i<waypoints.size();i++) {
            if (waypoints[i]) {
                DubinsRRTWaypointPtr wp = waypoints[i];
                if (wp->prevWaypoint) {
                    if (wp->dubinsCurve.mType == DubinsType::LSR || wp->dubinsCurve.mType == DubinsType::RSL) {

                        if (wp->distToPrev > wp->dubinsCurve.mD1+wp->dubinsCurve.mD2) {
                            //we have 3 segments
                            math::Vec3 pos1 = wp->dubinsCurve.mStartPos;
                            vertices3->push_back(osg::Vec3(pos1(0), pos1(1), pos1(2)));
                            math::Vec3 dirPos1 = wp->dubinsCurve.mI1Pos;
                            vertices3->push_back(osg::Vec3(dirPos1(0), dirPos1(1), dirPos1(2)));
                            math::Vec3 pos = wp->dubinsCurve.mI1Pos;
                            vertices3->push_back(osg::Vec3(pos(0), pos(1), pos(2)));
                            math::Vec3 dirPos = wp->dubinsCurve.mI2Pos;
                            vertices3->push_back(osg::Vec3(dirPos(0), dirPos(1), dirPos(2)));
                            math::Vec3 pos2 = wp->dubinsCurve.mI2Pos;
                            vertices3->push_back(osg::Vec3(pos2(0), pos2(1), pos2(2)));
                            math::Vec3 dirPos2 = wp->position;
                            vertices3->push_back(osg::Vec3(dirPos2(0), dirPos2(1), dirPos2(2)));
                        } else if (wp->distToPrev > wp->dubinsCurve.mD1) {
                            //we have 2 segments
                            math::Vec3 pos1 = wp->dubinsCurve.mStartPos;
                            vertices3->push_back(osg::Vec3(pos1(0), pos1(1), pos1(2)));
                            math::Vec3 dirPos1 = wp->dubinsCurve.mI1Pos;
                            vertices3->push_back(osg::Vec3(dirPos1(0), dirPos1(1), dirPos1(2)));
                            math::Vec3 pos = wp->dubinsCurve.mI1Pos;
                            vertices3->push_back(osg::Vec3(pos(0), pos(1), pos(2)));
                            math::Vec3 dirPos = wp->position;
                            vertices3->push_back(osg::Vec3(dirPos(0), dirPos(1), dirPos(2)));
                        } else {
                            //we have 1 segments
                            math::Vec3 pos1 = wp->dubinsCurve.mStartPos;
                            vertices3->push_back(osg::Vec3(pos1(0), pos1(1), pos1(2)));
                            math::Vec3 dirPos = wp->position;
                            vertices3->push_back(osg::Vec3(dirPos(0), dirPos(1), dirPos(2)));
                        }
                    }
                }
            }

        }
        osg::ref_ptr<osg::Geometry> geometry3 = new osg::Geometry();
        geometry3->setVertexArray(vertices3);
        geometry3->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, vertices3->size()));
        osg::ref_ptr<osg::Vec4Array> colourArray3 = new osg::Vec4Array();
        colourArray3->push_back(osg::Vec4(mLineColor3(0), mLineColor3(1), mLineColor3(2), mTransparency));
        geometry3->setColorArray(colourArray3.get());
        geometry3->setColorBinding(osg::Geometry::BIND_OVERALL);
        mLineGeode->addDrawable(geometry3);




    }



    //draw points

    mVertices->clear();
    for (int i=0;i<waypoints.size();i++) {
        if (waypoints[i]) {
            math::Vec3 pos = waypoints[i]->position;
            mVertices->push_back(osg::Vec3(pos(0), pos(1), pos(2)));
        }
    }
    mDrawArrayLines->setFirst(0);
    mDrawArrayLines->setCount(mVertices->size());
    mGeometry->setVertexArray(mVertices);

    //draw short line from point in direction of orientation
    mPointGeode->removeDrawables(0,mPointGeode->getNumDrawables());

    osg::ref_ptr<osg::Vec3Array> verticesP(new osg::Vec3Array());
    for (int i=0;i<waypoints.size();i++) {
        if (waypoints[i]) {
            DubinsRRTWaypointPtr wp = waypoints[i];
            math::Vec3 pos = wp->position;
            verticesP->push_back(osg::Vec3(pos(0), pos(1), pos(2)));
            math::Vec3 dirPos = wp->position+wp->orientation*0.1;
            verticesP->push_back(osg::Vec3(dirPos(0), dirPos(1), dirPos(2)));
        }
    }
    osg::ref_ptr<osg::Geometry> geometryP = new osg::Geometry();
    geometryP->setVertexArray(verticesP);
    geometryP->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, verticesP->size()));
    osg::ref_ptr<osg::Vec4Array> colourArrayP = new osg::Vec4Array();
    colourArrayP->push_back(osg::Vec4(mPointColor(0), mPointColor(1), mPointColor(2), mTransparency));
    geometryP->setColorArray(colourArrayP.get());
    geometryP->setColorBinding(osg::Geometry::BIND_OVERALL);
    mPointGeode->addDrawable(geometryP);



}

}
}
}
}
