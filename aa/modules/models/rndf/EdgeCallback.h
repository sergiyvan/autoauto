#pragma once
#include <osg/Geode>
#include "RndfGraph.h"
#include <boost/function.hpp>
#include <iostream>

typedef std::map< ::aa::modules::models::rndf::edge_descr, bool, ::aa::modules::models::rndf::EdgeOrder> EdgeNotifyList;

class EdgeNodeDataType : public osg::Referenced
{
public:
	EdgeNodeDataType(osg::Node * n, EdgeNotifyList & edgeNotifyList, ::aa::modules::models::rndf::edge_descr edge)
		: mEdgeNotifyList(edgeNotifyList)
		, mEdge(edge)
	{}

	virtual ~EdgeNodeDataType()
	{}

	EdgeNotifyList & getEdgeNotifyList() const {
		return mEdgeNotifyList;
	}
	::aa::modules::models::rndf::edge_descr getEdge() const {
		return mEdge;
	}

protected:
	EdgeNotifyList & mEdgeNotifyList;
	::aa::modules::models::rndf::edge_descr mEdge;
};

class EdgeNodeCallback : public osg::NodeCallback
{
	typedef boost::function2<void, ::aa::modules::models::rndf::edge_descr const &, osg::Node * > edge_update_callback;

public:
	EdgeNodeCallback(edge_update_callback cb)
		: osg::NodeCallback()
		, mEdgeCallback(cb)
	{}

	virtual ~EdgeNodeCallback()
	{}

	virtual void operator()(osg::Node * node, osg::NodeVisitor * nv) {
		osg::ref_ptr<EdgeNodeDataType> edgeNodeData =
			dynamic_cast<EdgeNodeDataType *>(node->getUserData());

		if (edgeNodeData.valid()) {
			EdgeNotifyList & el = edgeNodeData->getEdgeNotifyList();
			::aa::modules::models::rndf::edge_descr e = edgeNodeData->getEdge();
			EdgeNotifyList::iterator it = el.find(e);

			if (it != el.end()) {
				if (it->second) {
					it->second = false;
					mEdgeCallback(it->first, node);
				}
			}

			// May happen, when drawing, while a node has been already removed from the RndfGraph, but not the OpenSceneGraph
// 			assert(it != el.end());
		}

		traverse(node, nv);
	}

protected:
	edge_update_callback mEdgeCallback;
};
