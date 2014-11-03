#pragma once
#include "Xing.h"
#include <boost/noncopyable.hpp>

class XingInfo
	: boost::noncopyable
{
public:
	typedef Xing::Vec2 Vec2;
	typedef std::map< ::aa::modules::models::rndf::edge_descr, boost::shared_ptr<Xing>, ::aa::modules::models::rndf::EdgeOrder > map_type;
	typedef std::multimap< ::aa::modules::models::rndf::vertex_descr, boost::shared_ptr<Xing> > mmap_type;

	XingInfo();
	~XingInfo();

	bool computeXings();
	boost::shared_ptr<Xing> getXing(::aa::modules::models::rndf::edge_descr const & xingEdge) const;
	bool getNearestXing(boost::shared_ptr<Xing> & xingSh_ptr, Vec2 const  & pos) const;

	map_type const  & getXingMap() const;

	mmap_type const & vertexCrossing() const {
		return mVertexXing;
	}


private:

	::aa::modules::models::rndf::RNDFGraph & mRNDFGraph;
	map_type mXingMapShared;
	mmap_type mVertexXing;
};


