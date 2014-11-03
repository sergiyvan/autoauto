#pragma once
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
#include <util/SerialiseMat.h>
#include <util/SerialiseTuple.h>

typedef patterns::Singleton<RNDFGraph> theRndfGraph;

namespace boost
{
namespace serialization
{
template<typename Archive>
void serialize(Archive &, boost::any &,
			   const unsigned int version)
{
	// We cannot serialize boost::any
}

}
}

BOOST_SERIALIZATION_SPLIT_FREE(edge_descr);

namespace boost
{
namespace serialization
{
template<typename Archive>
void save(Archive & ar, edge_descr const & e,
		  const unsigned int version)
{
	aGraph const & g = theRndfGraph::instance().getBoostGraph();
	unsigned int srcidx = get(vertex_index_t(), g, source(e, g));
	unsigned int dstidx = get(vertex_index_t(), g, target(e, g));
	ar << srcidx << dstidx;
}

template<typename Archive>
void load(Archive & ar, edge_descr & e,
		  const unsigned int version)
{
	unsigned int srcidx;
	unsigned int dstidx;
	ar >> srcidx >> dstidx;
	aGraph const & g = theRndfGraph::instance().getBoostGraph();
	bool success(false);
	boost::tie(e, success) = edge(vertex(srcidx, g), vertex(dstidx, g), g);
	assert(success);
}

}
}

