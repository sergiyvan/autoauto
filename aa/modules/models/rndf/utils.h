#pragma once

#include <list>
#include <boost/tuple/tuple.hpp>
#include <math/Types.h>
#include <aa/modules/models/rndf/RndfGraph.h>

namespace aa
{
namespace modules
{
namespace models
{

namespace rndf
{


edge_descr findLanes(aa::modules::models::rndf::RNDFGraph const & rndfGraph, math::Vec2 const & carPos, std::list<boost::tuple<edge_descr, math::flt, math::Vec2, EdgeData> > & lanes);

}

}

}

}

