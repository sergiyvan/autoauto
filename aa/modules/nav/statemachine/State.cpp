#include "State.h"

namespace aa
{
namespace modules
{
namespace nav
{

namespace statemachine
{

State::State(int sid, int pid, std::string n, int default_cid)
	: id(sid)
	, parent_id(pid)
	, name(n)
	, last_child_id(default_cid)
{
	clearMeta();
};

State::State()
{
}


State::~State()
{
}

State::State(State const & rhs)
	: id(rhs.id)
	, parent_id(rhs.parent_id)
	, name(rhs.name)
	, last_child_id(rhs.last_child_id)

	, meta(rhs.meta)
{
}

State & State::operator=(State const & rhs)
{
	if (this == &rhs) {
		return *this;
	}

	id = rhs.id;
	parent_id = rhs.parent_id;
	name = rhs.name;
	last_child_id = rhs.last_child_id;

	meta = rhs.meta;

	return *this;
}


bool State::hasMeta() const
{
	return !meta.empty();
}

boost::any State::getMeta() const
{
	return meta;
}

void State::setMeta(boost::any m)
{
	meta = m;
}

void State::clearMeta()
{
	meta = boost::any();
}

std::ostream & operator<< (std::ostream & os, const State & s)
{
	return os 	<< s.name;
}

}


}


}


}



