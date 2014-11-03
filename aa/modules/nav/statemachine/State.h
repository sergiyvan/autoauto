#pragma once

#include <iostream>
#include <fstream>
#include <boost/any.hpp>


namespace aa
{
namespace modules
{
namespace nav
{

namespace statemachine
{


class State
{
public:

	State(int sid, int pid, std::string n, int default_cid);
	State();
	~State();

	State(State const & rhs);
	State & operator=(State const & rhs);

	bool hasMeta() const;
	boost::any getMeta() const;
	void setMeta(boost::any m);
	void clearMeta();

	int id;
	int parent_id;
	std::string name;
	int last_child_id;

	friend std::ostream & operator<<(std::ostream & os, State const & s);

private:
	boost::any meta;
};

}

}

}

}


