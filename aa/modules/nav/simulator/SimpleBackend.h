#pragma once
#include <string>
#include <math/Types.h>

namespace aa
{
namespace modules
{
namespace nav
{
namespace simulator
{

class Simulant;
class AutoSimulant;

class SimpleBackend
{
public:
	typedef ::math::flt flt;
	SimpleBackend();
	virtual ~SimpleBackend();

	virtual void update(flt frameTime);
	virtual Simulant * getSimulant();
	virtual AutoSimulant * getAutoSimulant();

	virtual void draw();
	virtual std::string getName() const;
private:
};

} // namespace modules
} // namespace nav
} // namespace simulator
}
