#include <fstream>

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

class RNDFData;
class Lane;
class Segment;
class Zone;
class Spot;

class RNDFWriter
{
public:
	RNDFWriter();
	~RNDFWriter();
	void saveRNDF(RNDFData * rndfData, const char * fname);
	void saveRNDF(const char * fname);

protected:
	void    writedownSpot(unsigned int z, Spot const & s);
	void    writedownPerimeter(unsigned int z);
	void    writedownZone(Zone const & z);
	void    writedownLane(Lane const & l);
	void    writedownSegment(Segment const & s);
	void    writedownRNDF();

private:
	RNDFData * rd;
	std::ofstream os;
};

}

}

}

}

}

