#include "Contour.h"
#include "ScannerSystem.h"

#include <math/AutoMath.h>
#include <math/IbeoMath.h>
#include <data/geometry/BoundingBox3D.h>


using namespace std;
using namespace math;
using namespace aa::data::obstacle::util;
using namespace ::data::geometry;

const flt Contour::MAX_CONTOUR_POINT_DISTANCE	  = 1.0;
const flt Contour::MAX_CONTOUR_POINT_DISTANCE_SQR = 1.0;

Contour::Contour()
	: mReferencePoint(Vec2(0.0, 0.0))
	, mMinInclinationAngle(0.0)
	, mHeight(0.0)
{

}


void Contour::compute(BoundingBox3D const & boundingBox, ScannerSystem const & scanner)
{
	clear();
	unsigned int num = BoundingBox3D::NUM_VERTICES_PER_PLANE;
	boost::array<unsigned int, 3> indices  = boundingBox.visibleVertexIndices(scanner.getGlobalPosition2D());

	if (indices[0] != indices[1] && indices[2] != indices[1]) {
		push_back(boundingBox.vertex2D(indices[0]));
		push_back(boundingBox.vertex2D(indices[1]));
		push_back(boundingBox.vertex2D(indices[2]));
	}
	else {
		if (indices[2] == ((indices[0] + 1) % num)) {
			push_back(boundingBox.vertex2D(indices[0]));
			push_back(boundingBox.vertex2D(indices[2]));
			push_back(boundingBox.vertex2D((indices[2] + 1) % num));
		}
		else {
			push_back(boundingBox.vertex2D(indices[0]));
			push_back(boundingBox.vertex2D(indices[2]));
			push_back(boundingBox.vertex2D((indices[2] - 1) % num));
		}
	}

	sample();
	mReferencePoint = scanner.getGlobalPosition2D();
	mMinInclinationAngle = 0.0 * math::D2R;
}

struct SortByAngle {
	SortByAngle(Vec2 const & p)
	{
		ref = p;
	}

	bool operator()(Vec2 const & p1, Vec2 const & p2)
	{
		return math::ibeomath::isLeftTo((p1 - ref), (p2 - ref));
	}

	Vec2 ref;
};

Contour Contour::merge(Contour const & c1, Contour const & c2)
{
	std::vector<Vec2> ch;

	for (uint i = 0; i < c1.size(); ++i) {
		ch.push_back(c1[i]);
	}

	for (uint i = 0; i < c2.size(); ++i) {
		ch.push_back(c2[i]);
	}

	Vec2 refPoint = (c1.referencePoint() + c2.referencePoint()) / 2.0;

	std::sort(ch.begin(), ch.end(), SortByAngle(refPoint));

	Contour c;
	c.compute(ch, refPoint, std::min(c1.minInclinationAngle(), c2.minInclinationAngle()));
	return c;
}


void Contour::compute(std::vector<Vec2> const & convexhull, Vec2 const & referencePoint)
{
	clear();

	mReferencePoint			= referencePoint;
	mMinInclinationAngle	= 0.0;
	mHeight					= 0.0;

	int n = convexhull.size();

	if (n == 0) {
		return;
	}

	if (n == 1) {
		push_back(convexhull.front());
		return;
	}

	if (n == 2) {
		push_back(convexhull.front());
		push_back(convexhull.back());
	}
	else {
		unsigned int lI = 0;
		unsigned int rI = 0;
		Vec2 l = convexhull.front() - mReferencePoint;
		Vec2 r = l;

		for (unsigned int i = 1; i < n; i++) {
			Vec2 const & p = convexhull.at(i);
			Vec2 v = p - mReferencePoint;

			if (ibeomath::isLeftTo(v, l)) {
				l = v;
				lI = i;
			}

			if (ibeomath::isRightTo(v, r)) {
				r = v;
				rI = i;
			}
		}

		l = convexhull.at(lI);
		r = convexhull.at(rI);

		if (ssd(l, r) == 0.0) {
			for (unsigned int i = 0; i < n; i++) {
				push_back(convexhull.at(i));
			}
		}
		else {
			Vec2 lr = r - l;
			Vec2 vecFirst = convexhull.at((lI + 1) % n) - l;

			if (ibeomath::isRightTo(vecFirst, lr)) {
				// Start with left convex hull vertex
				for (unsigned int i = lI; i < lI + n; i++) {
					push_back(convexhull.at(i % n));

					if (i % n == rI) {
						break;
					}
				}
			}
			else if (ibeomath::isLeftTo(vecFirst, lr)) {
				// Start with right convex hull vertex
				for (unsigned int i = rI; i < rI + n; i++) {
					push_back(convexhull.at(i % n));

					if (i % n == lI) {
						break;
					}
				}
			}
			else {
				for (unsigned int i = 0; i < n; i++) {
					push_back(convexhull.at(i));
				}
			}
		}
	}

//	validate();
	sample();
}

void Contour::compute(std::vector<Vec2> const & convexhull)
{
    clear();

    int n = convexhull.size();
    for (unsigned int i = 0; i < n; i++) {
        push_back(convexhull.at(i));
    }

    sample();
}


void Contour::computeFromUnsortedList(
    std::vector<Vec2> list,
    Vec2 const & refPoint,
    flt minInclinationAngle)
{
	std::sort(list.begin(), list.end(), SortByAngle(refPoint));
	compute(list, refPoint, minInclinationAngle);
}


void Contour::addSamplePoints(Vec2 p1, Vec2 p2, flt sampleDist)
{
	flt sampleDist2 = sqr(sampleDist);

	if (ssd(p1, p2) > sampleDist2) {
		Vec2 dir = (p2 - p1).normalized();

		while (ssd(p1, p2) > sampleDist2) {
			this->push_back(p1 + sampleDist * dir);
			p1 = this->back();
		}
	}
}


void Contour::buildFromSortedList(
    std::vector<Vec2> const & sorting,
    Vec2 const & referencePoint,
    flt sampleDist)
{
	// Clear contour
	clear();

	uint n = sorting.size();

	if (n == 1) {
		this->push_back(sorting.front());
	}
	else if (n == 2) {
		Vec2 const & p1 = sorting.front();
		Vec2 const & p2 = sorting.back();
		this->push_back(p1);
		addSamplePoints(p1, p2, sampleDist);
		this->push_back(p2);
	}
	else {
		uint nn = 0;
		vector<bool> liesOnContour(n, false);
		liesOnContour[0] = true;
		nn++;
		liesOnContour[1] = true;
		nn++;
		int index1 = -1;
		int index2 = -1;

		for (uint i = 2; i < n; i++) {
			index1 = -1;
			index2 = -1;

			for (int j = i - 1; j >= 0; j--) {
				if (liesOnContour[j] && index1 < 0) {
					index1 = j;
				}
				else if (liesOnContour[j]) {
					index2 = j;
					break;
				}
			}

			if (index2 < 0) {
				liesOnContour[i] = true;
				nn++;
				continue;
			}

			Vec2 const & p1 = sorting[index1];
			Vec2 const & p2 = sorting[index2];
			Vec2 const & p  = sorting[i];

			if ((p2 - p1).squaredNorm() == 0.0) {
				liesOnContour[index1] = false;
				liesOnContour[i] = true;
				continue;
			}

			if ((p - p1).squaredNorm() == 0.0 || (p - p2).squaredNorm() == 0.0) {
				continue;
			}

			if (leftOfTest(p, p2, p1)) {
				liesOnContour[i] = true;
				nn++;
			}
			else {
				liesOnContour[index1] = false;
				nn--;
				--i;
			}
		}

		this->reserve(nn * 2);

		for (unsigned int i = 0; i < n; i++) {
			if (liesOnContour[i]) {
				if (this->empty()) {
					this->push_back(sorting.at(i));
				}
				else {
					addSamplePoints(this->back(), sorting.at(i), sampleDist);
					this->push_back(sorting.at(i));
				}
			}
		}
	}

	mReferencePoint = referencePoint;
	mMinInclinationAngle = 0.0;
}


void Contour::compute(std::vector<Vec2> const & sorting,
                      Vec2 const & referencePoint,
                      flt minInclinationAngle)
{
	clear();
	const flt angle179 = 179.0 * math::D2R;
	const flt angle0   = 0.0;
	flt minangle = minInclinationAngle;

	unsigned int sizeSorting = sorting.size();

	if (sizeSorting == 1) {
		push_back(sorting.front());
	}
	else {
		bool liesOnContour[sizeSorting];

		for (unsigned int i = 0; i < sizeSorting; i++) {
			liesOnContour[i] = false;
		}

		liesOnContour[0] = true;
		liesOnContour[1] = true;

		for (unsigned int i = 2; i < sizeSorting; i++) {
			int index1 = -1;
			int index2 = -1;

			for (int j = i - 1; j >= 0; j--) {
				if (liesOnContour[j] && index1 < 0) {
					index1 = j;
				}
				else if (liesOnContour[j]) {
					index2 = j;
					break;
				}
			}

			if (index2 < 0) {
				liesOnContour[i] = true;
				continue;
			}

			Vec2 const & p1 = sorting.at(index1);
			Vec2 const & p2 = sorting.at(index2);
			Vec2 const & p  = sorting.at(i);

			if ((p2 - p1).squaredNorm() == 0.0f) {
				liesOnContour[index1] = false;
				liesOnContour[i] = true;
				continue;
			}

			if ((p - p1).squaredNorm() == 0.0f || (p - p2).squaredNorm() == 0.0f) {
				continue;
			}

			flt angle = math::angle((p1 - p2), (p - p1));

			if (angle > minangle && angle < angle179 && angle != angle0) {
				liesOnContour[i] = true;
			}
			else {
				liesOnContour[index1] = false;
				i--;
			}
		}

		for (unsigned int i = 0; i < sizeSorting; i++) {
			if (liesOnContour[i]) {
				push_back(sorting.at(i));
			}
		}
	}

	mReferencePoint = referencePoint;
	mMinInclinationAngle = minInclinationAngle;
	sample();
}


bool Contour::liesBehindContour(Vec2 const & point) const
{
	if (!liesInsideContourCone(point)) {
		return false;
	}

	if (ibeomath::isRightTo(front() - mReferencePoint, back() - mReferencePoint)) {
		for (unsigned int i = 0; i < size() - 1; i++) {
			Vec2 const & right = at(i);
			Vec2 const & left = at(i + 1);

			if (liesInsideContourCone(point, left, right)) {
				return ibeomath::isLeftTo(point - left, right - left);
			}
		}
	}
	else {
		for (int i = 0; i < int(size()) - 1; i++) {
			Vec2 const & left = at(i);
			Vec2 const & right = at(i + 1);

			if (liesInsideContourCone(point, left, right)) {
				return ibeomath::isLeftTo(point - left, right - left);
			}
		}
	}

	return false;
}


bool Contour::liesInsideContourCone(Vec2 const & point) const
{
	return liesInsideContourCone(point, front(), back());
}


bool Contour::occludes(Contour const & other) const
{
	for (unsigned int i = 0; i < other.size(); i++) {
		if (liesBehindContour(other[i])) {
			return true;
		}
	}

	return false;
}



/** *************************************************************************************
*  Private
**/

void Contour::sample()
{
	// Clear the contour points
	std::vector< ::math::Vec2> contour = *this;
	clear();

	for (int i = 0; i < int(contour.size()) - 1; i++) {
		Vec2 const & p = contour.at(i);
		push_back(p);

		Vec2 const & pNext = contour.at(i + 1);
		Vec2 dir = normalized(pNext - p);
		flt gap2 = ssd(p, pNext);

		while (gap2 > MAX_CONTOUR_POINT_DISTANCE_SQR) {
			push_back(p + MAX_CONTOUR_POINT_DISTANCE * dir);
			gap2 -= MAX_CONTOUR_POINT_DISTANCE_SQR;
		}
	}

	push_back(contour.back());
}


bool Contour::liesInsideContourCone(Vec2 const & point, Vec2 const & cpLeft, Vec2 const & cpRight) const
{
	Vec2 const & vecLeft  = cpLeft - mReferencePoint;
	Vec2 const & vecRight = cpRight - mReferencePoint;
	Vec2 const & vecPoint = point - mReferencePoint;

	return math::ibeomath::isLeftTo(vecPoint, vecRight) && math::ibeomath::isRightTo(vecPoint, vecLeft);
}


void Contour::validate() const
{
	Vec2 bf = this->back() - this->front();
	bool Left = ibeomath::isLeftTo(mReferencePoint - this->front(), bf);

	for (int i = 1; i < int(this->size()) - 1; i++) {
		bool left = ibeomath::isLeftTo(this->at(i) - this->front(), bf);
		bool right = ibeomath::isRightTo(this->at(i) - this->front(), bf);

		if (!left && !right) {
			continue;
		}

		assert(Left == left);
	}
}

void Contour::translate(Vec2 const & translation)
{
	for (unsigned int i = 0; i < this->size(); i++) {
		Vec2 & p = this->at(i);
		p = p + translation;
	}
}


