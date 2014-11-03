/**
 * \file "FieldOfView.h"
 * \brief Describes the field of view of a sensor as a polygon.
 */

#include "FieldOfView.h"
#include <math/IbeoMath.h>

using namespace aa::data::obstacle::util;

FieldOfView::FieldOfView()
{
}


FieldOfView::~FieldOfView()
{
}


FieldOfView::FieldOfView(FieldOfView const & o)
	: math::Polygon<flt>(o)
{
}


FieldOfView & FieldOfView::operator=(FieldOfView const & o)
{
	if (&o == this) {
		return *this;
	}

	math::Polygon<flt>::operator=(o);

	return *this;
}


FieldOfView & FieldOfView::operator=(math::Polygon<flt> const & o)
{
	math::Polygon<flt>::operator=(o);

	return *this;
}


void FieldOfView::transform(Affine3 const & m)
{
	for (unsigned int s = 0; s < num_sheets(); ++s) {
		for (unsigned int p = 0; p < (*this)[s].size(); ++p) {
			Vec3 p_new = m * Vec3((*this)[s][p][0], (*this)[s][p][1], 0.0);
			(*this)[s][p][0] = p_new[0];
			(*this)[s][p][1] = p_new[1];
		}
	}
}

