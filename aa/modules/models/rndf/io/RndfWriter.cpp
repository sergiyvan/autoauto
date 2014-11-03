#include "RndfWriter.h"
#include "RndfData.h"

using namespace ::math;

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

RNDFWriter::RNDFWriter()
{}

RNDFWriter::~RNDFWriter()
{}

//---------------------------------------------------------------------------
void RNDFWriter::writedownSpot(unsigned int z, Spot const & s)
{
	os << "spot\t" << z << '.' << s.id << std::endl;

	if (s.width != 0) {
		os << "spot_width\t" << s.width << std::endl;
	}

	os << "checkpoint\t" << z << '.' << s.id << '.' << '2' << '\t' << s.waypoints[1].checkpoint_id << std::endl;
	os << z << '.' << s.id << '.' << 1 << '\t' << s.waypoints[0].latitude << '\t' << s.waypoints[0].longitude << std::endl;
	os << z << '.' << s.id << '.' << '2' << '\t' << s.waypoints[1].latitude << '\t' << s.waypoints[1].longitude << std::endl;
	os << "end_spot" << std::endl;
}

void RNDFWriter::writedownPerimeter(unsigned int z)
{
	/*TODO:
	// 	fprintf(out, "%s%c%i.%i\n", "perimeter", 9, z+1+rd->number_of_segments, 0);
	os << "perimeter\t" << z+1+rd->number_of_segments << '.' << '0' << std::endl;

	// 	fprintf(out, "%s%c%i\n", "num_perimeterpoints", 9, rd->zones[z].number_of_perimeterpoints);
	os << "num_perimeterpoints\t" << rd->zones[z].number_of_perimeterpoints << std::endl;

	for (int x=0; x<2; x++) {
		for (int w= 0; w < rd->zones[z].number_of_perimeterpoints; w++) {
			switch(x)
			{
			case 0:
				if (rd->zones[z].perimeterpoints[w].exits != 0)
					for (int e = 0; e < rd->zones[z].perimeterpoints[w].exits; e++) {
						// Exit
	// 						fprintf(out, "%s%c%i.%i.%i%c%i.%i.%i\n", "exit", 9, z+1+rd->number_of_segments, 0, w+1, 9, rd->zones[z].perimeterpoints[w].exit_to[e].segment+1, rd->zones[z].perimeterpoints[w].exit_to[e].lane+1, rd->zones[z].perimeterpoints[w].exit_to[e].waypoint+1);
						os << "exit\t"
								<< z+1+rd->number_of_segments << '.'
								<< '0' << '.'
								<< w+1
							<< '\t'
								<< rd->zones[z].perimeterpoints[w].exit_to[e].segment+1 << '.'
								<< rd->zones[z].perimeterpoints[w].exit_to[e].lane+1 << '.'
								<< rd->zones[z].perimeterpoints[w].exit_to[e].waypoint+1 << std::endl;
					}
				break;
			default:
				// Waypoint
	// 				fprintf(out, "%i.%i.%i%c%.6f%c%.6f\n", z+1+rd->number_of_segments, 0, w+1, 9, rd->zones[z].perimeterpoints[w].latitude, 9, rd->zones[z].perimeterpoints[w].longitude);
				os << z+1+rd->number_of_segments << '.' << '0' << '.' << w+1 << '\t' <<  rd->zones[z].perimeterpoints[w].latitude << '\t' << rd->zones[z].perimeterpoints[w].longitude << std::endl;
			}
		}
	}
	// 	fprintf(out, "end_perimeter\n");
	os << "end_perimeter" << std::endl;
	*/
}

void RNDFWriter::writedownZone(Zone const & z)
{
	os << "zone\t" << z.id << std::endl;
	os << "num_spots\t" << z.spots.size() << std::endl;

	if (z.name.size() > 0) {
		os << "zone_name\t" << z.name << std::endl;
	}

	writedownPerimeter(z.id);

	for (std::vector<Spot>::const_iterator s = z.spots.begin(); s != z.spots.end(); s++) {
		writedownSpot(z.id, *s);
	}

	os << "end_zone" << std::endl;
}

void RNDFWriter::writedownLane(Lane const & l)
{
	os << "lane\t" <<  l.segment << '.' << l.id << std::endl;
	os << "num_waypoints\t" << l.waypoints.size() << std::endl;

// 	if (l.width != 0) {
// 		os << "lane_width\t" << l.width << std::endl;
// 	}

	if (l.left_boundary != LANE_BOUNDARY_NONE) {
		os << "left_boundary\t" << boundaryType_str[l.left_boundary] << std::endl;
	}

	if (l.right_boundary != LANE_BOUNDARY_NONE) {
		os << "right_boundary\t" << boundaryType_str[l.right_boundary] << std::endl;
	}

	for (int x = 0; x < 4; x++) {
		for (std::vector<Waypoint>::const_iterator w = l.waypoints.begin(); w != l.waypoints.end(); w++) {
			switch (x) {
			case 0:

				if (w->checkpoint_id != 0)
					/* Checkpoint */
				{
					os << "checkpoint\t" << l.segment << '.' << l.id << '.' << w->id << '\t' << w->checkpoint_id << std::endl;
				}

				break;

			case 1:

				if (w->stopsign)
					/* Stopsign   */
				{
					os << "stop\t" << l.segment << '.' << l.id << '.' << w->id << std::endl;
				}

				break;

			case 2:

				for (uint e = 0; e < w->exit_to.size(); e++)
					/* Exit       */
					os << "exit\t" << l.segment << '.' << l.id << '.' << w->id << '\t'
					   << w->exit_to[e].segment << '.' <<  w->exit_to[e].lane << '.' <<  w->exit_to[e].waypoint << std::endl;

				break;

			default:
				/* Waypoint  */
				os << l.segment << '.' << l.id << '.' << w->id << '\t' << w->latitude << '\t' << w->longitude << std::endl;
			}
		}
	}

	os << "end_lane" << std::endl;
}

void RNDFWriter::writedownSegment(Segment const & s)
{
	os << "segment\t" << s.id << std::endl;
	os << "num_lanes\t" << s.lanes.size() << std::endl;
	os << "segment_name\t" << s.name << std::endl;

	for (std::vector<Lane>::const_iterator l = s.lanes.begin(); l != s.lanes.end(); l++) {
		writedownLane(*l);
	}

	os << "end_segment" << std::endl;
}

void RNDFWriter::writedownRNDF()
{
	os << "RNDF_name\t" << rd->filename << std::endl;
	os << "num_segments\t" << rd->segments.size() << std::endl;
	os << "num_zones\t" << rd->zones.size() << std::endl;
	os << "format_version\t" << rd->format_version << std::endl;
	os << "creation_date\t" << rd->creation_date << std::endl;

	for (std::vector<Segment>::const_iterator x = rd->segments.begin(); x != rd->segments.end(); x++) {
		writedownSegment(*x);
	}

	for (std::vector<Zone>::const_iterator x = rd->zones.begin(); x != rd->zones.end(); x++) {
		writedownZone(*x);
	}

	os << "end_file" << std::endl;
}
//---------------------------------------------------------------------------
void RNDFWriter::saveRNDF(RNDFData * rndfData, const char * fname)
{
	rd = rndfData;
	os.open(fname);
	os.precision(30);
	writedownRNDF();
	os.close();
	rd = NULL;
}


}


}


}


}


}


