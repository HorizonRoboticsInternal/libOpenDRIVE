#include "OpenDriveMap.h"
#include <fstream>
#include <cassert>
#include <iostream>

void export_lane_points(const odr::OpenDriveMap& odr_map, const std::string& output_path, const double eps = 1.0)
{
    std::ofstream outfile(output_path);
    assert(outfile.is_open() && "Failed to open output file");
    outfile << "road_id,junction_id,lanesection_s0,lane_id,type,x,y,z,s,width,forward_x,forward_y,forward_z\n";

    for (const odr::Road& road : odr_map.get_roads())
    {
        for (const odr::LaneSection& lanesection : road.get_lanesections())
        {
            for (const odr::Lane& lane : lanesection.get_lanes())
            {
                if (lane.type != "sidewalk" && lane.type != "driving") continue;

                const double s_start = lanesection.s0;
                const double s_end = road.get_lanesection_end(lanesection);

                const odr::Lane& inner_neighbor_lane = lanesection.get_lane(odr::next_towards_zero(lane.id));

                double s = s_start;
                while (s <= s_end)
                {
                    double s_sample = (s + eps > s_end && s < s_end) ? s_end : s;

                    double t_outer = lane.outer_border.get(s_sample);
                    double t_inner = inner_neighbor_lane.outer_border.get(s_sample);
                    double t_center = 0.5 * (t_outer + t_inner);

                    odr::Vec3D normal, forward;
                    odr::Vec3D pt = road.get_surface_pt(s_sample, t_center, &normal, &forward);

                    double width = std::abs(t_outer - t_inner);
                    outfile << road.id << "," << road.junction << "," << lanesection.s0 << "," << lane.id << "," << lane.type << ","
                            << pt[0] << "," << pt[1] << "," << pt[2] << "," << s_sample << ","
                            << width << "," << forward[0] << "," << forward[1] << "," << forward[2] << "\n";

                    if (s_sample == s_end) break;
                    s += eps;
                }
            }
        }
    }

    outfile.close();
    std::cout << "Lane points written to " << output_path << std::endl;
}

void export_crosswalk_corners(const odr::OpenDriveMap& odr_map, const std::string& output_path)
{
    std::ofstream xwalk_file(output_path);
    assert(xwalk_file.is_open() && "Failed to open crosswalk file");
    xwalk_file << "road_id,junction_id,object_id,corner_index,x,y,z\n";

    for (const odr::Road& road : odr_map.get_roads())
    {
        for (const auto& [object_id, road_object] : road.id_to_object)
        {
            if (road_object.type != "crosswalk")
                continue;

            for (const auto& outline : road_object.outlines)
            {
                for (size_t i = 0; i < outline.outline.size(); ++i)
                {
                    odr::Vec3D e_s, e_t, e_h;
                    const odr::Vec3D p0 = road.get_xyz(road_object.s0, road_object.t0, road_object.z0, &e_s, &e_t, &e_h);
                    odr::Mat3D base_mat = {{{e_s[0], e_t[0], e_h[0]},
                                            {e_s[1], e_t[1], e_h[1]},
                                            {e_s[2], e_t[2], e_h[2]}}};

                    odr::Mat3D rot_mat = odr::EulerAnglesToMatrix<double>(road_object.roll, road_object.pitch, road_object.hdg);

                    const odr::Vec3D& local = outline.outline[i].pt;
                    odr::Vec3D rotated = odr::MatVecMultiplication(rot_mat, local);
                    odr::Vec3D transformed = odr::MatVecMultiplication(base_mat, rotated);
                    odr::Vec3D world_pt = odr::add(transformed, p0);

                    xwalk_file << road.id << "," << road.junction << "," << object_id << "," << i << ","
                               << world_pt[0] << "," << world_pt[1] << "," << world_pt[2] << "\n";
                }
            }
        }
    }

    xwalk_file.close();
    std::cout << "Crosswalk corners written to " << output_path << std::endl;
}

int main()
{
    // odr::OpenDriveMap odr_map("/home/carla/CarlaUnreal/Content/Carla/Maps/OpenDrive/Town03_Opt.xodr");
    odr::OpenDriveMap odr_map("/home/carla/CarlaUnreal/Content/Carla/Maps/OpenDrive/Town03_Opt.xodr");
    assert(odr_map.xml_parse_result && "Failed to parse test.xodr");

    // sample resolution
    const double eps = 1.00;

    // export_lane_points(odr_map, "/home/FlashDrive/temp/town03_map.csv", eps);
    // export_crosswalk_corners(odr_map, "/home/FlashDrive/temp/town03_crosswalks.csv");
    export_lane_points(odr_map, "/home/FlashDrive/temp/Town03_map.csv", eps);
    export_crosswalk_corners(odr_map, "/home/FlashDrive/temp/Town03_crosswalks.csv");
}