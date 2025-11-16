#pragma once

#include <string>
#include <vector>
#include <sdf/sdf.hh>

#include "../util/map_loader.h"

class SdfParser {
public:

    bool loadSdf(const std::string& filename, std::vector<VisLib::Segment> segments) {
        sdf::Root root;
        sdf::Errors errors = root.Load(filename);

        const sdf::World *world = root.WorldByIndex(0);
        const uint64_t nModels = world->ModelCount();

        std::cout << "=== " << nModels <<  " Models in " << world->Name() << " ===\n";

        for (size_t i = 0; i < nModels; ++i) {
            const sdf::Model *model = world->ModelByIndex(i);

            std::cout << "\nModel: " << model->Name() << std::endl;
            std::cout << "Pose (x y z roll pitch yaw): " << model->RawPose() << std::endl;

            for (size_t linkIndex = 0; linkIndex < model->LinkCount(); ++linkIndex) {


                const sdf::Link *link = model->LinkByIndex(linkIndex);
                std::cout << "\nLink: " << link->Name() << std::endl;
                const sdf::Collision *collision = link->CollisionByIndex(0);

                std::cout << "Pose (x y z roll pitch yaw): " << collision->RawPose() << std::endl;
                const sdf::Geometry *geom = collision->Geom();

                switch (geom->Type()) {
                    case sdf::GeometryType::BOX: {
                        const gz::math::Vector3d size = geom->BoxShape()->Size();
                        std::cout << "Box (L×B×H): "
                                  << size.X() << " × "
                                  << size.Y() << " × "
                                  << size.Z() << " m\n";
                        break;
                    }
                    case sdf::GeometryType::SPHERE: {
                        const double r = geom->SphereShape()->Radius();
                        std::cout << "Radius: " << r << " m\n";
                        break;
                    }
                    default:
                        std::cout << "Model type not in [Box, Sphere]\n";
                }
            }
        }
        std::cout << "\nDone!\n";
        return true;
    }

    void extractWallSegments(const std::string &filename, std::vector<VisLib::Segment>& segments, std::vector<VisLib::Point>& points) {
        int segmentId = 0;

        sdf::Root root;
        sdf::Errors errors = root.Load(filename);

        if (!errors.empty()) {
            std::cerr << "Error loading SDF file:\n";
            for (const auto &error: errors) {
                std::cerr << error << std::endl;
            }
        }

        const sdf::World *world = root.WorldByIndex(0);
        if (!world) {
            std::cerr << "No world found in SDF file\n";
        }

        const uint64_t nModels = world->ModelCount();
        std::cout << "Processing " << nModels << " models...\n";

        for (size_t i = 0; i < nModels; ++i) {
            const sdf::Model *model = world->ModelByIndex(i);
            gz::math::Pose3d modelPose = model->RawPose();

            std::cout << "\nModel: " << model->Name() << std::endl;

            for (size_t linkIndex = 0; linkIndex < model->LinkCount(); ++linkIndex) {
                const sdf::Link *link = model->LinkByIndex(linkIndex);
                gz::math::Pose3d linkPose = link->RawPose();

                gz::math::Pose3d absoluteLinkPose = modelPose * linkPose;

                std::cout << "  Link: " << link->Name() << std::endl;

                for (size_t collIndex = 0; collIndex < link->CollisionCount(); ++collIndex) {
                    const sdf::Collision *collision = link->CollisionByIndex(collIndex);
                    const sdf::Geometry *geom = collision->Geom();

                    if (geom->Type() != sdf::GeometryType::BOX) {
                        continue;
                    }

                    gz::math::Pose3d collisionPose = collision->RawPose();
                    const gz::math::Vector3d size = geom->BoxShape()->Size();

                    gz::math::Pose3d absolutePose = absoluteLinkPose * collisionPose;

                    double x = absolutePose.Pos().X();
                    double y = -absolutePose.Pos().Y();
                    double yaw = -absolutePose.Rot().Yaw();

                    if (size.Z() != 2.5) {
                        continue;
                    }
                    //points.push_back({x, y});

                    double length = size.X();
                    double width = size.Y();

                    double wallLength = length;
                    double wallThickness = width;

                    if (width > length) {
                        wallLength = width;
                        wallThickness = length;
                        yaw += M_PI / 2.0;
                    }

                    double halfLength = wallLength / 2.0;

                    double dx = (halfLength - wallThickness / 2) * cos(yaw);
                    double dy = (halfLength - wallThickness / 2) * sin(yaw);

                    VisLib::Point p1(x - dx, y - dy);
                    VisLib::Point p2(x + dx, y + dy);

                    VisLib::Segment seg(segmentId++, p1, p2);
                    segments.push_back(seg);

                    std::cout << "    " << seg << std::endl;
                }
            }
        }
        std::cout << "\nTotal segments extracted: " << segments.size() << std::endl;
    }
};
