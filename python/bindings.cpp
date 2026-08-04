#include "evalio/convert/base.h"
#include "evalio/convert/gtsam.h"
#include "evalio/pipeline.h"
#include "evalio/types.h"

#include "form/feature/extraction.hpp"
#include "form/form.hpp"

#include <cstdio>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/variant.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
namespace ev = evalio;

// ------------------------- Helpers ------------------------- //
namespace evalio {

template <> inline Point convert(const form::PointFeat &in) {
  return {
      .x = in.x,
      .y = in.y,
      .z = in.z,
      .intensity = 0.0,
      .t = Duration::from_sec(0),
      .row = 0,
      .col = static_cast<uint16_t>(in.scan),
  };
}

template <> inline Point convert(const form::PlanarFeat &in) {
  return {
      .x = in.x,
      .y = in.y,
      .z = in.z,
      .intensity = 0.0,
      .t = Duration::from_sec(0),
      .row = 0,
      .col = static_cast<uint16_t>(in.scan),
  };
}

template <> inline form::PointXYZf convert(const Point &in) {
  return {
      static_cast<float>(in.x),
      static_cast<float>(in.y),
      static_cast<float>(in.z),
  };
}

} // namespace evalio

// ------------------------- Pipeline ------------------------- //
class FORMDev : public ev::Pipeline {
public:
  FORMDev() : estimator_(), params_() {}

  // Info
  static std::string version() { return "0.2.0"; } // x-release-please-version

  static std::string name() { return "form-dev"; }

  static std::string url() { return "https://github.com/rpl-cmu/form"; }

  // clang-format off
  EVALIO_SETUP_PARAMS(
    // FEATURE EXTRACTION
    (int,         neighbor_points,   5, params_.extraction.neighbor_points),
    (int,             num_sectors,   6, params_.extraction.num_sectors),
    (double,     planar_threshold, 1.0, params_.extraction.planar_threshold),
    (int, planar_feats_per_sector,  50, params_.extraction.planar_feats_per_sector),
    (int,  point_feats_per_sector,   3, params_.extraction.point_feats_per_sector),
    (double,               radius, 1.0, params_.extraction.radius),
    (int,              min_points,   5, params_.extraction.min_points),
    // OPTIMIZATION
    (double,  max_dist_matching,   0.8, params_.matcher.max_dist_matching),
    (double, new_pose_threshold,  1e-4, params_.matcher.new_pose_threshold),
    (int,     max_num_rematches,    30, params_.matcher.max_num_rematches),
    (bool,    disable_smoothing, false, params_.constraints.disable_smoothing),
    // MAPPING
    (int,         max_num_keyscans,  50, params_.scans.max_num_keyscans),
    (int,     max_num_recent_scans,  10, params_.scans.max_num_recent_scans),
    (int, max_steps_unused_keyscan,  10, params_.scans.max_steps_unused_keyscan),
    (double,    keyscan_match_ratio, 0.1, params_.scans.keyscan_match_ratio),
    (double,           min_dist_map, 0.1, params_.map.min_dist_map),
    // misc
    (int, num_threads, 0, params_.num_threads)
  );
  // clang-format on

  // Getters
  const std::map<std::string, std::vector<ev::Point>> map() override {
    const auto planar = std::get<0>(estimator_.m_keypoint_map)
                            .to_vector(estimator_.m_constraints.get_values());
    const auto point = std::get<1>(estimator_.m_keypoint_map)
                           .to_vector(estimator_.m_constraints.get_values());

    return ev::make_map("planar", planar, "point", point);
  }

  // Setters
  void set_imu_params(ev::ImuParams params) override {}

  void set_lidar_params(ev::LidarParams params) override {
    params_.extraction.min_norm_squared = params.min_range * params.min_range;
    params_.extraction.max_norm_squared = params.max_range * params.max_range;
    params_.extraction.num_columns = params.num_columns;
    params_.extraction.num_rows = params.num_rows;
  }

  void set_imu_T_lidar(ev::SE3 T) override {
    lidar_T_imu_ = ev::convert<gtsam::Pose3>(T).inverse();
  }

  // Doers
  void initialize() override { estimator_ = form::Estimator(params_); }

  void add_imu(ev::ImuMeasurement mm) override {}

  void add_lidar(ev::LidarMeasurement mm) override {
    const auto scan = ev::convert_iter<std::vector<form::PointXYZf>>(mm.points);

    auto [planar_kp, point_kp] = estimator_.register_scan(scan);

    this->save(mm.stamp, estimator_.current_lidar_estimate() * lidar_T_imu_);
    this->save(mm.stamp, "planar", planar_kp, "point", point_kp);
  }

private:
  form::Estimator estimator_;
  form::Estimator::Params params_;

  gtsam::Pose3 lidar_T_imu_ = gtsam::Pose3::Identity();
  ev::SE3 current_pose_ = ev::SE3::identity();
};

NB_MODULE(_core, m) {
  m.doc() = "Custom evalio pipeline example";

  nb::module_ eval = nb::module_::import_("evalio");

  // Only have to override the static methods here
  // All the others will be automatically inherited from the base class
  nb::class_<FORMDev, evalio::Pipeline>(m, "FORMDev")
      .def(nb::init<>())
      .def_static("name", &FORMDev::name)
      .def_static("url", &FORMDev::url)
      .def_static("default_params", &FORMDev::default_params);

  // Expose extraction methods too
  nb::class_<form::FeatureExtractor::Params>(m, "KeypointExtractionParams")
      .def(nb::init<>())
      .def_rw("neighbor_points", &form::FeatureExtractor::Params::neighbor_points)
      .def_rw("num_sectors", &form::FeatureExtractor::Params::num_sectors)
      .def_rw("planar_feats_per_sector",
              &form::FeatureExtractor::Params::planar_feats_per_sector)
      .def_rw("planar_threshold", &form::FeatureExtractor::Params::planar_threshold)
      .def_rw("point_feats_per_sector",
              &form::FeatureExtractor::Params::point_feats_per_sector)
      // Parameters for normal estimation
      .def_rw("radius", &form::FeatureExtractor::Params::radius)
      .def_rw("min_points", &form::FeatureExtractor::Params::min_points)
      // Based on LiDAR info
      .def_rw("min_norm_squared", &form::FeatureExtractor::Params::min_norm_squared)
      .def_rw("max_norm_squared", &form::FeatureExtractor::Params::max_norm_squared)
      .def_rw("num_rows", &form::FeatureExtractor::Params::num_rows)
      .def_rw("num_columns", &form::FeatureExtractor::Params::num_columns);

  m.def("extract_keypoints", [](const std::vector<Eigen::Vector3d> &points,
                                const form::FeatureExtractor::Params &params,
                                evalio::LidarParams &lidar_params) {
    // Convert to form points
    std::vector<form::PointXYZf> points_form;
    for (const auto &point : points) {
      points_form.emplace_back(point.x(), point.y(), point.z());
    }

    // Call the keypoint extraction function from the Tclio class
    auto [planar_keypoints, point_keypoints] =
        form::FeatureExtractor(params, 0).extract(points_form, 0);

    // return a tuple of (planar_points, normals, point_points)
    std::vector<Eigen::Vector3d> planar_points;
    std::vector<Eigen::Vector3d> normals;
    std::vector<Eigen::Vector3d> point_points;
    for (const auto &keypoint : planar_keypoints) {
      planar_points.emplace_back(keypoint.x, keypoint.y, keypoint.z);
      normals.emplace_back(keypoint.nx, keypoint.ny, keypoint.nz);
    }
    for (const auto &keypoint : point_keypoints) {
      point_points.emplace_back(keypoint.x, keypoint.y, keypoint.z);
    }

    return std::make_tuple(planar_points, normals, point_points);
  });
}
