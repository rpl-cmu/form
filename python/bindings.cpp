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
#include <utility>

namespace nb = nanobind;

// ------------------------- Helpers ------------------------- //
gtsam::Pose3 pose_to_gtsam(const evalio::SE3 &pose) {
  return gtsam::Pose3(gtsam::Rot3(pose.rot.toEigen()), gtsam::Point3(pose.trans));
}

evalio::SE3 pose_to_evalio(const gtsam::Pose3 &pose) {
  return evalio::SE3(evalio::SO3::fromEigen(pose.rotation().toQuaternion()),
                     pose.translation());
}

template <typename Point> evalio::Point point_to_evalio(const Point &point) {
  return {
      .x = point.x,
      .y = point.y,
      .z = point.z,
      .intensity = 0.0,
      .t = evalio::Duration::from_sec(0),
      .row = 0,
      .col = static_cast<uint16_t>(point.scan),
  };
}

Eigen::Vector3f point_to_form(const evalio::Point &point) {
  return Eigen::Vector3f(point.x, point.y, point.z);
}

// ------------------------- Pipeline ------------------------- //
class FORM : public evalio::Pipeline {
public:
  FORM()
      : evalio::Pipeline(), params_(),
        estimator_(std::make_unique<form::Estimator>(params_)) {}

  std::unique_ptr<form::Estimator> estimator_;
  form::Estimator::Params params_;

  // helper params
  gtsam::Pose3 lidar_T_imu_ = gtsam::Pose3::Identity();
  evalio::Duration delta_time_;
  evalio::SE3 current_pose = evalio::SE3::identity();

  // ------------------------- Info ------------------------- //
  static std::string name() { return "form"; }

  static std::string url() { return "https://github.com/rpl-cmu/form"; }

  // clang-format off
  EVALIO_SETUP_PARAMS(
    // FEATURE EXTRACTION
    (int,         neighbor_points,   5, params_.extraction.neighbor_points),
    (int,             num_sectors,   6, params_.extraction.num_sectors),
    (int, planar_feats_per_sector,  50, params_.extraction.planar_feats_per_sector),
    (int,  point_feats_per_sector,   3, params_.extraction.point_feats_per_sector),
    (double,     planar_threshold, 1.0, params_.extraction.planar_threshold),
    (double,               radius, 1.0, params_.extraction.radius),
    (int,              min_points,   5, params_.extraction.min_points),
    // OPTIMIZATION
    (double, new_pose_threshold,  1e-4, params_.matcher.new_pose_threshold),
    (int,     max_num_rematches,    30, params_.matcher.max_num_rematches),
    (double,  max_dist_matching,   0.8, params_.matcher.max_dist_matching),
    (bool,    disable_smoothing, false, params_.constraints.disable_smoothing),
    // MAPPING
    (int,         max_num_keyframes,  50, params_.scans.max_num_keyframes),
    (int,     max_num_recent_frames,  10, params_.scans.max_num_recent_frames),
    (int, max_steps_unused_keyframe,  10, params_.scans.max_steps_unused_keyframe),
    (double,    keyscan_match_ratio, 0.1, params_.scans.keyscan_match_ratio),
    (double,           max_dist_map, 0.1, params_.map.max_dist_map),
    // misc
    (int, num_threads, 0, params_.num_threads)
  );
  // clang-format on

  // ------------------------- Getters ------------------------- //
  // Returns the most recent pose estimate
  const evalio::SE3 pose() override { return current_pose; }

  // Returns the current submap of the environment
  const std::map<std::string, std::vector<evalio::Point>> map() override {
    const auto world_map =
        form::tuple::transform(estimator_->m_keypoint_map, [&](auto &map) {
          return map.to_voxel_map(estimator_->m_constraints.get_values(),
                                  estimator_->m_params.map.max_dist_map);
        });

    std::tuple<std::string, std::string> map_names = {"planar", "point"};
    std::map<std::string, std::vector<evalio::Point>> points;

    form::tuple::for_seq(std::make_index_sequence<2>{}, [&](auto I) {
      const auto name = std::get<I>(map_names);
      points.insert({name, {}});
      auto &vec = points[name];

      for (const auto &[_, voxel] : std::get<I>(world_map)) {
        for (const auto &point : voxel) {
          vec.push_back(point_to_evalio(point));
        }
      }
    });

    return points;
  }

  // ------------------------- Setters ------------------------- //
  // Set the IMU parameters
  void set_imu_params(evalio::ImuParams params) override {}

  // Set the LiDAR parameters
  void set_lidar_params(evalio::LidarParams params) override {
    params_.extraction.min_norm_squared = params.min_range * params.min_range;
    params_.extraction.max_norm_squared = params.max_range * params.max_range;
    params_.extraction.num_columns = params.num_columns;
    params_.extraction.num_rows = params.num_rows;
    delta_time_ = params.delta_time();
  }

  // Set the transformation from IMU to LiDAR
  void set_imu_T_lidar(evalio::SE3 T) override {
    lidar_T_imu_ = pose_to_gtsam(T).inverse();
  }

  // ------------------------- Doers ------------------------- //
  // Initialize the pipeline
  void initialize() override {
    estimator_ = std::make_unique<form::Estimator>(params_);
  }

  // Add an IMU measurement
  void add_imu(evalio::ImuMeasurement mm) override {}

  // Add a LiDAR measurement
  std::map<std::string, std::vector<evalio::Point>>
  add_lidar(evalio::LidarMeasurement mm) override {
    // convert to evalio
    std::vector<Eigen::Vector3f> scan;
    auto start = mm.stamp;
    auto end = mm.stamp + delta_time_;

    for (const auto &point : mm.points) {
      scan.push_back(point_to_form(point));
    }

    // run the estimator
    auto [planar_kp, point_kp] = estimator_->registerScan(scan);
    current_pose =
        pose_to_evalio(estimator_->current_lidar_estimate() * lidar_T_imu_);

    // extract the keypoints
    std::map<std::string, std::vector<evalio::Point>> points = {{"planar", {}},
                                                                {"point", {}}};
    auto &all_planar = points["planar"];
    auto &all_point = points["point"];
    for (const auto &point : planar_kp) {
      const auto ev_point = point_to_evalio(point);
      all_planar.push_back(ev_point);
    }

    for (const auto &point : point_kp) {
      const auto ev_point = point_to_evalio(point);
      all_point.push_back(ev_point);
    }

    return points;
  }
};

NB_MODULE(_core, m) {
  m.doc() = "Custom evalio pipeline example";

  nb::module_ eval = nb::module_::import_("evalio");

  // Only have to override the static methods here
  // All the others will be automatically inherited from the base class
  nb::class_<FORM, evalio::Pipeline>(m, "FORM")
      .def(nb::init<>())
      .def_static("name", &FORM::name)
      .def_static("url", &FORM::url)
      .def_static("default_params", &FORM::default_params);

  // Expose extraction methods too
  nb::class_<form::KeypointExtractionParams>(m, "KeypointExtractionParams")
      .def(nb::init<>())
      .def_rw("neighbor_points", &form::KeypointExtractionParams::neighbor_points)
      .def_rw("num_sectors", &form::KeypointExtractionParams::num_sectors)
      .def_rw("planar_feats_per_sector",
              &form::KeypointExtractionParams::planar_feats_per_sector)
      .def_rw("planar_threshold", &form::KeypointExtractionParams::planar_threshold)
      .def_rw("point_feats_per_sector",
              &form::KeypointExtractionParams::point_feats_per_sector)
      // Parameters for normal estimation
      .def_rw("radius", &form::KeypointExtractionParams::radius)
      .def_rw("min_points", &form::KeypointExtractionParams::min_points)
      // Based on LiDAR info
      .def_rw("min_norm_squared", &form::KeypointExtractionParams::min_norm_squared)
      .def_rw("max_norm_squared", &form::KeypointExtractionParams::max_norm_squared)
      .def_rw("num_rows", &form::KeypointExtractionParams::num_rows)
      .def_rw("num_columns", &form::KeypointExtractionParams::num_columns);

  m.def("extract_keypoints", [](const std::vector<Eigen::Vector3d> &points,
                                const form::KeypointExtractionParams &params,
                                evalio::LidarParams &lidar_params) {
    // Call the keypoint extraction function from the Tclio class
    auto [planar_keypoints, point_keypoints] =
        form::extract_keypoints(points, params, 0);

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
