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
  FORM() : evalio::Pipeline(), estimator_(form::Estimator::Params()), params_() {}

  form::Estimator estimator_;
  form::Estimator::Params params_;

  // helper params
  gtsam::Pose3 lidar_T_imu_ = gtsam::Pose3::Identity();
  evalio::Duration delta_time_;
  evalio::SE3 current_pose = evalio::SE3::identity();

  // ------------------------- Info ------------------------- //
  static std::string name() { return "form"; }

  static std::string url() { return "https://github.com/rpl-cmu/form"; }

  static std::map<std::string, evalio::Param> default_params() {
    return {
        {"max_dist_min", 0.1},
        {"max_dist_max", 0.8},
        {"max_dist_map", 0.1},
        {"new_pose_threshold", 1e-4},
        {"max_num_rematches", 30},
        // constraint params
        {"max_num_keyframes", 50},
        {"max_num_recent_frames", 10},
        {"max_steps_unused_keyframe", 10},
        {"keyscan_match_ratio", 0.1},
        // keypoint params
        {"neighbor_points", 5},
        {"num_sectors", 6},
        {"planar_feats_per_sector", 50},
        {"point_feats_per_sector", 3},
        {"planar_threshold", 1.0},
        {"radius", 1.0},
        {"min_points", 5},
        // misc
        {"num_threads", 0},
        {"disable_smoothing", false},
    };
  }

  // ------------------------- Getters ------------------------- //
  // Returns the most recent pose estimate
  const evalio::SE3 pose() override { return current_pose; }

  // Returns the current submap of the environment
  const std::map<std::string, std::vector<evalio::Point>> map() override {
    const auto world_map =
        tuple_transform(estimator_.m_keypoint_map, [&](auto &map) {
          return map.to_voxel_map(estimator_.m_constraints.get_values());
        });

    std::tuple<std::string, std::string> map_names = {"planar", "point"};
    std::map<std::string, std::vector<evalio::Point>> points;

    for_sequence(std::make_index_sequence<2>{}, [&](auto I) {
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
    params_.keypointExtraction.min_norm_squared =
        params.min_range * params.min_range;
    params_.keypointExtraction.max_norm_squared =
        params.max_range * params.max_range;
    params_.keypointExtraction.num_columns = params.num_columns;
    params_.keypointExtraction.num_rows = params.num_rows;
    delta_time_ = params.delta_time();
  }

  // Set the transformation from IMU to LiDAR
  void set_imu_T_lidar(evalio::SE3 T) override {
    lidar_T_imu_ = pose_to_gtsam(T).inverse();
  }

  // Set the custom parameters
  std::map<std::string, evalio::Param>
  set_params(std::map<std::string, evalio::Param> params) override {
    for (const auto &[key, value] : params) {
      // handle form parameters
      if (key == "max_dist_min") {
        params_.max_dist_min = std::get<double>(value);
      } else if (key == "max_dist_max") {
        params_.max_dist_max = std::get<double>(value);
      } else if (key == "new_pose_threshold") {
        params_.new_pose_threshold = std::get<double>(value);
      } else if (key == "max_num_rematches") {
        params_.max_num_rematches = std::get<int>(value);
      } else if (key == "max_dist_map") {
        params_.max_dist_map = std::get<double>(value);
      } else if (key == "num_threads") {
        params_.num_threads = std::get<int>(value);
      } else if (key == "disable_smoothing") {
        params_.constraints.disable_smoothing = std::get<bool>(value);
      }

      // handle constraint parameters
      else if (key == "max_num_keyframes") {
        params_.constraints.max_num_keyframes = std::get<int>(value);
      } else if (key == "max_steps_unused_keyframe") {
        params_.constraints.max_steps_unused_keyframe = std::get<int>(value);
      } else if (key == "max_num_recent_frames") {
        params_.constraints.max_num_recent_frames = std::get<int>(value);
      } else if (key == "keyscan_match_ratio") {
        params_.constraints.keyscan_match_ratio = std::get<double>(value);
      }

      // handle keypoint extraction parameters
      else if (key == "neighbor_points") {
        params_.keypointExtraction.neighbor_points = std::get<int>(value);
      } else if (key == "num_sectors") {
        params_.keypointExtraction.num_sectors = std::get<int>(value);
      } else if (key == "planar_feats_per_sector") {
        params_.keypointExtraction.planar_feats_per_sector = std::get<int>(value);
      } else if (key == "point_feats_per_sector") {
        params_.keypointExtraction.point_feats_per_sector = std::get<int>(value);
      } else if (key == "planar_threshold") {
        params_.keypointExtraction.planar_threshold = std::get<double>(value);
      } else if (key == "radius") {
        params_.keypointExtraction.radius = std::get<double>(value);
      } else if (key == "min_points") {
        params_.keypointExtraction.min_points = std::get<int>(value);
      } else {
        throw std::runtime_error("Unknown parameter: " + key);
      }
    }

    return {};
  }

  // ------------------------- Doers ------------------------- //
  // Initialize the pipeline
  void initialize() override { estimator_.reset(params_); }

  // Add an IMU measurement
  void add_imu(evalio::ImuMeasurement mm) override {}

  // Add a LiDAR measurement
  std::map<std::string, std::vector<evalio::Point>>
  add_lidar(evalio::LidarMeasurement mm) override {
    // convert to evalio
    std::vector<Eigen::Vector3f> scan;
    auto start = mm.stamp;
    auto end = mm.stamp + delta_time_;

    // TODO: Use start or end as the stamp?
    for (const auto &point : mm.points) {
      scan.push_back(point_to_form(point));
    }

    // run the estimator
    auto [planar_kp, point_kp] = estimator_.registerScan(scan);
    current_pose =
        pose_to_evalio(estimator_.current_lidar_estimate() * lidar_T_imu_);

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
  nb::class_<form::feature::KeypointExtractionParams>(m, "KeypointExtractionParams")
      .def(nb::init<>())
      .def_rw("neighbor_points",
              &form::feature::KeypointExtractionParams::neighbor_points)
      .def_rw("num_sectors", &form::feature::KeypointExtractionParams::num_sectors)
      .def_rw("planar_feats_per_sector",
              &form::feature::KeypointExtractionParams::planar_feats_per_sector)
      .def_rw("planar_threshold",
              &form::feature::KeypointExtractionParams::planar_threshold)
      .def_rw("point_feats_per_sector",
              &form::feature::KeypointExtractionParams::point_feats_per_sector)
      // Parameters for normal estimation
      .def_rw("radius", &form::feature::KeypointExtractionParams::radius)
      .def_rw("min_points", &form::feature::KeypointExtractionParams::min_points)
      // Based on LiDAR info
      .def_rw("min_norm_squared",
              &form::feature::KeypointExtractionParams::min_norm_squared)
      .def_rw("max_norm_squared",
              &form::feature::KeypointExtractionParams::max_norm_squared)
      .def_rw("num_rows", &form::feature::KeypointExtractionParams::num_rows)
      .def_rw("num_columns", &form::feature::KeypointExtractionParams::num_columns);

  m.def("extract_keypoints",
        [](const std::vector<Eigen::Vector3d> &points,
           const form::feature::KeypointExtractionParams &params,
           evalio::LidarParams &lidar_params) {
          // Call the keypoint extraction function from the Tclio class
          auto [planar_keypoints, point_keypoints] =
              form::feature::extract_keypoints(points, params, 0);

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
