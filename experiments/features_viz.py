from pathlib import Path
from typing import cast
from evalio import datasets as ds, types as ty, pipelines as pl
from tqdm import tqdm
import pyvista as pv
import seaborn as sns
import numpy as np
from time import time
import pickle

from env import GRAPHICS_DIR

# ------------------------- Config! ------------------------- #
dataset = ds.MultiCampus.ntu_night_13

start = 1550
end = 1650

window_size = [1500, 700]

sphere_config = {
    "theta_resolution": 16,
    "phi_resolution": 16,
}
sphere_map_radius = 0.12

cam_vals = {
    "distance": 100.0,
    "azimuth": 150.0,
    "elevation": 23.0,
    "focal_point": (60, 5, 0),
}


c = sns.color_palette("colorblind")


def set_camera_position(
    pl: pv.Plotter,
    focal_point: tuple[float, float, float],
    distance: float,
    azimuth: float,
    elevation: float,
):
    """Set the camera position given spherical coordinates."""
    pl.camera.focal_point = focal_point
    azimuth = np.radians(azimuth)
    elevation = np.radians(elevation)
    x = focal_point[0] + distance * np.cos(elevation) * np.cos(azimuth)
    y = focal_point[1] + distance * np.cos(elevation) * np.sin(azimuth)
    z = focal_point[2] + distance * np.sin(elevation)
    pl.camera.position = (x, y, z)


# ------------------------- Loop through to get map ------------------------- #
cache = Path(".cache") / dataset.full_name / f"{start}_{end}.pkl"
pipe = pl.get_pipeline("form")
if isinstance(pipe, Exception):
    raise pipe
out = ty.Experiment.from_pl_ds(pipe, dataset).setup()
if isinstance(out, Exception):
    raise out
pipe, dataset = out

global_map = {}
features = None
pose = None

if cache.exists():
    print(f"Loading from {cache}")
    with open(cache, "rb") as f:
        global_map, features, pose = pickle.load(f)

else:
    curr_idx = 0
    loop = tqdm(total=end)
    for data in dataset.lidar():
        if isinstance(data, ty.LidarMeasurement):
            # visualize in rerun
            if curr_idx > start:
                # feed through pipeline
                features = pipe.add_lidar(data)
                pose = pipe.pose()
                global_map = pipe.map()

            curr_idx += 1
            loop.update(1)
            if curr_idx >= end:
                break

    loop.close()

    cache.parent.mkdir(parents=True, exist_ok=True)
    pickle.dump((global_map, features, pose), open(cache, "wb"))

if features is None or global_map is None or pose is None:
    raise ValueError("No features or map found")

# ------------------------- Visualize ------------------------- #
start = time()
pl = pv.Plotter(
    off_screen=True,
    window_size=window_size,
    lighting="three lights",
)
pose = pose * dataset.imu_T_lidar()
R = pose.rot.toMat()
t = pose.trans

# plot map
map_sphere = pv.Sphere(radius=sphere_map_radius, **sphere_config)  # type: ignore
for i, feat_map in enumerate(global_map.values()):
    #
    map_points = np.array(
        [[p.x, p.y, p.z] for p in feat_map if not (p.x < 10.0 and p.z > 2.0)]
    )
    map_colors = np.tile(c[i], (len(map_points) * map_sphere.n_points, 1))
    map_pv = pv.PolyData(map_points).glyph(
        geom=map_sphere,
        scale=False,
        orient=False,
    )
    map_pv = cast(pv.PolyData, map_pv)
    pl.add_mesh(
        map_pv,
        scalars=map_colors,
        rgb=True,
        smooth_shading=True,
        render=False,
    )

# set camera location
set_camera_position(pl, **cam_vals)
# pl.show_axes()  # type: ignore

# pl.show(auto_close=False)
pl.screenshot(GRAPHICS_DIR / "features.png")

time_elapsed = time() - start
print(f"Rendered in {time_elapsed:.2f} seconds")
