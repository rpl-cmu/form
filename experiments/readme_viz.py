from pathlib import Path
from typing import cast
from evalio import datasets as ds, types as ty, pipelines as pl
from tqdm import tqdm
import pyvista as pv
import seaborn as sns
import numpy as np
from time import time
import pickle

from itertools import chain

from env import GRAPHICS_DIR

# ------------------------- Config! ------------------------- #
dataset = ds.OxfordSpires.observatory_quarter_01

start = 450
end = 572

window_size_big = [1000, 600]
window_size_small = [600, 600]

sphere_config = {
    "theta_resolution": 16,
    "phi_resolution": 16,
}
sphere_map_radius = 0.07
sphere_feat_radius = 0.1

cam_vals = {
    "distance": 90.0,
    "azimuth": 160.0,
    "elevation": 25.0,
}

far_focal_point = (-7, 5, 0)
zoomed_focal_point = (-16, 11.5, 0)
zoom_big = 1.25
zoom_small = 8.0

c = sns.color_palette("colorblind")[:3]


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
    window_size=window_size_big,
    lighting="three lights",
)
pose = pose * dataset.imu_T_lidar()
R = pose.rot.toMat()
t = pose.trans

# plot map
map_sphere = pv.Sphere(radius=sphere_map_radius, **sphere_config)  # type: ignore
map_points = list(chain.from_iterable(global_map.values()))
map_colors = np.vstack(
    [np.tile(c[p.col % len(c)], (map_sphere.n_points, 1)) for p in map_points]
)
map_points = np.array([[p.x, p.y, p.z] for p in map_points])
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

# plot features
feat_sphere = pv.Sphere(radius=sphere_feat_radius, **sphere_config)  # type: ignore
feat_points = list(chain.from_iterable(features.values()))
feat_points = np.array([[p.x, p.y, p.z] for p in feat_points])
feat_points = feat_points @ R.T + t
feat_pv = pv.PolyData(feat_points).glyph(
    geom=feat_sphere,
    scale=False,
    orient=False,
)
feat_pv = cast(pv.PolyData, feat_pv)
pl.add_mesh(
    feat_pv,
    color="k",
    smooth_shading=True,
    render=False,
)

# set camera location
pl.camera.zoom(zoom_big)
set_camera_position(
    pl,
    focal_point=far_focal_point,
    **cam_vals,
)
# pl.show_axes()  # type: ignore
pl.screenshot(GRAPHICS_DIR / "readme_big.png")

pl.window_size = window_size_small
set_camera_position(
    pl,
    focal_point=zoomed_focal_point,
    **cam_vals,
)
pl.camera.zoom(zoom_small)
pl.update()
pl.screenshot(GRAPHICS_DIR / "readme_small.png")

time_elapsed = time() - start
print(f"Rendered in {time_elapsed:.2f} seconds")
