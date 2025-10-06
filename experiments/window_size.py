from evalio.cli.stats import evaluate
from evalio.types import Trajectory, SE3, SO3
from evalio.stats import align, WindowMeters
import numpy as np
import matplotlib.pyplot as plt

from env import (
    COL_WIDTH,
    pretty_pipe_names,
    WINDOW_SMALL,
    FINAL_DIR,
    plt_show,
    setup_plot,
)

c = setup_plot()
metric = "RTEt"
data = FINAL_DIR / "oxford_spires" / "blenheim_palace_02"

names = [
    "form",
    "kiss",
    "genz",
]

# Rotate the trajectories to make them horizontal
degrees = 94
ROT = SE3(rot=SO3.exp(np.array([0.0, 0.0, degrees * np.pi / 180.0])), trans=np.zeros(3))

fig, ax = plt.subplots(
    4,
    1,
    figsize=(COL_WIDTH + 0.5, 3.0),
    layout="constrained",
    gridspec_kw={"height_ratios": [4, 1, 1, 1]},
)

# ------------------------- First plot the window size computations ------------------------- #
# gather the results
windows = np.arange(WINDOW_SMALL.value, 18.0, 2.0)
final_results = {name: [] for name in names}
for w in windows:
    results = evaluate([data], windows=[WindowMeters(float(w))])
    print(f"Evaluating window size {w} with {len(results)} results")
    for r in results:
        if r["name"] in final_results:
            final_results[r["name"]].append(r[metric] * 10)

# plot the results
ax[0].plot([], [], marker="o", color=c["gt"], label=pretty_pipe_names("gt", False))
for i, (key, values) in enumerate(final_results.items()):
    ax[0].plot(
        windows, values, marker="o", color=c[key], label=pretty_pipe_names(key, False)
    )

ax[0].tick_params(axis="x", pad=-2.0)
ax[0].tick_params(axis="y", pad=-2.0)

ax[0].set_xlabel("Window Size $j$ (m)", labelpad=0.75)
ax[0].set_ylabel(r"RTEt$_j$ (cm)", labelpad=1.0)

# ------------------------- Next plot the rough trajectory ------------------------- #
rough_indices = slice(1185, 1242)

# load all the data
trajectories = [Trajectory.from_file(data / f"{name}.csv") for name in names]
gt = Trajectory.from_file(data / "gt.csv")
if isinstance(gt, Exception):
    print("Missing ground truth trajectory, quitting.")
    exit()

trajectories = {}
for n in names:
    t = Trajectory.from_file(data / f"{n}.csv")
    if isinstance(t, Exception):
        print(f"Missing trajectory for {n}, quitting.")
        exit()

    out = align(t, gt)
    if isinstance(out, Exception):
        print(f"Failed to align {n} to ground truth, quitting.")
        exit()
    gt_align, t = out

    offset = ROT * t.poses[rough_indices.start].inverse()
    xyz = np.asarray([(offset * pose).trans for pose in t.poses[rough_indices]])
    trajectories[n] = xyz

    if "gt" not in trajectories:
        offset = ROT * gt_align.poses[rough_indices.start].inverse()
        xyz = np.asarray(
            [(offset * pose).trans for pose in gt_align.poses[rough_indices]]
        )
        trajectories["gt"] = xyz

# The actually plotting
gt = trajectories["gt"]
for i, n in enumerate(names):
    t = trajectories[n]
    ax[i + 1].plot(gt[:, 0], gt[:, 1], color=c["gt"])
    ax[i + 1].scatter(t[:, 0], t[:, 1], color=c[n], s=1)

for i in range(1, 3):
    ax[i].set_xticklabels([])

for i in range(1, 4):
    ax[i].set_aspect("equal")
    ax[i].set_ylim(-0.35, 0.1)
    ax[i].set_yticks([-0.3, 0.0])
    ax[i].tick_params(axis="y", pad=-2.0)


ax[1].set_title("   ")
ax[-2].set_ylabel("y (m)", labelpad=0.5)
ax[-1].tick_params(axis="x", pad=-2.0)
ax[-1].set_xlabel("x (m)", labelpad=-0.25)

handles, labels = ax[0].get_legend_handles_labels()

leg = fig.legend(
    ncols=4,
    borderpad=0.2,
    labelspacing=0.0,
    loc="outside upper left",
    columnspacing=2.8,
    bbox_to_anchor=(0.102, -0.03),
).get_frame()
leg.set_boxstyle("square")  # type: ignore
leg.set_linewidth(1.0)

plt_show("window")
