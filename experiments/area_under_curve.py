import polars as pl
import seaborn as sns
import matplotlib.pyplot as plt

from env import (
    COL_WIDTH,
    pretty_pipe_names,
    compute_and_cache,
    setup_plot,
    WINDOW_SMALL,
    WINDOW_LARGE,
    plt_show,
)

c = setup_plot()
# plot the results
fig, ax = plt.subplots(
    1, 2, figsize=(COL_WIDTH + 0.5, 2.0), layout="constrained", sharey=True
)

limits = [0.9, 7.0]

for i, window in enumerate([WINDOW_SMALL, WINDOW_LARGE]):
    df = compute_and_cache(window)
    df = df.filter(pl.col("status").is_in(["success", "slow"]))

    # get all values to compute steps at
    rte = f"RTEt_{window.name()}"
    steps = df.select(pl.col(rte)).to_series().sort().to_list()

    pivot = df.pivot(on="name", index="sequence", values=rte).drop("sequence")
    num_trajectories = pivot.height

    # collect all the values
    results = {name: [] for name in pivot.columns}

    for s in steps:
        vals = (100 * (pivot < s).sum() / num_trajectories).to_dict(as_series=False)
        for name, val in vals.items():
            results[name].append(val[0])

    for n, vals in results.items():
        sns.lineplot(
            x=steps,
            y=vals,
            label=pretty_pipe_names(n, False),
            c=c[n],
            ax=ax[i],
        )

    ax[i].legend().remove()
    ax[i].set_xlim(0, limits[i])
    ax[i].set_xlabel(rf"RTE$_{{{int(window.value)}}}$ Threshold (m)")
    ax[i].set_ylabel("% Sequences Above Threshold")
    # ax[i].legend(
    #     loc="lower right",
    # )

    ax[i].tick_params(axis="x", pad=-2.5)
    ax[i].tick_params(axis="y", pad=-2.5)

handles, labels = ax[0].get_legend_handles_labels()
leg = fig.legend(
    handles=handles,
    labels=labels,
    ncols=5,
    borderpad=0.2,
    labelspacing=0.0,
    loc="outside upper left",
    columnspacing=1.5,
    bbox_to_anchor=(0.01, -0.02),
).get_frame()
leg.set_boxstyle("square")  # type: ignore
leg.set_linewidth(1.0)


plt_show("curve")
