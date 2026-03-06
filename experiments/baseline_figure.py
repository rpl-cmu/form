from env import (
    SHORT_DATASET_NAMES,
    WINDOW_SMALL,
    compute_and_cache,
    plt_show,
    pretty_pipe_names,
    setup_plot,
    WINDOW_LARGE,
    COL_WIDTH,
)

import polars as pl
import seaborn as sns
import matplotlib.pyplot as plt


names = ["form", "kiss", "genz", "mad", "ct"]

c = setup_plot()
fig, ax = plt.subplots(1, 2, figsize=(COL_WIDTH * 2, 3.5), layout="constrained")

df_small = compute_and_cache(WINDOW_SMALL, pipes=names)
df_big = compute_and_cache(WINDOW_LARGE, pipes=names)

h = {p: pretty_pipe_names(p, False) for p in names}
df_small = df_small.with_columns(
    pl.col("name").replace(h).alias("Pipeline"),
    pl.col("dataset").replace(SHORT_DATASET_NAMES).alias("Dataset"),
)
df_big = df_big.with_columns(
    pl.col("name").replace(h).alias("Pipeline"),
    pl.col("dataset").replace(SHORT_DATASET_NAMES).alias("Dataset"),
)

c = {h[p]: c[p] for p in names}

sns.barplot(
    data=df_small,
    x="Dataset",
    y="RTEt_1.0m",
    hue="Pipeline",
    dodge=True,
    estimator="mean",
    errorbar=None,
    ax=ax[0],
    legend=False,
    palette=c,
)
ax[0].set_ylabel("1.0 RTEt (m)")
ax[0].set_yscale("symlog", linthresh=0.2, linscale=4.0)
ax[0].set_yticks([0.1, 0.2, 0.5, 1.0], labels=["0.1", "0.2", "0.5", "1.0"])
ax[0].tick_params(axis="both", pad=-2.5)

sns.barplot(
    data=df_big,
    y="RTEt_30.0m",
    # data=df_small,
    # y="RTEt_1.0m",
    x="Dataset",
    hue="Pipeline",
    dodge=True,
    estimator="mean",
    errorbar=None,
    ax=ax[1],
    palette=c,
)
ax[1].set_ylabel("30.0 RTEt (m)")
ax[1].set_yscale("symlog", linthresh=2.0, linscale=4.0)
ax[1].set_yticks([1, 2, 5, 10], labels=["1", "2", "5", "10"])
ax[1].tick_params(axis="both", pad=-2.5)

plt_show("baseline")
