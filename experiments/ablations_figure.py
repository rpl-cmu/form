# pyrefly: ignore [missing-import]
from env import (
    SHORT_DATASET_NAMES,
    WINDOW_SMALL,
    compute_and_cache,
    plt_show,
    setup_plot,
    WINDOW_LARGE,
    COL_WIDTH,
)

import polars as pl
import seaborn as sns
import matplotlib.pyplot as plt


pipes = ["form", "form_single", "form_planar"]
header = ["FORM", "Filtered", "Planar"]
exp = "ablation"

c = setup_plot()
fig, ax = plt.subplots(1, 2, figsize=(6, 2.25), layout="constrained")

df_small = compute_and_cache(WINDOW_SMALL, pipes=pipes)
df_big = compute_and_cache(WINDOW_LARGE, pipes=pipes)

h = {p: h for p, h in zip(pipes, header)}
df_small = df_small.with_columns(
    pl.col("name").replace(h).alias("Ablation"),
    pl.col("dataset").replace(SHORT_DATASET_NAMES).alias("Dataset"),
)
df_big = df_big.with_columns(
    pl.col("name").replace(h).alias("Ablation"),
    pl.col("dataset").replace(SHORT_DATASET_NAMES).alias("Dataset"),
)

sns.barplot(
    data=df_small,
    x="Dataset",
    y="RTEt_1.0m",
    hue="Ablation",
    dodge=True,
    estimator="mean",
    errorbar=None,
    ax=ax[0],
    legend=False,
    palette=[c["form"], c["ab"], c["ab2"]],
)
ax[0].set_ylabel("1.0 RTEt (m)")
sns.barplot(
    data=df_big,
    y="RTEt_30.0m",
    # data=df_small,
    # y="RTEt_1.0m",
    x="Dataset",
    hue="Ablation",
    dodge=True,
    estimator="mean",
    errorbar=None,
    ax=ax[1],
    palette=[c["form"], c["ab"], c["ab2"]],
)
ax[1].set_ylabel("30.0 RTEt (m)")

for i in range(2):
    for i, bar in enumerate(ax[i].patches):
        if i >= 7:
            cr, cg, cb, _ = bar.get_facecolor()
            bar.set_facecolor((cr, cg, cb, 0.5))
            bar.set_edgecolor((cr, cg, cb, 0.7))

plt_show(exp)
