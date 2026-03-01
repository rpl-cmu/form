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


pipesss = [
    ["form_planar", "form"],
    ["form_single", "form"],
]
headers = [["Planar", "Point & Planar"], ["Filtered", "Smoothed"]]
experiments = ["feature", "smoothing"]

for pipes, header, exp in zip(pipesss, headers, experiments):
    c = setup_plot()
    fig, ax = plt.subplots(1, 2, figsize=(COL_WIDTH * 2, 4.0), layout="constrained")

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
        palette=[c["ab"], c["form"]],
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
        palette=[c["ab"], c["form"]],
    )
    ax[1].set_ylabel("30.0 RTEt (m)")
    # sns.barplot(
    #     data=df_big,
    #     x="ds",
    #     y="hz",
    #     hue="Ablation",
    #     dodge=True,
    #     estimator="mean",
    #     errorbar=None,
    #     ax=ax[1],
    # )
    plt_show(exp)
