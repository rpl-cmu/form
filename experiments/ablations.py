from env import (
    DATASETS,
    GRAPHICS_DIR,
    SHORT_DATASET_NAMES,
    WINDOW_SMALL,
    compute_and_cache,
    compute_avg,
    format_latex,
    setup_plot,
    WINDOW_LARGE,
)


c = setup_plot()

pipesss = [
    ["form_planar", "form"],
    ["form_single", "form"],
]
headers = [["Planar", r"Point \& Planar"], ["Filtered", "Smoothed"]]
experiments = ["feature", "smoothing"]

captions = [
    r"Ablation study comparing features used in \ac{ours}. Values are averaged across dataset sequences, with results in bold representing the best results. While the point and planar variant of \ac{ours} does not significantly improve smoothness, it does help with odometry drift at a performance cost.",
    r"Ablation study showing the effects of smoothing in \ac{ours}. Values are averaged across dataset sequences, with results in bold representing the best results. Smoothing improves the drift performance of \ac{ours} but has limited impact on the smoothness of the trajectory.",
]

files = [
    GRAPHICS_DIR / "table_ablation_features.tex",
    GRAPHICS_DIR / "table_ablation_smoothing.tex",
]


def mark_mins(val0, val1):
    """Mark the minimum value in each list."""
    # If they're the same
    if round(val0, 2) == round(val1, 2):
        return [f"\\textbf{{{val0:.2f}}}", f"\\textbf{{{val1:.2f}}}"]
    elif val0 < val1:
        return [f"\\textbf{{{val0:.2f}}}", f"{val1:.2f}"]
    else:
        return [f"{val0:.2f}", f"\\textbf{{{val1:.2f}}}"]


for file, caption, pipes, header, exp in zip(
    files, captions, pipesss, headers, experiments
):
    with open(file, "w") as f:
        df_small = compute_and_cache(WINDOW_SMALL, pipes=pipes)
        df_big = compute_and_cache(WINDOW_LARGE, pipes=pipes)

        columns = df_small.select("name").unique().to_series().to_list()
        f.write(rf"""
            % chktex-file 44
            \begin{{table}}
            \centering
            \caption{{{caption}}}\label{{tab:ablation_{exp}}}
            \begin{{tabular}}{{c||{"c" * 3 * len(columns)}}}
            \toprule
            D. & \multicolumn{{3}}{{c}}{{{header[0]}}} & \multicolumn{{3}}{{c}}{{{header[1]}}} \\
            \midrule
            & \rte{{{WINDOW_SMALL.value:.0f}}} & \rte{{{WINDOW_LARGE.value:.0f}}} & Hz & \rte{{{WINDOW_SMALL.value:.0f}}} & \rte{{{WINDOW_LARGE.value:.0f}}} & Hz \\
            \midrule
        """)

        for d in DATASETS:
            vals_small = compute_avg(
                df_small, d, values=f"RTEt_{WINDOW_SMALL.name()}", pipes=pipes
            )
            vals_big = compute_avg(
                df_big, d, values=f"RTEt_{WINDOW_LARGE.name()}", pipes=pipes
            )
            hz = compute_avg(df_small, d, "hz", pipes=pipes)

            # convert to strings, bolding the smaller one
            vals_small = mark_mins(*vals_small)
            vals_big = mark_mins(*vals_big)
            hz = [f"{hz[0]:.1f}", f"{hz[1]:.1f}"]

            f.write(
                f"{SHORT_DATASET_NAMES[d]} & {vals_small[0]} & {vals_big[0]} & {hz[0]} & {vals_small[1]} & {vals_big[1]} & {hz[1]} \\\\ \n"
            )

        f.write(r"""
            \bottomrule
            \end{tabular}
            \end{table}
        """)
    print(file)
    format_latex(file)
