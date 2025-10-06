from env import (
    DATASETS,
    GRAPHICS_DIR,
    PIPES,
    pretty_pipe_names,
    SHORT_DATASET_NAMES,
    WINDOW_LARGE,
    compute_and_cache,
    compute_avg,
    format_latex,
    setup_plot,
)

import polars as pl

pretty_names = [pretty_pipe_names(n) for n in PIPES]

c = setup_plot()

caption = r"Average compute speed of the various pipelines in hertz, while excluding trajectories with \rte{30} greater than ten meters. Real-time performance is achieved above the \ac{lidar} sensor rate, \SI{10}{\hertz} for all datasets except CU-Multi which is \SI{20}{\hertz}. \ac{ours} consistently provides real-time performance."


file = GRAPHICS_DIR / "table_speed.tex"

with open(file, "w") as f:
    df = compute_and_cache(WINDOW_LARGE)
    columns = df.select("name").unique().to_series().to_list()
    f.write(rf"""
        % chktex-file 44
        \begin{{table}}
        \centering
        \caption{{{caption}}}\label{{tab:baseline_speed}}
        \begin{{tabular}}{{l||{"c" * len(columns)}}}
        \toprule
    """)
    f.write("D. & " + " & ".join(pretty_names) + " \\\\ \n\\midrule\n")

    df = df.filter(pl.col(f"RTEt_{WINDOW_LARGE.name()}") < 10.0)

    for d in DATASETS:
        avg = compute_avg(df, d, "hz")
        f.write(
            f"{SHORT_DATASET_NAMES[d]} & "
            + " & ".join(f"{v:.1f}" for v in avg)
            + " \\\\\n"
        )

    f.write(r"""
        \bottomrule
        \end{tabular}
        \end{table}
    """)

format_latex(file)
