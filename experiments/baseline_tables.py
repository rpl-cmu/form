import polars as pl
import numpy as np

from env import (
    DATASETS,
    GRAPHICS_DIR,
    PIPES,
    pretty_pipe_names,
    SHORT_DATASET_NAMES,
    WINDOW_SMALL,
    compute_and_cache,
    setup_plot,
    WINDOW_LARGE,
    format_latex,
)


names_to_include = [
    "form",
    "kiss",
    "genz",
    "mad",
    "ct",
]
pretty_names = [pretty_pipe_names(n) for n in names_to_include]


def format(value: str, status: str) -> str:
    """Format the value based on its status.

    Args:
        value (str): _description_
        status (str): _description_

    Returns:
        str: _description_
    """
    if status == "fail":
        value = rf"\textcolor{{slow}}{{\sout{{{value}}}}}"
    if status == "slow":
        value = rf"\textcolor{{slow}}{{\textit{{{value}}}}}"

    return value


def mark_min(values: tuple, status: tuple, outputs: list[str]):
    """Marks the minimum value in bold, excluding failed or slow runs.

    Args:
        values (tuple): tuple of values to check
        status (tuple): tuple of status strings corresponding to the values
        outputs (list[str]): list of output strings to modify

    Returns:
        _type_: _description_
    """
    indices = np.argsort(
        [v if s == "success" else np.inf for v, s in zip(values, status)]
    )
    outputs[indices[0]] = rf"\textcolor{{color0}}{{\textbf{{{outputs[indices[0]]}}}}}"
    outputs[indices[1]] = rf"\textcolor{{color0!80}}{{{outputs[indices[1]]}}}"

    # for i, (v, s) in enumerate(zip(values, status)):
    #     # if round(v, 2) == round(min_value, 2) and s == "success":
    #     if v == min_value and s == "success":
    #         outputs[i] = f"\\textbf{{{outputs[i]}}}"
    return outputs


c = setup_plot()

windows = [WINDOW_SMALL, WINDOW_LARGE]
caption = rf"Both \rte{{{WINDOW_SMALL.value:.0f}}} and \rte{{{WINDOW_LARGE.value:.0f}}} for all datasets. Strike-throughs indicate a failure to complete, italics sub-real-time performance, \textcolor{{color0}}{{\textbf{{bold blue}}}} the best real-time result, and \textcolor{{color0!80}}{{light blue}} the second best. \ac{{ours}} has competitive drift performance while still maintaining a smooth trajectory and real-time speed."


file = GRAPHICS_DIR / "table_baselines.tex"


with open(file, "w") as f:
    f.write(rf"""
        % chktex-file 44
        \begin{{table*}}
        \centering
        \caption{{{caption}}}\label{{tab:baseline_main}}
    """)

    for window in windows:
        f.write(rf"""
            \begin{{minipage}}{{.5\linewidth}}
            \centering
            \begin{{tabular}}{{l|c||{"c" * len(PIPES)}}}
            \toprule
			\multicolumn{{{2 + len(PIPES)}}}{{c}}{{\rte{{{window.value:.0f}}}}} \\
			\toprule
        """)
        df = compute_and_cache(window)
        columns = df.select("name").unique().to_series().to_list()

        f.write("D. & Seq. &" + " & ".join(pretty_names) + " \\\\ \n")

        for d in DATASETS:
            # print(d)
            this = df.filter(pl.col("dataset").eq(d))
            pivot = this.pivot(on="name", index="seq", values=f"RTEt_{window.name()}")
            pivot_status = this.pivot(on="name", index="seq", values="status")
            num_seq = len(pivot)

            f.write("\\midrule\n")
            f.write(
                r"\multirow[c]{"
                + str(num_seq)
                + r"}{*}{\rotatebox[origin=c]{90}{"
                + SHORT_DATASET_NAMES[d]
                + "}}"
            )
            f.write("\n")

            for row, status in zip(pivot.rows(), pivot_status.rows()):
                outputs = ["~~~~" if np.isnan(v) else f"{v:.2f}" for v in row[1:]]
                # italics for failed runs
                outputs = [format(v, s) for v, s in zip(outputs, status[1:])]
                # mark the minimums
                outputs = mark_min(row[1:], status[1:], outputs)
                f.write(f"& {row[0]} & " + " & ".join(outputs))
                f.write("\\\\\n")

        f.write(r"""
            \bottomrule
            \end{tabular}
            \end{minipage}%""")

    f.write("\n\\end{table*}")

format_latex(file)
