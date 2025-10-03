from pathlib import Path
from evalio.stats import WindowKind

import matplotlib.pyplot as plt
import matplotlib
import os
import seaborn as sns

import shutil
import subprocess

import polars as pl
from evalio.cli.stats import evaluate

from typing import Optional


WINDOW_KIND = WindowKind.distance
WINDOW_SMALL = 1.0
WINDOW_LARGE = 30.0

FINAL_DIR = Path("evalio_results/25.08.26_final")

FIGURE_DIR = Path("figures")
GRAPHICS_DIR = Path("graphics")

TEXT_WIDTH = 516.0 / 72.27
COL_WIDTH = 252.0 / 72.27

MARKERS = ["s", "X", "o"]

DATASETS = [
    "newer_college_2020",
    "newer_college_2021",
    "hilti_2022",
    "oxford_spires",
    "multi_campus",
    "cumulti",
    "botanic_garden",
]

PIPES = [
    "form",
    "kiss",
    "genz",
    "mad",
    "ct",
]


def pretty_pipe_names(name, in_latex: bool = True) -> str:
    return {
        "form": r"\acs{ours}" if in_latex else "FORM",
        "kiss": "KISS",
        "genz": "GenZ",
        "mad_slow": "MAD",
        "mad": "MAD",
        "ct": "CT",
        "ct_2022": "CT",
        "ct_2022_defaults_2": "CT",
        "gt": "GT",
    }[name]


SHORT_DATASET_NAMES = {
    "newer_college_2020": "N20",
    "newer_college_2021": "N21",
    "hilti_2022": "H22",
    "oxford_spires": "OS",
    "multi_campus": "MC",
    "cumulti": "CU",
    "botanic_garden": "BG",
}


def plt_show(name: str):
    plt.savefig(FIGURE_DIR / f"{name}.png", dpi=300, bbox_inches="tight")
    plt.savefig(GRAPHICS_DIR / f"{name}.pdf", dpi=300, bbox_inches="tight")
    if not is_remote():
        plt.show()


def is_remote() -> bool:
    return os.environ.get("SSH_CONNECTION") is not None


def setup_plot() -> dict[str, tuple[float, float, float]]:
    matplotlib.rc("pdf", fonttype=42)
    sns.set_context("paper")
    sns.set_style("whitegrid")
    sns.set_palette("colorblind")
    c = sns.color_palette("colorblind")

    # Make sure you install times & clear matplotlib cache
    # https://stackoverflow.com/a/49884009
    plt.rcParams["font.family"] = "Times New Roman"
    plt.rcParams["mathtext.fontset"] = "stix"

    return {
        "form": c[0],
        "kiss": c[1],
        "genz": c[2],
        "mad_slow": c[3],
        "mad": c[3],
        "ct": c[4],
        "ct_2022": c[4],
        "ct_2022_defaults_2": c[4],
        "gt": c[7],
    }


def format_latex(file: Path):
    if shutil.which("latexindent"):
        subprocess.run(
            ["latexindent", "-s", str(file), "--output", str(file)],
            check=True,
        )
    else:
        print(f"latexindent not found, skipping formatting for {file}")


def short(f: str) -> str:
    if f.isdigit():
        return f[-2:]
    else:
        return f[:1]


def sequence_pretty_names(seq) -> str:
    # for cumulti
    if "_robot" in seq:
        seq = seq[:-1] + "_" + seq[-1]
    # for botanic gardens
    if seq.startswith("b1"):
        seq = seq[1:]
    seq = seq.replace("-", "_")
    return "".join(short(d) for d in seq.split("_"))


lidar_rates = {
    "newer_college_2020": 10.0,
    "newer_college_2021": 10.0,
    "hilti_2022": 10.0,
    "oxford_spires": 10.0,
    "multi_campus": 10.0,
    "cumulti": 20.0,
    "botanic_garden": 10.0,
}
# Provide a smidge of leeway for handling overhead of evalio conversions
lidar_rates = {k: v - 0.5 for k, v in lidar_rates.items()}


def compute_and_cache(
    window_size: float, pipes: Optional[list[str]] = None
) -> pl.DataFrame:
    if pipes is None:
        pipes = PIPES

    file = FINAL_DIR / f"results_{window_size:.1f}_{WINDOW_KIND.value}.csv"
    if file.exists():
        print(f"loading cached results from {file}")
        df = pl.read_csv(file)

    else:
        results = evaluate(
            [str(FINAL_DIR)],
            quiet=True,
            window_kind=WINDOW_KIND,
            window_size=window_size,
        )
        # Replace all mentions of "--" in results with None
        for row in results:
            for key, value in row.items():
                if value == "--":
                    row[key] = None

        df = pl.DataFrame(results)
        print(f"Saving results to {file}")
        df.write_csv(file)

    # clean up a few things
    df = df.filter(pl.col("name").is_in(pipes) & pl.col("dataset").is_in(DATASETS))
    df = (
        df.lazy()
        .with_columns(
            pl.col("sequence")
            .map_elements(
                lambda seq: sequence_pretty_names(seq), return_dtype=pl.String
            )
            .alias("seq"),
            status=pl.when((pl.col.status != "complete") | pl.col.RTEt.is_nan())
            .then(pl.lit("fail"))
            .otherwise(
                pl.when(
                    pl.col.Hz
                    < pl.col.dataset.replace(lidar_rates, return_dtype=pl.Float64)
                )
                .then(pl.lit("slow"))
                .otherwise(pl.lit("success"))
            ),
        )
        .collect()
    )

    df = df.sort(
        pl.col("dataset").cast(pl.Enum(DATASETS)),
        "seq",
        pl.col("name").cast(pl.Enum(pipes)),
    )

    return df


def compute_avg(
    df: pl.DataFrame, d: str, values: str = "RTEt", pipes: Optional[list[str]] = None
) -> tuple:
    if pipes is None:
        pipes = PIPES

    this = df.filter(pl.col("dataset").eq(d))
    pivot = this.pivot(on="name", index="seq", values=values)
    return pivot.select(pl.mean(*pipes)).row(0)
