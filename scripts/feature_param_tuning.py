from typing import cast
import uuid
import gradio as gr
import rerun as rr
from gradio_rerun import Rerun

import numpy as np
from evalio.types import LidarParams
from evalio import datasets as ds
from pathlib import Path

from form import extract_keypoints, KeypointExtractionParams


recording_id = uuid.uuid4()
default_trajectory = ds.OxfordSpires.sequences()[0]


def update_sequences(dataset_name: str) -> gr.Dropdown:
    choices = [
        s.seq_name
        for s in ds.get_dataset(dataset_name).sequences()  # type: ignore
    ]
    return gr.Dropdown(
        choices=choices, label="Sequence", value=choices[0], interactive=True
    )


def update_index(dataset_name: str, sequence_name: str) -> gr.Slider:
    dataset = ds.get_sequence(f"{dataset_name}/{sequence_name}")
    dataset = cast(ds.Dataset, dataset)

    return gr.Slider(
        0,
        len(dataset),
        value=0,
        label="Index",
        step=1,
        interactive=True,
    )


def load_scan(
    dataset_name: str, sequence_name: str, index: int
) -> tuple[np.ndarray, LidarParams]:
    dataset = ds.get_sequence(f"{dataset_name}/{sequence_name}")
    dataset = cast(ds.Dataset, dataset)

    # check if there's a cache for it before loading
    if (path := Path(".cache") / dataset.full_name / f"{index}.npy").exists():
        print(f"Loading scan from cache: {path}")
        scan = np.load(path)
    else:
        print(f"Loading scan from dataset: {dataset_name}/{sequence_name}/{index}")
        scan = dataset.get_one_lidar(index).to_vec_positions()
        scan = cast(np.ndarray, scan)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.save(path, scan)
        print("Loaded scan with points.")

    return scan, dataset.lidar_params()


def update_keypoints(
    dataset_name: str,
    sequence_name: str,
    index: int,
    neighbor_points: int,
    num_sectors: int,
    planar_feats_per_sector: int,
    planar_threshold: float,
    point_feats_per_sector: int,
    radius: float,
    min_points: int,
):
    scan, params_lidar = load_scan(dataset_name, sequence_name, index)

    params_extract = KeypointExtractionParams()
    params_extract.neighbor_points = neighbor_points
    params_extract.num_sectors = num_sectors
    params_extract.planar_feats_per_sector = planar_feats_per_sector
    params_extract.planar_threshold = planar_threshold
    params_extract.point_feats_per_sector = point_feats_per_sector
    params_extract.radius = radius
    params_extract.min_points = min_points
    params_extract.min_norm_squared = params_lidar.min_range**2
    params_extract.max_norm_squared = params_lidar.max_range**2

    planar, normals, points = extract_keypoints(
        scan,  # type: ignore
        params_extract,
        params_lidar,
    )
    print(f"Extracted {len(planar)} planar and {len(points)} keypoints.")

    rec = rr.RecordingStream(
        application_id="rerun_example_gradio", recording_id=recording_id
    )
    rec.log(
        "planar",
        rr.Arrows3D(
            vectors=np.asarray(normals),
            origins=np.asarray(planar),
            colors=np.asarray([0, 255, 0]),
        ),
    )
    rec.log(
        "points",
        rr.Points3D(
            positions=np.asarray(points),
            colors=np.asarray([255, 0, 0]),
        ),
    )
    rec.log(
        "scan",
        rr.Points3D(
            positions=np.asarray(scan),
            colors=np.asarray([0, 255, 255]),
            radii=np.array([0.01]),
        ),
    )
    stream = rec.binary_stream()
    yield stream.read()


# name, min, max, step
params = KeypointExtractionParams()
sliders = [
    ["neighbor_points", 0, 10, 1],
    ["num_sectors", 1, 10, 1],
    ["planar_feats_per_sector", 5, 70, 5],
    ["planar_threshold", 0.0, 2.0, 0.01],
    ["occlusion_thresh", 0.0, 200.0, 0.01],
    ["parallel_thresh", 0.0, 2.0, 0.01],
    ["point_feats_per_sector", 1, 100, 1],
    # normal extraction parameters
    ["radius", 0.0, 10.0, 0.01],
    ["min_points", 1, 10, 1],
]
all_datasets = list(ds.all_datasets().keys())

with gr.Blocks(
    fill_height=True, fill_width=True, css="footer {visibility: hidden}"
) as demo:
    with gr.Sidebar():
        sliders_dataset = [
            gr.Dropdown(
                choices=all_datasets,
                label="Dataset",
                value=default_trajectory.dataset_name(),
                interactive=True,
            ),
            update_sequences(default_trajectory.dataset_name()),
            update_index(
                default_trajectory.dataset_name(), default_trajectory.seq_name
            ),
        ]
        sliders_params = [
            gr.Slider(
                min_val,
                max_val,
                value=getattr(params, name),
                label=name,
                step=step,
                interactive=True,
            )
            for name, min_val, max_val, step in sliders
        ]

    with gr.Row():
        viewer = Rerun(
            streaming=True,
            height="95vh",
            panel_states={
                "time": "collapsed",
                "blueprint": "expanded",
                "selection": "expanded",
                "top": "hidden",
            },
        )

    # Hook everything up
    sliders_dataset[0].change(
        update_sequences, inputs=[sliders_dataset[0]], outputs=[sliders_dataset[1]]
    )
    sliders_dataset[1].change(
        update_index,
        inputs=[sliders_dataset[0], sliders_dataset[1]],
        outputs=[sliders_dataset[2]],
    )
    for s in sliders_params + sliders_dataset[-1:]:
        s.release(
            update_keypoints,
            inputs=sliders_dataset + sliders_params,
            outputs=[viewer],
        )


if __name__ == "__main__":
    demo.launch(ssr_mode=False)
