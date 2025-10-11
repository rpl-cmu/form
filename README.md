<div align="center">
<h1>FORM</h1>
<h3 style="font-weight: normal"><b>F</b>ixed-Lag <b>O</b>dometry with <b>R</b>eparative <b>M</b>apping</h4>
<!-- <a href="https://github.com/rpl-cmu/form/blob/master/LICENSE"><img src="https://img.shields.io/github/license/rpl-cmu/form" /></a> -->
</div>

![](.github/assets/readme.png)

FORM is a LiDAR Odometry system that performs fixed-lag smoothing over a window of prior poses along with map reparations, all in real-time with minimal parameters.

## Building

FORM has minimal dependencies. You will need:
- [eigen3](https://libeigen.gitlab.io/eigen/docs-nightly/)
- [gtsam](https://github.com/borglab/gtsam/)
- [tbb](https://github.com/uxlfoundation/oneTBB)
- [tsl::robin_map](https://github.com/Tessil/robin-map) (will be pulled from source if not found)

After that, building is as simple as:
```bash
mkdir build
cd build
cmake ..
make
```

## Running Experiments from Source
All experiments are ran through [evalio](https://github.com/contagon/evalio/tree/master), our internal LiDAR-inertial odometry evaluation tool. Downloading the desired datasets is done as:
```bash
evalio dl newer_college_2020/*
```
FORM can then be installed as a python package (preferably in a virtual environment):
```bash
pip install -e .  # for pip usage
uv sync --verbose # for uv usage
```

Finally, experiments can be run as (with the venv activated if needed):
```bash
evalio run -M form -c config/25.10.03_full.yaml
```
Datasets can be commented out in the config file if they haven't been downloaded. Results are saved in `evalio_results/25.10.03_full/` and stats can be viewed as:
```bash
evalio stats evalio_results/25.10.03_full/
```

## Roadmap
- [ ] ROS Node
- [ ] Merge wrapper into upstream evalio

## Citation
If you use FORM for any academic work, please cite:
```
TODO
```