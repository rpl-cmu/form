<div align="center">
<h1>FORM</h1>
<b>F</b>ixed-Lag <b>O</b>dometry with <b>R</b>eparative <b>M</b>apping
<br />
<br />
<a href="https://github.com/rpl-cmu/form/releases"><img src="https://img.shields.io/github/v/release/rpl-cmu/form" /></a>
<a href="https://github.com/rpl-cmu/form/actions/workflows/ci.yml"><img src="https://img.shields.io/github/actions/workflow/status/rpl-cmu/form/ci.yml" /></a>
<a href="https://github.com/rpl-cmu/form/blob/master/LICENSE"><img src="https://img.shields.io/github/license/rpl-cmu/form" /></a>
<a href="https://arxiv.org/abs/2510.09966"><img src="https://img.shields.io/badge/arXiv-2510.09966-b31b1b.svg" /></a>
<br />
<br />
FORM is a LiDAR Odometry system that performs fixed-lag **smoothing** and sub-map **reparations**, all in **real-time** with minimal parameters.
<br />
<br />

![](.github/assets/readme.png)

</div>
<hr />

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
@misc{potokar2025formfixedlagodometryreparative,
  title={FORM: Fixed-Lag Odometry with Reparative Mapping utilizing Rotating LiDAR Sensors}, 
  author={Easton R. Potokar and Taylor Pool and Daniel McGann and Michael Kaess},
  year={2025},
  eprint={2510.09966},
  archivePrefix={arXiv},
  primaryClass={cs.RO},
  url={https://arxiv.org/abs/2510.09966}, 
}
```