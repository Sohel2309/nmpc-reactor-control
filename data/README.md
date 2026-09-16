# data/

This project's scenarios (setpoint trajectories, feed-disturbance
sequences, noise levels) are defined as code in `src/scenarios.py`, not
as standalone data files, because they are generated programmatically
and must stay in sync with `config/experiment_config.yaml`.

`scenario_definitions.json` in this directory is a **generated snapshot**
(see `scripts/` -- regenerate by re-running the small export snippet in
the project's development notes, or just read `src/scenarios.py`
directly) of every named scenario used by the experiment scripts, for
quick human inspection without reading Python. It is not read by any
script in this repository; `src/scenarios.py` is the single source of
truth.
