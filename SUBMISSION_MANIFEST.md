# D435i RGB-D Visual SLAM Submission Manifest

## Provenance

- Upstream repository: `Zhuyicheng-HIT/multi-slam-simulation`
- Upstream base: `main` at `6d878b7b95770670a8ab5323bed4cecbbcf4c6fc`
- Submission branch: `feat/d435i-rgbd-visual-slam`
- Read-only source branch: `codex/d435i-visual-slam-baseline`
- Approved source commit: `727d6e036c435d75a87f58b84a0acc885b95ce9d`
- Common ancestor used for source inventory:
  `5508ef114577083438359b981d93a2edefcdfde9`

The source branch contained 69 changed files and 12,256 insertions relative to
its common ancestor. Files were selected by function and transplanted onto the
latest upstream main; the development history was not cherry-picked.

## Commit structure

1. `feat: add high-performance d435i rgbd bridge`
2. `feat: add d435i rtabmap visual slam profile`
3. `feat: add headless d435i simulation workflow`
4. `test: add d435i visual slam validation tools`
5. `docs: document d435i visual slam baseline`

## Included scope

- C++ RGB-D bridge package and Python fallback;
- exact-sync RTAB-Map profiles and launch adapter;
- D435i-only model, baseline/textured worlds and headless lifecycle scripts;
- bridge/RTAB performance, latency, ATE/RPE and robustness profilers;
- read-only RTAB-Map database diagnostics;
- A-G visual-friendly route, feature-alignment matrix and speed envelope tools;
- three core D435i documents, README entry and artifact ignore rules.

The authoritative path list is `PR_FILE_LIST.txt`.

## Explicit exclusions

- `build/`, `install/`, `log/` and `logs/`;
- raw experimental CSV/Markdown/TXT evidence under
  `logs/d435i_visual_slam/`;
- `*.db`, `*.bag`, `*.bak`, `*.patch`, temporary PDF/GV, PID and active files;
- source-audit documents: `EXTERNAL_REPOSITORY_MAP.md`,
  `PROJECT_IMPLEMENTATION_STATUS.md`, `PROJECT_SOURCE_INDEX.md`,
  `SOURCE_CONFLICTS.md` and `SOURCE_DUPLICATES_AND_VERSIONS.md`;
- the historical D435i experiment-document set, replaced by three core docs;
- unrelated `mid360_reliable_mapper`, FAST-LIO and Ultra-Fusion changes;
- the local `gz_mid360_pointcloud_bridge.py` compatibility tweak;
- machine paths, Windows configuration material, credentials and secrets;
- uncommitted source-worktree content and unfinished stage A/B/C work.

## Validation

| Check | Result |
|---|---|
| `git diff --check origin/main` | PASS |
| Bash syntax | PASS, 9 D435i shell scripts |
| Python syntax | PASS, 11 D435i/RTAB validation modules |
| YAML parse | PASS, 2 RTAB profiles |
| XML parse | PASS, model config/model and 2 worlds |
| C++/ROS build | PASS, `d435i_rgbd_bridge_cpp` |
| ROS package build | PASS, `multi_slam_uav_sim` |
| `colcon test` | PASS, 2 packages; 0 tests registered |
| Added-line privacy scan | PASS; no path, drive, key, password or secret value |
| Changed-path artifact scan | PASS; no prohibited artifact |
| Largest changed file | 51,322 bytes |
| D435i-only runtime smoke in this clone | NOT RUN: another isolated development task owns the active simulator |
| Full-simulation runtime regression in this clone | NOT RUN for the same isolation reason |

The approved stable commit records the runtime baseline summarized in the
benchmark: 640×480 RGB-D about 28–29 Hz, RTAB-Map about 16 Hz,
feature-aligned visual words/GlobalClosure, 0.35 m/s recommended, and
0.75 m/s straight-line 3/3 PASS with no lost/reset/wrong closure. This PR does
not claim those runs were repeated in the submission clone.

## Runtime isolation note

At submission time an existing D435i/Gazebo/SITL/MAVROS/RTAB-Map long-route run
from the development repository was active. The submission task did not
subscribe to its topics, start a competing simulator, or stop any process.
