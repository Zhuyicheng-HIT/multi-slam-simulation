# UltraFusion Replay Viewer

Browser playback UI for synchronized RGB, depth, local LiDAR, incremental map,
and trajectory inspection. The map starts empty and only reveals scans whose
timestamps are at or before the selected bag time. Seeking backward restores the
corresponding map prefix instead of displaying the final map.

## Generate M2DGR-Plus assets

```bash
cd tools/replay_viewer
source /opt/ros/humble/setup.bash
python3 scripts/export_replay_assets.py \
  /path/to/anomaly_ros2_mcap \
  public/replay/m2dgr-anomaly

source /home/ld666/ultrafusion-datasets/adapters_ws/install/setup.bash
python3 scripts/export_r3live_assets.py \
  /path/to/degenerate_seq_02 \
  public/replay/r3live-degenerate-02
```

The generated WebM and binary point-cloud assets are intentionally ignored by
Git. They are derived from the local dataset and total about 37 MB with the
default sampling limits.

## Run

```bash
npm ci
npm run dev -- --port 5173
```

Open <http://localhost:5173/>. Playback runs once and stops at the end. Dragging
the bottom timeline pauses playback and seeks RGB, depth, local LiDAR, map, and
trajectory to the same bag time.

The current dataset demo uses the bag's `/odom` poses to place scans. It does not
claim these poses are UltraFusion estimates. An algorithm-run exporter should
replace that trajectory source when connecting the viewer to backend output.
