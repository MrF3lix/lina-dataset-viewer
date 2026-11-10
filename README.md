# LINA Dataset Viewer

A small Node.js app to browse an exported linasim dataset: every scene (segment) of the
dataset in a list and a thumbnail gallery, and per scene the camera frames, the event camera,
the LiDAR top view, depth, a satellite map and the annotations, frame by frame. It works like
the Preview tab of the simulator web app (`09_simulator_config`), but reads the **exported
dataset** (`segment-*.h5`, the Ev-Waymo HDF5 layout of
`08_simulator/docs/DATASET.md` §7) instead of a run directory, and needs nothing else: no
Python, no simulator, no queue.

```bash
npm ci
```

```bash
node server.js --port 8700 /path/to/dataset
```

and open http://localhost:8700. The dataset folder is searched up to four levels deep for
`segment-*.h5`, so it can be one assembled dataset (`dataset.json`, `train.txt`,
`segment-….h5`) or a folder holding several (e.g. `09_simulator_config/data/datasets`); each
sub-folder becomes a group in the scene list, named after its `dataset.json`. **rescan** picks
up segments added while the server runs.

## What it shows

- **Scene list and gallery**: thumbnail (middle frame), location, local time, frames, split
  (from `train.txt`/`val.txt`), which sensors the segment has. Filter by text or split.
- **Camera**: the RGB frame with the recorded 2D boxes (dashed: tight mask bounds), the 3D
  boxes projected as wireframes, the object masks, and optionally the LiDAR scan projected
  into the image (coloured by depth, points inside a 3D box in the object colour).
- **Events**: the events of 1–8 dataset frames from the current one, or of a single
  sub-interval, accumulated into an image (ON white-blue, OFF red).
- **LiDAR top view**: the scan in the scene frame (x east, y north) centred on the scanner,
  coloured by height, range or intensity; points inside a 3D box in the object colour; the 3D
  boxes, the drones' paths, the camera's field of view. Wheel to zoom, drag to pan, **fit**
  to reset. Without a LiDAR it still shows the boxes and paths.
- **Depth** (when the segment has it), log-scaled.
- **Map**: the paths on Esri satellite imagery or OpenStreetMap (the browser loads the tiles
  and Leaflet from the internet; the server needs no internet).
- **Info**: the objects at this frame (vehicle-frame box, distance, size, heading, speed,
  LiDAR points, in FOV, 2D box) and the scene metadata with the full `segment_meta.json`.

Play / pause with the button or space, step with ← →; the timeline under the slider marks
the frames where an object is inside the image (blue) and the events per frame (orange). The
URL keeps the scene and frame (`#<segment id>@<frame>`), so a view can be shared.

## Options

```
node server.js <dataset_root> [--port 8700] [--host 127.0.0.1] [--token T] [--max-width 1280]
```

| Option | Default | |
|---|---|---|
| `--port` | 8700 (`VIEWER_PORT`) | |
| `--host` | 127.0.0.1 (`VIEWER_HOST`) | `0.0.0.0` to reach it from other machines |
| `--token` | none on loopback, else random (`VIEWER_TOKEN`) | the printed URL carries `?token=…`; the page sets a cookie and drops it from the URL. `none` disables it |
| `--max-width` | 1280 | camera images wider than this are box-downscaled by an integer factor before they are sent (a 1920 × 1080 frame goes out at 960 × 540); `0` sends them at full size |

`VIEWER_LOG=1` logs every request with its time.

## On the GPU cluster (CPU node)

Once, on the login node (needs internet; installs Node 22 into `~/.local/node` if there is
no `node` ≥ 18 on PATH, then `npm ci`):

```bash
git clone <this repo> ~/dataset_viewer && cd ~/dataset_viewer && cluster/install_node.sh
```

Then, each time, from the repository on the login node:

```bash
cluster/view.sh /raid/persistent_scratch/saaf/linasim/datasets/<name>
```

It submits `cluster/viewer.submit` to the `cpu` partition (2 cores, 4 GB, 8 h; extra
arguments go to `sbatch`, e.g. `cluster/view.sh <dataset> --time=120`), waits until the
server is up and prints something like

```
open http://node12:8742/?token=…
or tunnel from your machine:  ssh -N -L 8742:node12:8742 zhaw-login
and open http://localhost:8742/?token=…
```

Run the `ssh -N -L …` line on your own machine and open the localhost URL. The port is
8700 + job id mod 1000 (so several viewers on one node don't collide) unless `VIEWER_PORT`
is set; `VIEWER_SSH_HOST` sets the login alias in the printed command. The job serves until
its time limit or `scancel <job>`. Without `view.sh`:
`DATASET=/path sbatch cluster/viewer.submit`, and read `logs/slurm/viewer_<job>_<node>.out`.

The server binds `0.0.0.0` on the node, so anyone on the cluster network could reach the
port; the random per-job token keeps them out.

If the nodes cannot run the Node binary, an Apptainer image works too:
`apptainer exec --bind /raid:/raid docker://node:22-slim node server.js <dataset> --host 0.0.0.0`.

## How it works

- `server.js`: `node:http` server, static files from `public/`, a JSON/PNG API.
- `lib/segment.js`: reads one `segment-*.h5` with [h5wasm](https://github.com/usnistgov/h5wasm)
  (HDF5 compiled to WebAssembly; gzip and shuffle filters included, so no native HDF5
  library is needed): the per-scene summary (metadata, poses, calibration, all 3D and 2D
  boxes, per-frame counts), RGB frames, event images (accumulated on the server), masks,
  depth, LiDAR points.
- `lib/dataset.js`: finds the segments, reads `dataset.json` and the split files, keeps up
  to 12 files open.
- `lib/png.js`: a minimal PNG encoder on `node:zlib`.
- `public/`: the page, plain ES modules without a build step. The projections (LiDAR and 3D
  boxes into the camera, vehicle → scene frame) run in the browser with the conventions of
  `DATASET.md` §4.

| Route | Returns |
|---|---|
| `GET /api/dataset` | the groups and every segment's listing |
| `GET /api/segment?id=` | the scene summary |
| `GET /api/thumb?id=&w=` | middle frame, PNG |
| `GET /api/image?id=&frame=[&max_width=]` | camera frame, PNG |
| `GET /api/events?id=&frame=[&frames=N \| &sub=J]` | accumulated events, PNG (`X-Event-Count`) |
| `GET /api/mask?id=&frame=`, `/api/depth?id=&frame=` | mask overlay (RGBA) / depth, PNG |
| `GET /api/lidar?id=&frame=` | float32 (N, 6) points of the frame, vehicle frame |

`id` is the segment's path relative to the dataset root.

## Limits

- Only the HDF5 layout (`--format h5`, the default of `linasim-export-evwaymo` and of the web
  app's dataset assembly). The directory layout (`--format files`) keeps its 3D boxes in
  pickles only; convert it with `linasim-evwaymo-convert` first.
- LiDAR points carry no object id in the export, so "on objects" means inside a 3D box
  (+15 cm).
- The map converts scene metres to WGS84 with a flat-earth approximation around the
  location's origin, good to well under a metre within a few hundred metres.
