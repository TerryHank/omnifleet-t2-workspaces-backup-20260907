# Robot 113 multi-robot shared-map and Zenoh materials

Collected from `iecme@192.168.3.113` on 2026-09-26.

- `shared_map_msc/`: the MSC coordinator/agent package, shared-map implementation, tests, deployment probes, and service units.
- `multi_robot_sim/`: three-robot simulation source, fleet controller, layouts, launch scripts, and interface documentation.
- `zenoh_gateway/`: cross-machine gateway and transport scripts from `/home/iecme/omnifleet_fleet`.
- `zenoh_rmw/`: the robot's `rmw_zenoh_cpp` source/config snapshot used for the Zenoh path.
- `multirobot_evidence/`: selected source/configuration snapshots from the 113 backup directories.

Build outputs, shared libraries, logs, captures, point clouds, screenshots, caches, and runtime state were excluded. These files document implementation and deployment inputs; they do not by themselves prove a current two-machine live handshake or shared-map convergence.
