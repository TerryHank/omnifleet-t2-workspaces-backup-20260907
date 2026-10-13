# Foxglove 113 / 111 adjustments, 2026-10-13

This is an incremental source/configuration backup on top of robot113-source-20261013-111614 (967777d38756695b4bfa1b839f32de6addbab5ba). The original src tree is unchanged. This directory records the subsequent Foxglove client/layout modifications, including the local Windows client. The destination repository is PUBLIC. No authentication files, credentials, recordings, caches, installed dependencies or Electron distributions are included.

## Captured changes

- 113 remote and local Teleop!t2Control: left angular.z +1, right angular.z -1; original forward/back +0.15/-0.15, topic /robot_113/cmd_vel, rate 10 Hz, and stopOnRelease retained.
- robot113-remote-layout.json: current /home/iecme/桌面/foxglove-layout.json, including its custom panels.
- local-active-layout.json: actual selected local layout read through Foxglove MCP, including operator edits beyond the reusable profile.
- local-113-control.json: persistent desktop-launcher profile with native direction controls and the restored Nav2 permanent-parameter panel at the right.
- robot-111-adapted.json: separate unnamespaced Ackermann profile, /cmd_vel_teleop, forward 0.15 m/s, reverse -0.35 m/s and steering linear.y +/-15 degrees. This is independent source switching, not a new multi-robot navigation backend.
- demo-control.cjs: deployed Windows loopback layout API source with Gauge/Teleop support and robot host/topic matching. Existing token checks, script/Publish rejection and fixed joint-extension constraints retained.
- Nav2PermanentPanel113.js: actual current 113 native parameter-panel source, copied unchanged. The Nav2 backend itself was not modified.
- LukaConsole.index.before.tsx / LukaConsole.index.with-nav2.tsx: source preimage and equivalent integration source for the installed native adapter patch. The deployed compiled chunk was patched directly; this source form has not been separately rebuilt. It selects the imported native panel only for initialView=nav2Parameters113 and exposes its read status through the Foxglove API.
- switch_robot.py and Foxglove-111.cmd / Foxglove-113.cmd: matching connection/layout selection. The script reads the current local API credential at runtime; no credential is embedded. Launcher paths reflect the original Windows installation and must be adjusted when restoring elsewhere.

## Restore

Back up the current client/layout before applying anything. Copy robot113-remote-layout.json to the 113 desktop layout path, then import it into that robot's existing Foxglove client. The original snapshot contains the robot-side native panel source; its existing customized Foxglove client is also required.

For a Windows client built from TerryHank/foxglove-studio-cn, place Nav2PermanentPanel113.js and its declaration alongside packages/studio-base/src/panels/LukaConsole/index.tsx; review the supplied preimage and replace that file with LukaConsole.index.with-nav2.tsx, and replace desktop-cn/demo-control.cjs with the supplied API source. Rebuild the corresponding desktop client. These instructions require that fork's source and normal build dependencies; they do not claim that the older 113 fork in the parent snapshot is interchangeable with the Windows fork.

Keep the JSON profiles beside switch_robot.py, adjust the two launchers' Python/script paths, then use the launcher matching the intended robot. The launcher temporarily removes direction controls, connects, verifies source identity, and loads its matching profile. Local-active-layout.json preserves the operator's current panel arrangement if preferred.

Rollback files remain on the original workspace under FOXGLOVE_DEPLOY_111_113_20261013; 113's previous layout also remains at /home/iecme/桌面/foxglove-layout.json.before-direction-fix-20261013-114731. Those historical copies and the installed app.asar binary were intentionally excluded from this source backup.

## Evidence and limits

Both 113 remote and local corrected direction values were read back. Local Foxglove connected to 113 and reported '已读取永久配置，可以修改并保存。' The UI rendered the native direction pad and Nav2 parameter panel. Both 111/113 switching scripts were exercised without pressing control buttons.

No motion command or parameter-save action was sent by the agent. Physical direction, release-to-stop and emergency-stop acceptance were not tested. 111 IMU/odometry/voltage were received; its advertised lidar channel had no pointcloud samples during verification. A source backup does not prove a fresh build or real-robot acceptance.
