# OmniFleet T2 workspace backup

Snapshot source: `iecme@192.168.3.113`
Snapshot time: `2026-09-07`

## Included

- `hardware_drivers_ws.tar.gz`
- `mola_3_2_ws.tar.gz`
- `omnifleet_t2_ws.tar.gz`
- `omnifleet_t2_mola_experiments_ws.tar.gz`

Each archive contains the workspace source/configuration tree. The T2 archive includes its runtime configuration, including `runtime/etc/omnifleet_t2/robot.env`; this repository must remain private because it contains robot/network/device configuration.

## Excluded

- Rebuildable `build/`, `install/`, `log/`, `Log/` directories
- Nested `.git/` metadata and Python caches
- `/home/iecme/omnifleet_t2_ws/backups/` historical crash/backups directory
- The unreadable file `omnifleet_t2_ws/backups/mola-cli-crash-20260901.crash`
- API keys, access tokens, and GitHub/Gitee credentials

## Restore

From the parent directory of the desired restore location:

```bash
tar -xzf hardware_drivers_ws.tar.gz
tar -xzf mola_3_2_ws.tar.gz
tar -xzf omnifleet_t2_ws.tar.gz
tar -xzf omnifleet_t2_mola_experiments_ws.tar.gz
```

`SHA256SUMS` is the source-host checksum manifest for the four archives. The local copies were checked against it before upload.
