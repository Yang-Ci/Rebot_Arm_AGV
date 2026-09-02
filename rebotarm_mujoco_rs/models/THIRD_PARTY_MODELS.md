# Third-party model assets

## reBot B601-RS D405 wrist mount

`meshes/rebot_b601_rs_d405_mount_30deg.stl` is the release mesh from:

- Bowen Zhu, “reBot B601-RS RealSense D405 Wrist Mount”
- Version: `v1.0.0`, commit `ca379b9a4e19096730cd255dfc35ec419a7ad144`
- Source: <https://github.com/bowenszhu/rebot-b601-rs-d405-wrist-mount>
- Release STL SHA-256: `bf9ffe0eafc1c5f34afeac5485eced94ce3dc87977e3a995142e9cda287930d0`
- Hardware license: CERN Open Hardware Licence Version 2 — Weakly Reciprocal (`CERN-OHL-W-2.0`)

The design derives its wrist interface and D405 cradle from Seeed Studio's
`D405_305_Mount.step` in `Seeed-Projects/reBot-DevArm`, also licensed under
`CERN-OHL-W-2.0`. The imported STL is unmodified. The MuJoCo pose and camera
proxy are integration code in this repository; the printable STEP remains
available from the upstream release.
