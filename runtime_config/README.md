# runtime_config/ — runtime-writable per-cell state

Bind-mounted into the container at `/ws/runtime_config` (see
`docker-compose.yaml`), never baked into the image. Everything here belongs to
THIS cell / robot and is gitignored except this README.

| file | owner | what |
|---|---|---|
| `trajectories.json` | `cobot_operation` | remembered positions (v2: each pose carries `frame`, `tool_number`, canonical `joints_deg`). Seeded from `ros2_ws/src/cobot_bringup/config/trajectories.json` on first boot; a v1 file is migrated in place (`trajectories.json.v1.bak` kept). Rewritten by the controller-kinematics conversion whenever the active tool changes. |
| `tooling_state.yaml` | `cobot_control` | the active tool selection (`/cobot/set_active_tool`), restored at boot |
| `controller_profile/` | `cobot_control` | raw `TOOL.CND`, `UFRAME.CND`, `system_info.yaml`, `errors.txt` as last read from the YRC1000 (read-only dump; copy these to `ros2_ws/src/cobot_control/test/data/` as parser fixtures) |

Reset: delete `trajectories.json` → reseeded at the next boot; delete
`tooling_state.yaml` → boots with tool `none` (tool file 0).
