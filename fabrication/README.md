# 3D print files: coin elevator (A4 wall model)

One set of STL files, 1:1 in mm, PETG. Each STL already lies in its print orientation. The filament colour is in the file name. `COLOURS.md` lists every part and its colour.

| File | Link | Part |
|---|---|---|
| `disc_LIGHTBLUE.stl` | 2 | Disc: gear rim, cam groove, crank pin |
| `rocker_YELLOW.stl` | 4 | Rocker (pawl) |
| `pinion_EMERALD.stl` | 5 | Motor pinion |
| `hopper_DARKGREY.stl` | 1 | Coin hopper |
| `pivot_post_BLACK.stl` | 1 | Rocker pivot post |
| `axle_stub_BLACK.stl` | 1 | Disc axle stub |
| `motor_bracket_BLACK.stl` | 1 | TT motor pad |

The same parts, as STEP, are in `cad/parts/`. `cad/build_parts.py` generates them from `params.m`.

The other parts of the model are a laser-cut acrylic base plate, slider, front cover, guide rails and coins, and a bought yellow TT gear motor.
