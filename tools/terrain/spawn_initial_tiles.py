#!/usr/bin/env python3

from pathlib import Path
import subprocess
import yaml


MANIFEST = Path(
    "data/manifests/reykjanes_4km_tiles.yaml"
)

RUNTIME_DIR = Path(
    "data/processed/gazebo/runtime_models"
)

WORLD = "auv_world"

# Initial AUV position for this test.
AUV_X = 6000.0
AUV_Y = -2000.0


def make_tile_sdf(tile):
    """
    Create an SDF model containing:

      GeoTIFF -> collision
      OBJ     -> Ogre2 visual
    """

    tile_id = tile["id"]

    dem = Path(tile["dem"]).resolve()
    mesh = Path(tile["mesh"]).resolve()

    z_min = float(tile["z_min"])
    z_range = float(tile["z_range"])

    return f"""<?xml version="1.0"?>
<sdf version="1.9">

  <model name="{tile_id}">

    <static>true</static>

    <link name="seabed_link">

      <collision name="seabed_collision">

        <geometry>

          <heightmap>

            <uri>file://{dem}</uri>

            <size>
              4000 4000 {z_range}
            </size>

            <pos>
              0 0 {z_min}
            </pos>

          </heightmap>

        </geometry>

      </collision>


      <visual name="seabed_visual">

        <geometry>

          <mesh>

            <uri>file://{mesh}</uri>

            <scale>1 1 1</scale>

          </mesh>

        </geometry>

      </visual>

    </link>

  </model>

</sdf>
"""


def main():

    with MANIFEST.open() as f:
        manifest = yaml.safe_load(f)

    tiles = manifest["tiles"]

    # Find tile whose centre is closest to the AUV.
    centre = min(
        tiles,
        key=lambda t:
            (float(t["x"]) - AUV_X) ** 2
            +
            (float(t["y"]) - AUV_Y) ** 2
    )

    centre_row = int(centre["row"])
    centre_col = int(centre["col"])

    print()
    print("AUV position:")
    print(f"  x = {AUV_X}")
    print(f"  y = {AUV_Y}")

    print()
    print("Centre tile:")
    print(
        f"  {centre['id']} "
        f"(row {centre_row}, col {centre_col})"
    )

    wanted = []

    # 3 x 3 neighbourhood.
    for tile in tiles:

        row = int(tile["row"])
        col = int(tile["col"])

        if (
            abs(row - centre_row) <= 1
            and
            abs(col - centre_col) <= 1
        ):
            wanted.append(tile)

    if len(wanted) != 9:
        raise RuntimeError(
            f"Expected 9 tiles, found {len(wanted)}"
        )

    RUNTIME_DIR.mkdir(
        parents=True,
        exist_ok=True
    )

    print()
    print("Spawning tiles:")

    for tile in wanted:

        tile_id = tile["id"]

        tile_x = float(tile["x"])
        tile_y = float(tile["y"])

        sdf_file = (
            RUNTIME_DIR /
            f"{tile_id}.sdf"
        )

        sdf_file.write_text(
            make_tile_sdf(tile)
        )

        print(
            f"  {tile_id}: "
            f"x={tile_x:.0f}, "
            f"y={tile_y:.0f}"
        )

        command = [
            "ros2",
            "run",
            "ros_gz_sim",
            "create",

            "--world",
            WORLD,

            "--file",
            str(sdf_file.resolve()),

            "--name",
            tile_id,

            "--x",
            str(tile_x),

            "--y",
            str(tile_y),

            "--z",
            "0",
        ]

        result = subprocess.run(
            command,
            check=False
        )

        if result.returncode != 0:
            raise RuntimeError(
                f"Failed to spawn {tile_id}"
            )

    print()
    print("Done.")
    print(f"Spawned {len(wanted)} terrain tiles.")


if __name__ == "__main__":
    main()