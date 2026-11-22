#!/usr/bin/env python3

from pathlib import Path

import numpy as np
import yaml
from osgeo import gdal


MANIFEST = Path(
    "data/manifests/reykjanes_4km_tiles.yaml"
)

EDGE_TOLERANCE_M = 0.01


def load_dem(path):

    ds = gdal.Open(str(path))

    if ds is None:
        raise RuntimeError(
            f"Unable to open {path}"
        )

    return (
        ds.GetRasterBand(1)
        .ReadAsArray()
        .astype(np.float64)
    )


def main():

    with MANIFEST.open() as f:
        manifest = yaml.safe_load(f)

    tile_size = float(
        manifest["tile_size_m"]
    )

    tiles = manifest["tiles"]

    by_index = {
        (
            int(tile["row"]),
            int(tile["col"]),
        ): tile
        for tile in tiles
    }

    print(
        f"Tiles: {len(tiles)}"
    )

    print(
        f"Tile size: {tile_size:.1f} m"
    )

    print()

    worst_edge_error = 0.0

    horizontal_pairs = 0
    vertical_pairs = 0

    failures = []

    # -------------------------------------------------------------
    # Check adjacent terrain edges.
    # -------------------------------------------------------------

    for (row, col), tile in by_index.items():

        a = load_dem(
            Path(tile["dem"])
        )

        # ---------------------------------------------------------
        # EAST neighbour
        #
        # Current tile right edge should equal neighbour left edge.
        # ---------------------------------------------------------

        east = by_index.get(
            (row, col + 1)
        )

        if east is not None:

            b = load_dem(
                Path(east["dem"])
            )

            error = float(
                np.max(
                    np.abs(
                        a[:, -1]
                        -
                        b[:, 0]
                    )
                )
            )

            worst_edge_error = max(
                worst_edge_error,
                error,
            )

            horizontal_pairs += 1

            if error > EDGE_TOLERANCE_M:

                failures.append(
                    (
                        tile["id"],
                        east["id"],
                        error,
                    )
                )

        # ---------------------------------------------------------
        # SOUTH neighbour
        #
        # Current tile bottom edge should equal neighbour top edge.
        # ---------------------------------------------------------

        south = by_index.get(
            (row + 1, col)
        )

        if south is not None:

            b = load_dem(
                Path(south["dem"])
            )

            error = float(
                np.max(
                    np.abs(
                        a[-1, :]
                        -
                        b[0, :]
                    )
                )
            )

            worst_edge_error = max(
                worst_edge_error,
                error,
            )

            vertical_pairs += 1

            if error > EDGE_TOLERANCE_M:

                failures.append(
                    (
                        tile["id"],
                        south["id"],
                        error,
                    )
                )

    # -------------------------------------------------------------
    # Check manifest tile-centre spacing.
    # -------------------------------------------------------------

    spacing_failures = []

    for (row, col), tile in by_index.items():

        x = float(
            tile["x"]
        )

        y = float(
            tile["y"]
        )

        east = by_index.get(
            (row, col + 1)
        )

        if east is not None:

            dx = (
                float(east["x"])
                - x
            )

            if abs(
                dx - tile_size
            ) > 0.001:

                spacing_failures.append(
                    (
                        tile["id"],
                        east["id"],
                        "x",
                        dx,
                    )
                )

        south = by_index.get(
            (row + 1, col)
        )

        if south is not None:

            dy = (
                float(south["y"])
                - y
            )

            # Rows run south, therefore Gazebo Y decreases.
            if abs(
                dy + tile_size
            ) > 0.001:

                spacing_failures.append(
                    (
                        tile["id"],
                        south["id"],
                        "y",
                        dy,
                    )
                )

    # -------------------------------------------------------------
    # Results
    # -------------------------------------------------------------

    print(
        f"Horizontal joins checked: "
        f"{horizontal_pairs}"
    )

    print(
        f"Vertical joins checked: "
        f"{vertical_pairs}"
    )

    print(
        f"Worst DEM edge mismatch: "
        f"{worst_edge_error:.6f} m"
    )

    print()

    if failures:

        print("FAILED TERRAIN EDGES:")

        for a, b, error in failures[:20]:

            print(
                f"  {a} <-> {b}: "
                f"{error:.6f} m"
            )

    else:

        print(
            "PASS: all terrain edges match."
        )

    print()

    if spacing_failures:

        print("FAILED MANIFEST SPACING:")

        for failure in spacing_failures[:20]:

            print(
                "  ",
                failure,
            )

    else:

        print(
            "PASS: manifest tile spacing is correct."
        )

    print()

    if failures or spacing_failures:

        raise SystemExit(1)

    print(
        "Terrain dataset verification PASSED."
    )


if __name__ == "__main__":
    main()