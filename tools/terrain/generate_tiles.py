#!/usr/bin/env python3

from pathlib import Path
from osgeo import gdal
import numpy as np


# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

SOURCE = Path(
    "data/processed/gazebo/gebco_2026_reykjanes_tile_source.tif"
)

OUTPUT_DIR = Path(
    "data/processed/gazebo/tiles_4km"
)

MANIFEST = Path(
    "data/manifests/reykjanes_4km_tiles.yaml"
)

SOURCE_SPACING_M = 250.0

# 4 km / 250 m = 16 intervals = 17 original GEBCO samples
SOURCE_SAMPLES_PER_TILE = 17
SOURCE_INTERVALS_PER_TILE = 16

TILE_SIZE_M = 4000.0

# Render / collision representation
OUTPUT_SAMPLES = 65

# Earlier preprocessing shifted GEBCO:
#     original elevation + 2799
#
# Therefore return to world elevation using:
Z_DATUM_OFFSET = -2799.0


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

def resample_bilinear(data, output_size):
    """Resample a square raster using separable linear interpolation."""

    src_rows, src_cols = data.shape

    src_x = np.arange(src_cols)
    src_y = np.arange(src_rows)

    dst_x = np.linspace(0, src_cols - 1, output_size)
    dst_y = np.linspace(0, src_rows - 1, output_size)

    temp = np.empty((src_rows, output_size), dtype=np.float64)

    for r in range(src_rows):
        temp[r, :] = np.interp(dst_x, src_x, data[r, :])

    result = np.empty((output_size, output_size), dtype=np.float64)

    for c in range(output_size):
        result[:, c] = np.interp(dst_y, src_y, temp[:, c])

    return result


def write_geotiff(path, data, projection, first_sample_x, first_sample_y):
    """
    Write the resampled terrain tile as a GeoTIFF.

    The 65 samples span exactly 4000 m:
        4000 / 64 = 62.5 m sample spacing.
    """

    spacing = TILE_SIZE_M / (OUTPUT_SAMPLES - 1)

    driver = gdal.GetDriverByName("GTiff")

    ds = driver.Create(
        str(path),
        OUTPUT_SAMPLES,
        OUTPUT_SAMPLES,
        1,
        gdal.GDT_Float32,
        options=["COMPRESS=LZW"],
    )

    # first_sample_x/y represent the centre of the first raster sample.
    # GeoTIFF geotransform wants the outer pixel corner.
    ds.SetGeoTransform(
        (
            first_sample_x - spacing / 2,
            spacing,
            0.0,
            first_sample_y + spacing / 2,
            0.0,
            -spacing,
        )
    )

    ds.SetProjection(projection)

    band = ds.GetRasterBand(1)
    band.WriteArray(data.astype(np.float32))
    band.SetNoDataValue(-9999)
    band.FlushCache()

    ds = None


def write_mesh(obj_path, mtl_path, shifted_height):
    """
    Generate an Ogre2-friendly OBJ.

    shifted_height is still in our working raster datum:
        real Z = shifted_height - 2799
    """

    rows, cols = shifted_height.shape

    spacing = TILE_SIZE_M / (OUTPUT_SAMPLES - 1)

    half_x = TILE_SIZE_M / 2.0
    half_y = TILE_SIZE_M / 2.0

    z_world = shifted_height + Z_DATUM_OFFSET

    # Terrain gradients for smooth vertex normals.
    dzdx = np.gradient(z_world, spacing, axis=1)

    # Raster rows go south while local mesh +Y goes north.
    dzdy = -np.gradient(z_world, spacing, axis=0)

    nx = -dzdx
    ny = -dzdy
    nz = np.ones_like(z_world)

    lengths = np.sqrt(nx**2 + ny**2 + nz**2)

    nx /= lengths
    ny /= lengths
    nz /= lengths

    with mtl_path.open("w") as f:
        f.write("newmtl seabed\n")
        f.write("Ka 0.12 0.16 0.14\n")
        f.write("Kd 0.28 0.38 0.30\n")
        f.write("Ks 0.03 0.03 0.03\n")
        f.write("Ns 8\n")
        f.write("illum 2\n")

    with obj_path.open("w") as f:
        f.write(f"mtllib {mtl_path.name}\n")
        f.write("o gebco_seabed\n")
        f.write("usemtl seabed\n")

        # Vertices
        for r in range(rows):

            y = half_y - r * spacing

            for c in range(cols):

                x = -half_x + c * spacing
                z = z_world[r, c]

                f.write(
                    f"v {x:.6f} {y:.6f} {z:.6f}\n"
                )

        # Normals
        for r in range(rows):
            for c in range(cols):

                f.write(
                    f"vn "
                    f"{nx[r, c]:.8f} "
                    f"{ny[r, c]:.8f} "
                    f"{nz[r, c]:.8f}\n"
                )

        # Faces
        for r in range(rows - 1):
            for c in range(cols - 1):

                a = r * cols + c + 1
                b = a + 1
                d = (r + 1) * cols + c + 1
                e = d + 1

                f.write(
                    f"f {a}//{a} {d}//{d} {b}//{b}\n"
                )

                f.write(
                    f"f {b}//{b} {d}//{d} {e}//{e}\n"
                )


# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------

def main():

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    MANIFEST.parent.mkdir(parents=True, exist_ok=True)

    ds = gdal.Open(str(SOURCE))

    if ds is None:
        raise RuntimeError(f"Could not open {SOURCE}")

    source = ds.GetRasterBand(1).ReadAsArray().astype(np.float64)

    gt = ds.GetGeoTransform()
    projection = ds.GetProjection()

    rows, cols = source.shape

    print(f"Source raster: {cols} x {rows}")

    expected_cols = (
        (cols - 1) // SOURCE_INTERVALS_PER_TILE
    )

    expected_rows = (
        (rows - 1) // SOURCE_INTERVALS_PER_TILE
    )

    if (cols - 1) % SOURCE_INTERVALS_PER_TILE != 0:
        raise RuntimeError(
            "Source width does not divide exactly into 4 km tiles"
        )

    if (rows - 1) % SOURCE_INTERVALS_PER_TILE != 0:
        raise RuntimeError(
            "Source height does not divide exactly into 4 km tiles"
        )

    print(
        f"Generating {expected_cols} x {expected_rows} "
        f"= {expected_cols * expected_rows} tiles"
    )

    # Centre sample of the complete cropped dataset.
    dataset_centre_col = (cols - 1) / 2.0
    dataset_centre_row = (rows - 1) / 2.0

    manifest_entries = []

    for tile_row in range(expected_rows):

        for tile_col in range(expected_cols):

            start_col = (
                tile_col * SOURCE_INTERVALS_PER_TILE
            )

            start_row = (
                tile_row * SOURCE_INTERVALS_PER_TILE
            )

            raw = source[
                start_row:
                start_row + SOURCE_SAMPLES_PER_TILE,
                start_col:
                start_col + SOURCE_SAMPLES_PER_TILE,
            ]

            if raw.shape != (
                SOURCE_SAMPLES_PER_TILE,
                SOURCE_SAMPLES_PER_TILE,
            ):
                raise RuntimeError(
                    f"Unexpected tile shape at "
                    f"{tile_row}, {tile_col}: {raw.shape}"
                )

            if np.any(raw <= -9000):
                raise RuntimeError(
                    f"NoData remains in tile "
                    f"{tile_row}, {tile_col}"
                )

            terrain = resample_bilinear(
                raw,
                OUTPUT_SAMPLES,
            )

            tile_id = (
                f"tile_r{tile_row:02d}_c{tile_col:02d}"
            )

            tif_path = OUTPUT_DIR / f"{tile_id}.tif"
            obj_path = OUTPUT_DIR / f"{tile_id}.obj"
            mtl_path = OUTPUT_DIR / f"{tile_id}.mtl"

            # ---------------------------------------------------------
            # Global projected coordinates of first source sample
            # ---------------------------------------------------------

            first_sample_x = (
                gt[0]
                + (start_col + 0.5) * gt[1]
            )

            first_sample_y = (
                gt[3]
                + (start_row + 0.5) * gt[5]
            )

            write_geotiff(
                tif_path,
                terrain,
                projection,
                first_sample_x,
                first_sample_y,
            )

            write_mesh(
                obj_path,
                mtl_path,
                terrain,
            )

            # ---------------------------------------------------------
            # Local Gazebo coordinates
            # ---------------------------------------------------------

            centre_col = (
                start_col
                + SOURCE_INTERVALS_PER_TILE / 2.0
            )

            centre_row = (
                start_row
                + SOURCE_INTERVALS_PER_TILE / 2.0
            )

            gazebo_x = (
                centre_col - dataset_centre_col
            ) * SOURCE_SPACING_M

            # Raster rows increase south.
            gazebo_y = (
                dataset_centre_row - centre_row
            ) * SOURCE_SPACING_M

            shifted_min = float(np.min(terrain))
            shifted_max = float(np.max(terrain))

            world_min = shifted_min + Z_DATUM_OFFSET
            world_max = shifted_max + Z_DATUM_OFFSET

            manifest_entries.append(
                {
                    "id": tile_id,
                    "row": tile_row,
                    "col": tile_col,
                    "x": gazebo_x,
                    "y": gazebo_y,
                    "z_min": world_min,
                    "z_max": world_max,
                    "z_range": world_max - world_min,
                    "dem": str(tif_path),
                    "mesh": str(obj_path),
                }
            )

    # -------------------------------------------------------------
    # YAML manifest without requiring PyYAML
    # -------------------------------------------------------------

    with MANIFEST.open("w") as f:

        f.write("dataset: gebco_2026_reykjanes\n")
        f.write(f"tile_size_m: {TILE_SIZE_M}\n")
        f.write(f"source_spacing_m: {SOURCE_SPACING_M}\n")
        f.write(f"render_samples: {OUTPUT_SAMPLES}\n")
        f.write(f"rows: {expected_rows}\n")
        f.write(f"cols: {expected_cols}\n")
        f.write("tiles:\n")

        for tile in manifest_entries:

            f.write(f"  - id: {tile['id']}\n")
            f.write(f"    row: {tile['row']}\n")
            f.write(f"    col: {tile['col']}\n")
            f.write(f"    x: {tile['x']:.3f}\n")
            f.write(f"    y: {tile['y']:.3f}\n")
            f.write(f"    z_min: {tile['z_min']:.3f}\n")
            f.write(f"    z_max: {tile['z_max']:.3f}\n")
            f.write(f"    z_range: {tile['z_range']:.3f}\n")
            f.write(f"    dem: {tile['dem']}\n")
            f.write(f"    mesh: {tile['mesh']}\n")

    print()
    print("Done.")
    print(f"Manifest: {MANIFEST}")
    print(f"Terrain files: {OUTPUT_DIR}")
    print(f"Total tiles: {len(manifest_entries)}")


if __name__ == "__main__":
    main()