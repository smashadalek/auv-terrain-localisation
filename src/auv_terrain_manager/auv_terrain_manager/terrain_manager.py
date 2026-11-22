#!/usr/bin/env python3

from pathlib import Path

import rclpy
from rclpy.node import Node

import yaml

from geometry_msgs.msg import PoseStamped

from ros_gz_interfaces.srv import SpawnEntity
from ros_gz_interfaces.srv import DeleteEntity


MODEL_TYPE = 2


class TerrainManager(Node):

    def __init__(self):

        super().__init__("terrain_manager")

        # -------------------------------------------------------------
        # Parameters
        # -------------------------------------------------------------

        self.declare_parameter(
            "manifest",
            "/workspace/data/manifests/reykjanes_4km_tiles.yaml",
        )

        self.declare_parameter(
            "repo_root",
            "/workspace",
        )

        self.declare_parameter(
            "ground_truth_topic",
            "/simulation/ground_truth_pose",
        )

        self.declare_parameter(
            "active_radius_tiles",
            1,
        )

        self.declare_parameter(
            "update_period_s",
            1.0,
        )

        self.declare_parameter(
            "world_name",
            "auv_world",
        )

        self.declare_parameter(
            "hysteresis_margin_m",
            500.0,
        )

        # -------------------------------------------------------------
        # Read parameters
        # -------------------------------------------------------------

        manifest_path = Path(
            self.get_parameter(
                "manifest"
            ).value
        )

        self.repo_root = Path(
            self.get_parameter(
                "repo_root"
            ).value
        )

        self.radius = int(
            self.get_parameter(
                "active_radius_tiles"
            ).value
        )

        update_period = float(
            self.get_parameter(
                "update_period_s"
            ).value
        )

        self.world_name = str(
            self.get_parameter(
                "world_name"
            ).value
        )

        self.hysteresis_margin = float(
            self.get_parameter(
                "hysteresis_margin_m"
            ).value
        )

        # -------------------------------------------------------------
        # Load terrain manifest
        # -------------------------------------------------------------

        with manifest_path.open() as f:
            manifest = yaml.safe_load(f)

        self.tile_size_m = float(
            manifest["tile_size_m"]
        )

        self.tiles = manifest["tiles"]

        self.tiles_by_index = {
            (
                int(tile["row"]),
                int(tile["col"]),
            ): tile
            for tile in self.tiles
        }

        # -------------------------------------------------------------
        # Runtime terrain state
        # -------------------------------------------------------------

        self.loaded = set()

        self.pending_spawn = set()
        self.pending_delete = set()

        self.latest_pose = None

        # This stores the complete tile dictionary rather than
        # just the tile ID.
        self.current_centre = None

        # -------------------------------------------------------------
        # Ground-truth subscription
        #
        # IMPORTANT:
        # This node belongs to the SIMULATION environment.
        # It is allowed to consume ground truth so that Gazebo can
        # construct the physical world surrounding the true vehicle.
        #
        # Localisation / autonomy must never consume this information.
        # -------------------------------------------------------------

        ground_truth_topic = self.get_parameter(
            "ground_truth_topic"
        ).value

        self.subscription = self.create_subscription(
            PoseStamped,
            ground_truth_topic,
            self.pose_callback,
            10,
        )

        # -------------------------------------------------------------
        # Gazebo entity services
        # -------------------------------------------------------------

        self.spawn_client = self.create_client(
            SpawnEntity,
            f"/world/{self.world_name}/create",
        )

        self.delete_client = self.create_client(
            DeleteEntity,
            f"/world/{self.world_name}/remove",
        )

        # -------------------------------------------------------------
        # Terrain update timer
        # -------------------------------------------------------------

        self.timer = self.create_timer(
            update_period,
            self.update_tiles,
        )

        # -------------------------------------------------------------
        # Startup information
        # -------------------------------------------------------------

        self.get_logger().info(
            f"Loaded terrain manifest with "
            f"{len(self.tiles)} tiles"
        )

        self.get_logger().info(
            f"Terrain tile size: "
            f"{self.tile_size_m:.0f} m"
        )

        self.get_logger().info(
            f"Active terrain radius: "
            f"{self.radius} tile(s)"
        )

        self.get_logger().info(
            f"Hysteresis margin: "
            f"{self.hysteresis_margin:.0f} m"
        )

        active_width = (
            (self.radius * 2 + 1)
            * self.tile_size_m
        )

        self.get_logger().info(
            f"Nominal active terrain area: "
            f"{active_width / 1000:.1f} km x "
            f"{active_width / 1000:.1f} km"
        )

    # -----------------------------------------------------------------
    # Ground-truth pose
    # -----------------------------------------------------------------

    def pose_callback(self, msg):

        self.latest_pose = msg.pose

    # -----------------------------------------------------------------
    # Tile selection
    # -----------------------------------------------------------------

    def find_closest_tile(
        self,
        x,
        y,
    ):

        return min(
            self.tiles,
            key=lambda tile:
                (
                    float(tile["x"]) - x
                ) ** 2
                +
                (
                    float(tile["y"]) - y
                ) ** 2
        )

    def find_centre_tile(
        self,
        x,
        y,
    ):

        # -------------------------------------------------------------
        # Startup
        #
        # No active centre exists yet, so simply find the tile whose
        # centre is closest to the vehicle.
        # -------------------------------------------------------------

        if self.current_centre is None:

            return self.find_closest_tile(
                x,
                y,
            )

        # -------------------------------------------------------------
        # Hysteresis
        #
        # A 4 km tile has a nominal half-width of 2 km.
        #
        # With a 500 m hysteresis margin, the existing centre tile is
        # retained until the AUV moves more than 2.5 km away from its
        # centre along either axis.
        #
        # This prevents terrain repeatedly changing:
        #
        #   A -> B -> A -> B
        #
        # if the vehicle oscillates near a tile boundary.
        # -------------------------------------------------------------

        centre_x = float(
            self.current_centre["x"]
        )

        centre_y = float(
            self.current_centre["y"]
        )

        half_tile = (
            self.tile_size_m / 2.0
        )

        switch_distance = (
            half_tile
            + self.hysteresis_margin
        )

        x_distance = abs(
            x - centre_x
        )

        y_distance = abs(
            y - centre_y
        )

        if (
            x_distance <= switch_distance
            and
            y_distance <= switch_distance
        ):

            return self.current_centre

        # -------------------------------------------------------------
        # Vehicle has genuinely left the hysteresis region.
        # Select its new nearest terrain tile.
        # -------------------------------------------------------------

        return self.find_closest_tile(
            x,
            y,
        )

    # -----------------------------------------------------------------
    # Desired neighbourhood
    # -----------------------------------------------------------------

    def desired_tiles(
        self,
        centre,
    ):

        row = int(
            centre["row"]
        )

        col = int(
            centre["col"]
        )

        desired = {}

        for dr in range(
            -self.radius,
            self.radius + 1,
        ):

            for dc in range(
                -self.radius,
                self.radius + 1,
            ):

                index = (
                    row + dr,
                    col + dc,
                )

                tile = self.tiles_by_index.get(
                    index
                )

                if tile is not None:

                    desired[
                        tile["id"]
                    ] = tile

        return desired

    # -----------------------------------------------------------------
    # Path handling
    # -----------------------------------------------------------------

    def resolve_path(
        self,
        value,
    ):

        path = Path(value)

        if not path.is_absolute():

            path = (
                self.repo_root
                / path
            )

        return path.resolve()

    # -----------------------------------------------------------------
    # SDF generation
    # -----------------------------------------------------------------

    def make_tile_sdf(
        self,
        tile,
    ):

        tile_id = tile["id"]

        dem = self.resolve_path(
            tile["dem"]
        )

        mesh = self.resolve_path(
            tile["mesh"]
        )

        z_min = float(
            tile["z_min"]
        )

        z_range = float(
            tile["z_range"]
        )

        return f"""<?xml version="1.0"?>
<sdf version="1.9">

  <model name="{tile_id}">

    <static>true</static>

    <link name="seabed_link">

      <!--
        Physics / collision representation.

        The GeoTIFF remains the authoritative terrain surface
        for Gazebo physics.

        Ogre2 does not render this heightmap directly because
        the heightmap shader currently fails through our
        Docker / WSLg graphics path.
      -->

      <collision name="seabed_collision">

        <geometry>

          <heightmap>

            <uri>file://{dem}</uri>

            <size>
              {self.tile_size_m} {self.tile_size_m} {z_range}
            </size>

            <pos>
              0 0 {z_min}
            </pos>

          </heightmap>

        </geometry>

      </collision>


      <!--
        Visual representation.

        The OBJ contains the same terrain surface but is rendered
        using the normal Ogre2 mesh path.
      -->

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

    # -----------------------------------------------------------------
    # Spawn terrain
    # -----------------------------------------------------------------

    def spawn_tile(
        self,
        tile,
    ):

        tile_id = tile["id"]

        if (
            tile_id in self.loaded
            or
            tile_id in self.pending_spawn
        ):
            return

        request = SpawnEntity.Request()

        factory = (
            request.entity_factory
        )

        factory.name = tile_id

        factory.allow_renaming = False

        factory.sdf = (
            self.make_tile_sdf(
                tile
            )
        )

        factory.pose.position.x = float(
            tile["x"]
        )

        factory.pose.position.y = float(
            tile["y"]
        )

        factory.pose.position.z = 0.0

        factory.pose.orientation.w = 1.0

        factory.relative_to = "world"

        self.pending_spawn.add(
            tile_id
        )

        future = (
            self.spawn_client.call_async(
                request
            )
        )

        future.add_done_callback(
            lambda f, tid=tile_id:
                self.spawn_complete(
                    tid,
                    f,
                )
        )

        self.get_logger().info(
            f"Spawning {tile_id}"
        )

    def spawn_complete(
        self,
        tile_id,
        future,
    ):

        self.pending_spawn.discard(
            tile_id
        )

        try:

            response = future.result()

        except Exception as exc:

            self.get_logger().error(
                f"Spawn call failed for "
                f"{tile_id}: {exc}"
            )

            return

        if response.success:

            self.loaded.add(
                tile_id
            )

            self.get_logger().info(
                f"Loaded {tile_id}"
            )

        else:

            self.get_logger().error(
                f"Gazebo rejected "
                f"{tile_id}"
            )

    # -----------------------------------------------------------------
    # Remove terrain
    # -----------------------------------------------------------------

    def delete_tile(
        self,
        tile_id,
    ):

        if tile_id in self.pending_delete:
            return

        request = DeleteEntity.Request()

        request.entity.name = tile_id
        request.entity.type = MODEL_TYPE

        self.pending_delete.add(
            tile_id
        )

        future = (
            self.delete_client.call_async(
                request
            )
        )

        future.add_done_callback(
            lambda f, tid=tile_id:
                self.delete_complete(
                    tid,
                    f,
                )
        )

        self.get_logger().info(
            f"Removing {tile_id}"
        )

    def delete_complete(
        self,
        tile_id,
        future,
    ):

        self.pending_delete.discard(
            tile_id
        )

        try:

            response = future.result()

        except Exception as exc:

            self.get_logger().error(
                f"Delete call failed for "
                f"{tile_id}: {exc}"
            )

            return

        if response.success:

            self.loaded.discard(
                tile_id
            )

            self.get_logger().info(
                f"Removed {tile_id}"
            )

        else:

            self.get_logger().error(
                f"Gazebo failed to remove "
                f"{tile_id}"
            )

    # -----------------------------------------------------------------
    # Terrain update
    # -----------------------------------------------------------------

    def update_tiles(self):

        # -------------------------------------------------------------
        # We cannot select terrain until a ground-truth pose exists.
        # -------------------------------------------------------------

        if self.latest_pose is None:
            return

        # -------------------------------------------------------------
        # Wait for Gazebo services.
        # -------------------------------------------------------------

        if not self.spawn_client.service_is_ready():

            self.get_logger().warning(
                "Waiting for Gazebo "
                "spawn service..."
            )

            return

        if not self.delete_client.service_is_ready():

            self.get_logger().warning(
                "Waiting for Gazebo "
                "delete service..."
            )

            return

        # -------------------------------------------------------------
        # Current true simulated position
        # -------------------------------------------------------------

        x = (
            self.latest_pose.position.x
        )

        y = (
            self.latest_pose.position.y
        )

        # -------------------------------------------------------------
        # Select centre tile using hysteresis.
        # -------------------------------------------------------------

        centre = self.find_centre_tile(
            x,
            y,
        )

        centre_id = centre["id"]

        centre_changed = (
            self.current_centre is None
            or
            centre_id
            != self.current_centre["id"]
        )

        if centre_changed:

            if self.current_centre is None:

                self.get_logger().info(
                    f"Initial terrain region: "
                    f"{centre_id}"
                )

            else:

                old_id = (
                    self.current_centre["id"]
                )

                self.get_logger().info(
                    f"Terrain region changed: "
                    f"{old_id} -> "
                    f"{centre_id}"
                )

            self.current_centre = centre

        # -------------------------------------------------------------
        # Determine required local terrain neighbourhood.
        # -------------------------------------------------------------

        desired = self.desired_tiles(
            centre
        )

        desired_ids = set(
            desired.keys()
        )

        # -------------------------------------------------------------
        # SPAWN FIRST
        #
        # New terrain is always created before old terrain is removed.
        #
        # This ensures the vehicle never reaches an empty gap while a
        # terrain neighbourhood transition is taking place.
        # -------------------------------------------------------------

        missing = (
            desired_ids
            - self.loaded
            - self.pending_spawn
        )

        for tile_id in sorted(
            missing
        ):

            self.spawn_tile(
                desired[tile_id]
            )

        # -------------------------------------------------------------
        # Do not unload anything until every desired tile has finished
        # spawning successfully.
        # -------------------------------------------------------------

        if not desired_ids.issubset(
            self.loaded
        ):
            return

        # -------------------------------------------------------------
        # REMOVE OBSOLETE TERRAIN
        # -------------------------------------------------------------

        obsolete = (
            self.loaded
            - desired_ids
            - self.pending_delete
        )

        for tile_id in sorted(
            obsolete
        ):

            self.delete_tile(
                tile_id
            )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = TerrainManager()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()