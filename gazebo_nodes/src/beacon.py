#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from etsi_its_msgs.msg import (
    DENM,
    ItsPduHeader,
    ManagementContainer,
    ActionID,
    ReferencePosition,
    RelevanceDistance,
    RelevanceTrafficDirection,
    StationType
)

import time


ETSI_EPOCH_OFFSET = 1072915200000  # ms between 1970-01-01 and 2004-01-01


def etsi_time_now_ms():
    """Milliseconds since ETSI epoch (2004-01-01)."""
    unix_ms = int(time.time() * 1000)
    return unix_ms - ETSI_EPOCH_OFFSET


class RoadworkDENMBeacon(Node):

    def __init__(self):
        super().__init__('roadwork_denm_beacon')

        self.publisher = self.create_publisher(
            DENM,
            '/infra/roadwork/beacon',
            10
        )

        # ETSI requires periodic rebroadcast
        self.timer = self.create_timer(1.0, self.publish_denm)

        self.sequence_number = 0

        self.get_logger().info('Roadwork DENM beacon started.')

    def publish_denm(self):
        denm = DENM()

        # ---------------- Header ----------------
        denm.header = Header()
        denm.header.stamp = self.get_clock().now().to_msg()
        denm.header.frame_id = 'map'

        # ---------------- ITS PDU Header ----------------
        denm.its_header = ItsPduHeader()
        denm.its_header.protocol_version = 1
        denm.its_header.message_id = ItsPduHeader.MESSAGE_ID_DENM
        denm.its_header.station_id = 1001  # RSU / infrastructure ID

        # ---------------- Management Container ----------------
        mgmt = ManagementContainer()

        mgmt.action_id = ActionID()
        mgmt.action_id.station_id = denm.its_header.station_id
        mgmt.action_id.sequence_number = self.sequence_number
        self.sequence_number += 1

        now_etsi = etsi_time_now_ms()

        mgmt.detection_time = now_etsi
        mgmt.reference_time = now_etsi

        mgmt.termination = ManagementContainer.TERMINATION_UNAVAILABLE

        # --- Event position (roadwork location) ---
        pos = ReferencePosition()
        pos.latitude = 9   # example latitude (0.1 micro-degree)
        pos.longitude = 58  # example longitude
        mgmt.event_position = pos

        mgmt.relevance_distance = RelevanceDistance()
        mgmt.relevance_distance.value = RelevanceDistance.LESS_THAN_100M

        mgmt.relevance_traffic_direction = RelevanceTrafficDirection()
        mgmt.relevance_traffic_direction.value = (
            RelevanceTrafficDirection.UPSTREAM_TRAFFIC
        )

        mgmt.validity_duration = ManagementContainer.VALIDITY_DURATION_DEFAULT
        mgmt.transmission_interval = (
            ManagementContainer.TRANSMISSION_INTERVAL_TEN_SECONDS
        )

        mgmt.station_type = StationType()
        mgmt.station_type.value = StationType.ROAD_SIDE_UNIT

        denm.management = mgmt

        # ---------------- Disable unused containers ----------------
        denm.has_situation = False
        denm.has_location = False

        self.publisher.publish(denm)

        self.get_logger().debug(
            f'Published DENM seq={mgmt.action_id.sequence_number}'
        )


def main():
    rclpy.init()
    node = RoadworkDENMBeacon()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()