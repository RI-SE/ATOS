#!/usr/bin/env python3


import hashlib
import xml.etree.ElementTree as ET
from pathlib import Path

import psycopg2
import rclpy
from rclpy.node import Node

from atos_interfaces.msg import Monitor
from atos_interfaces.srv import GetOpenScenarioFilePath


def sha256_of_xml(path: Path):
    return hashlib.sha256(str.encode(ET.canonicalize(from_file=str(path)))).hexdigest()


class SqlBridge(Node):
    def __init__(self):
        super().__init__("sql_bridge")

        self.get_logger().info("Init SqlBridge")

        self.init_ = self.create_subscription(
            Monitor,
            "object_monitor",
            self.monitor_subscription_callback,
            10,
        )

        self.declare_parameter("host", "")
        self.declare_parameter("port", 0)
        self.declare_parameter("user", "")
        self.declare_parameter("password", "")
        self.declare_parameter("dbname", "")

        self.xosc_path = None
        self.xosc_sha256 = None
        self.xodr_path = None
        self.xodr_sha256 = None

        self.get_open_scenario_file_path_ = self.create_client(
            GetOpenScenarioFilePath,
            "/atos/get_open_scenario_file_path",
        )
        self.get_open_scenario_file_path_.wait_for_service(timeout_sec=1.0)
        future = self.get_open_scenario_file_path_.call_async(
            GetOpenScenarioFilePath.Request()
        )
        future.add_done_callback(
            lambda future: self.get_open_scenario_file_path_callback_done(future)
        )

        self.create_table = """CREATE TABLE monitor (
            id                     SERIAL PRIMARY KEY,
            sec                    INT,
            nanosec                INT,
            object_id              INT,
            pose_point_x           DOUBLE PRECISION,
            pose_point_y           DOUBLE PRECISION,
            pose_point_z           DOUBLE PRECISION,
            pose_orientation_x     DOUBLE PRECISION,
            pose_orientation_z     DOUBLE PRECISION,
            pose_orientation_w     DOUBLE PRECISION,
            velocity_linear_x      DOUBLE PRECISION,
            velocity_linear_y      DOUBLE PRECISION,
            velocity_linear_z      DOUBLE PRECISION,
            velocity_angular_x     DOUBLE PRECISION,
            velocity_angular_y     DOUBLE PRECISION,
            velocity_angular_z     DOUBLE PRECISION,
            acceleration_linear_x  DOUBLE PRECISION,
            acceleration_linear_y  DOUBLE PRECISION,
            acceleration_linear_z  DOUBLE PRECISION,
            acceleration_angular_x DOUBLE PRECISION,
            acceleration_angular_y DOUBLE PRECISION,
            acceleration_angular_z DOUBLE PRECISION,
            drive_direction        INT,
            ready_to_arm           INT,
            object_state           INT,
            xosc_hash              VARCHAR(64),
            xodr_hash              VARCHAR(64)
        );

        CREATE TABLE xosc (
            hash   VARCHAR(64) PRIMARY KEY,
            data   XML
        );

        CREATE TABLE xodr (
            hash   VARCHAR(64) PRIMARY KEY,
            data   XML
        );
        """

        self.sql_insert_into_table = """
            INSERT INTO monitor
            VALUES (DEFAULT,
                    '{}', '{}', '{}',
                    '{}', '{}', '{}', '{}', '{}', '{}',
                    '{}', '{}', '{}', '{}', '{}', '{}',
                    '{}', '{}', '{}', '{}', '{}', '{}',
                    '{}', '{}', '{}',
                    '{}', '{}'
            );"""

        self.connection = psycopg2.connect(
            host=self.get_parameter("host").value,
            port=self.get_parameter("port").value,
            user=self.get_parameter("user").value,
            password=self.get_parameter("password").value,
            dbname=self.get_parameter("dbname").value,
        )
        self.cursor = self.connection.cursor()

    def get_open_scenario_file_path_callback_done(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info("Service call failed %r" % (e,))
        else:
            self.xosc_path = Path(response.path)
            self.xodr_path = (
                self.xosc_path.parent
                / Path(
                    ET.parse(self.xosc_path)
                    .getroot()
                    .find("RoadNetwork")
                    .find("LogicFile")
                    .attrib["filepath"]
                )
            ).resolve()

            self.xosc_sha256 = sha256_of_xml(self.xosc_path)
            self.xodr_sha256 = sha256_of_xml(self.xodr_path)

            self.insert_xml_if_new_hash("xosc", self.xosc_sha256, self.xosc_path)
            self.insert_xml_if_new_hash("xodr", self.xodr_sha256, self.xodr_path)

    def insert_xml_if_new_hash(self, table, hash, path):
        self.cursor.execute(
            f"SELECT exists (SELECT 1 FROM {table} WHERE hash = '{hash}' LIMIT 1);"
        )
        if not self.cursor.fetchone()[0]:
            self.cursor.execute(
                f"INSERT INTO {table} VALUES ('{hash}', XMLPARSE (DOCUMENT %s));",
                (path.read_text(),),
            )
            self.connection.commit()
            self.get_logger().info("Committed!")

    def monitor_subscription_callback(self, msg):
        insert_query = self.sql_insert_into_table.format(
            msg.atos_header.header.stamp.sec,
            msg.atos_header.header.stamp.nanosec,
            msg.atos_header.object_id,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
            msg.velocity.twist.linear.x,
            msg.velocity.twist.linear.y,
            msg.velocity.twist.linear.z,
            msg.velocity.twist.angular.x,
            msg.velocity.twist.angular.y,
            msg.velocity.twist.angular.z,
            msg.acceleration.accel.linear.x,
            msg.acceleration.accel.linear.y,
            msg.acceleration.accel.linear.z,
            msg.acceleration.accel.angular.x,
            msg.acceleration.accel.angular.y,
            msg.acceleration.accel.angular.z,
            msg.drive_direction.drive_direction,
            msg.ready_to_arm.ready_to_arm,
            msg.object_state.state,
            self.xosc_sha256,
            self.xodr_sha256,
        )
        self.cursor.execute(insert_query)
        self.connection.commit()


def main(args=None):
    rclpy.init(args=args)

    sql_bridge = SqlBridge()

    rclpy.spin(sql_bridge)

    sql_bridge.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
