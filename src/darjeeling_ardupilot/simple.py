# -*- coding: utf-8 -*-
__all__ = ('SimpleMonitor', 'SimpleMonitorStatus')

from typing import Optional, MutableSet
import time
import logging

from loguru import logger
import attr
import dronekit

from .core import Monitor, MonitorStatus
from .ardu import Mission, distance_metres

DIST_APPROX_SAME = 3.0


@attr.s
class SimpleMonitorStatus(MonitorStatus):
    _reached_home: bool = attr.ib(default=False)
    _visited_all_wps: bool = attr.ib(default=False)

    def is_ok(self) -> bool:
        return self._reached_home and self._visited_all_wps


@attr.s
class SimpleMonitor(Monitor):
    _mission: Mission = attr.ib()
    _status: SimpleMonitorStatus = attr.ib(factory=SimpleMonitorStatus)
    _url_mavlink: Optional[str] = attr.ib(init=False, default=None)
    _connection: Optional[dronekit.Vehicle] = \
        attr.ib(init=False, default=None, repr=False)
    _visited_wps: MutableSet[int] = \
        attr.ib(init=False, factory=set, repr=False)

    @property
    def status(self) -> MonitorStatus:
        return self._status

    def attach_to(self, url_mavlink: str) -> None:
        logger.debug(f"attaching [{self}] to MAVLink [{url_mavlink}]")
        self._url_mavlink = url_mavlink

    def open(self) -> None:
        assert self._url_mavlink
        self._connection = dronekit.connect(self._url_mavlink,
                                            heartbeat_timeout=15)

        def listener_waypoint(conn, name, message):
            self._visited_wps.add(message.seq)

        self._connection.add_message_listener('MISSION_CURRENT',
                                              listener_waypoint)

    def notify_mission_end(self) -> None:
        # TODO for the demo, just check that end location is home location?
        lat_expected, lon_expected, alt_expected = \
            self._mission.home_location[0:3]
        loc_expected: dronekit.LocationGlobal = \
            dronekit.LocationGlobal(lat_expected, lon_expected, alt_expected)
        loc_actual: dronekit.LocationGlobal = \
            self._connection.location.global_frame

        # have we visited all waypoints?
        visited_wps = self._visited_wps
        expected_wps = self._mission.waypoints
        if visited_wps == expected_wps:
            self._status._visited_all_wps = True
        else:
            logger.debug("failed to visit all waypoints\n"
                         f"* visited waypoints: {visited_wps}\n"
                         f"* expected waypoints: {expected_wps}")

        # check if the expected and actual location are roughly the same
        dist_to_home = distance_metres(loc_expected, loc_actual)
        logger.debug(f"distance to home: {dist_to_home:.3f} metres")
        if dist_to_home < DIST_APPROX_SAME:
            self._status._reached_home = True

    def close(self) -> None:
        if self._connection:
            self._connection.close()
