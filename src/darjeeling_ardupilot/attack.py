# -*- coding: utf-8 -*-
__all__ = ('Attack',)

from typing import Any, Dict, Iterator
import contextlib
import logging
import threading

from loguru import logger
import attr
import dronekit


@attr.s(frozen=True, slots=True, auto_attribs=True)
class Attack:
    parameter: str
    value: int
    waypoint: int

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> 'Attack':
        parameter = d['parameter']
        value = d['value']
        waypoint = d['waypoint']
        return Attack(parameter, value, waypoint)

    def _send(self, connection: dronekit.Vehicle) -> None:
        """Immediately sends this attack to the vehicle."""
        try:
            connection.parameters[self.parameter] = self.value
        except dronekit.TimeoutError:
            logger.debug("lost connection to vehicle -- unable to attack")
            return

    @contextlib.contextmanager
    def wait_and_send(self, url_mavlink: str) -> Iterator[None]:
        event_stop = threading.Event()

        def wait_loop(connection: dronekit.Vehicle):
            wp_trigger = self.waypoint
            while not event_stop.wait(0.1):
                try:
                    wp_current = connection.commands.next
                    if wp_current >= wp_trigger:
                        self._send(connection)
                        return
                except dronekit.TimeoutError:
                    logger.debug("lost connection to vehicle -- "
                                 "unable to attack")
                    return

        try:
            with contextlib.closing(dronekit.connect(url_mavlink, heartbeat_timeout=15)) as conn:  # noqa
                thread_loop = threading.Thread(target=wait_loop, args=(conn,))
                try:
                    thread_loop.start()
                    yield
                finally:
                    event_stop.set()
                    thread_loop.join()
        except dronekit.TimeoutError:
            logger.debug("lost connection to vehicle -- unable to attack")
