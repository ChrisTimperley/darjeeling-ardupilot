# -*- coding: utf-8 -*-
__all__ = ('MonitorStatus', 'Monitor')

import abc


class MonitorStatus(abc.ABC):
    @abc.abstractmethod
    def is_ok(self) -> bool:
        """Checks the status of the monitor."""
        ...


class Monitor(abc.ABC):
    @abc.abstractmethod
    def attach_to(self, url_mavlink: str) -> None:
        """Attaches this monitor to a given vehicle."""
        ...

    @abc.abstractmethod
    def open(self) -> None:
        """Opens this monitor."""
        ...

    @abc.abstractmethod
    def close(self) -> None:
        """Closes this monitor."""
        ...

    @property
    @abc.abstractmethod
    def status(self) -> MonitorStatus:
        ...

    @abc.abstractmethod
    def notify_mission_end(self) -> None:
        """Called when the mission has finished."""
        ...

    def is_ok(self) -> bool:
        return self.status.is_ok()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, exc_tb):
        self.close()
