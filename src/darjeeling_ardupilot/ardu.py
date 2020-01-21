# -*- coding: utf-8 -*-
"""
This module provides interfaces and data structures for interacting with
ArduPilot via Dronekit and MAVLink.
"""
__all__ = ('Mission', 'SITL')

from typing import Union, Sequence, Tuple, Optional, Iterator, FrozenSet
import os
import time
import math
import contextlib
import signal
import subprocess
import pkg_resources
import logging

from darjeeling.container import Container as DarjeelingContainer
from loguru import logger
from roswire.util import Stopwatch
import attr
import dronekit

BIN_MAVPROXY = \
    pkg_resources.resource_filename(__name__, 'data/mavproxy')


def distance_metres(x: dronekit.LocationGlobal,
                    y: dronekit.LocationGlobal
                    ) -> float:
    """Computes the ground distance in metres between two locations.

    This method is an approximation, and will not be accurate over large
    distances and close to the earth's poles.

    Source
    ------
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    d_lat = y.lat - x.lat
    d_lon = y.lon - x.lon
    return math.sqrt((d_lat*d_lat) + (d_lon*d_lon)) * 1.113195e5


@attr.s(frozen=True, slots=True)
class Mission(Sequence[dronekit.Command]):
    """Represents a WPL mission."""
    filename: str = attr.ib()
    commands: Sequence[dronekit.Command] = \
        attr.ib(repr=False, converter=tuple, hash=False, eq=False)
    home_location: Tuple[float, float, float, float] = \
        attr.ib(init=False, repr=False, eq=False, hash=False)
    waypoints: FrozenSet[int] = \
        attr.ib(init=False, repr=False, eq=False, hash=False)

    def __attrs_post_init__(self) -> None:
        cmd_home = self.commands[0]
        home_lat = float(cmd_home.x)
        home_lon = float(cmd_home.y)
        home_alt = float(cmd_home.z)
        home_heading = 0.0
        object.__setattr__(self, 'home_location',
                           (home_lat, home_lon, home_alt, home_heading))
        object.__setattr__(self, 'waypoints',
                           frozenset(i for i in range(len(self.commands))))

    @commands.validator
    def validate_commands(self, attr, value):
        if not self.commands:
            raise ValueError("mission must have at least one command")

    @classmethod
    def from_file(cls, filename: str) -> 'Mission':
        logger.debug(f"loading mission file: {filename}")
        with open(filename, 'r') as f:
            lines = [l.strip() for l in f]
        filename = os.path.abspath(filename)
        return Mission(filename=filename,
                       commands=[cls.line_to_command(l) for l in lines[1:]])

    @staticmethod
    def line_to_command(s: str) -> dronekit.Command:
        """Parses a WPL line to a Dronekit command."""
        logger.debug(f"parsing command string: {s}")
        args = s.split()
        arg_index = int(args[0])
        arg_currentwp = 0
        arg_frame = int(args[2])
        arg_cmd = int(args[3])
        arg_autocontinue = 0  # not supported by dronekit
        (p1, p2, p3, p4, x, y, z) = [float(x) for x in args[4:11]]
        cmd = dronekit.Command(
            0, 0, 0, arg_frame, arg_cmd, arg_currentwp, arg_autocontinue,
            p1, p2, p3, p4, x, y, z)
        return cmd

    def __getitem__(self, index: Union[int, slice]) -> dronekit.Command:
        """Retrieves the i-th command from this mission."""
        return self.commands[index]

    def __len__(self) -> int:
        """Returns the number of commands in this mission."""
        return len(self.commands)

    def issue(self,
              connection: dronekit.Vehicle,
              *,
              timeout: float = 30.0
              ) -> None:
        """Issues this mission to a given vehicle.

        Parameters
        ----------
        connection: dronekit.Vehicle
            A connection to the vehicle.
        timeout: float
            A timeout that is enforced on the process of preparing the vehicle
            and issuing this mission.

        Raises
        ------
        TimeoutError
            If the timeout occurs before the mission has been issued to the
            vehicle.
        """
        timer = Stopwatch()
        timer.start()

        def time_left() -> float:
            return max(0.0, timeout - timer.duration)

        logger.debug("waiting for vehicle to be armable")
        while not connection.is_armable:
            if timer.duration > timeout:
                raise TimeoutError
            time.sleep(0.05)

        # set home location
        logger.debug("waiting for home location")
        while not connection.home_location:
            if timer.duration > timeout:
                raise TimeoutError
            vcmds = connection.commands
            vcmds.download()
            try:
                vcmds.wait_ready(timeout=time_left())
            except dronekit.TimeoutError:
                raise TimeoutError
            time.sleep(0.1)
        logger.debug(f'determined home location: {connection.home_location}')

        logger.debug("attempting to arm vehicle")
        connection.armed = True
        while not connection.armed:
            if timer.duration > timeout:
                raise TimeoutError
            time.sleep(0.05)
        logger.debug("armed vehicle")

        # upload mission
        vcmds = connection.commands
        vcmds.clear()
        for command in self:
            vcmds.add(command)
        vcmds.upload()
        try:
            vcmds.wait_ready(timeout=time_left())
        except dronekit.TimeoutError:
            raise TimeoutError

        # start mission
        logger.debug("switching to AUTO mode")
        connection.mode = dronekit.VehicleMode('AUTO')
        logger.debug("switched to AUTO mode")
        message = connection.message_factory.command_long_encode(
            0, 0, 300, 0, 1, len(self) + 1, 0, 0, 0, 0, 4)
        connection.send_mavlink(message)

    def execute(self,
                connection: dronekit.Vehicle,
                *,
                timeout_setup: float = 90.0,
                timeout_mission: float = 120.0
                ) -> None:
        """Executes this mission on a given vehicle.

        Parameters
        ----------
        connection: dronekit.Vehicle
            A connection to the vehicle.
        timeout_setup: float
            The timeout to enforce during mission setup.
        timeout_mission: float
            The timeout to enforce during the execution of the mission itself
            (i.e., after setup has been performed).
        """
        self.issue(connection, timeout=timeout_setup)

        timer = Stopwatch()
        timer.start()
        has_started = False
        command_num = 0

        def listener_statustext(other, name, message):
            logger.debug(f'STATUSTEXT: {message.text}')

        logger.debug('attaching STATUSTEXT listener')
        connection.add_message_listener('STATUSTEXT', listener_statustext)
        logger.debug('attached STATUSTEXT listener')

        def distance_to_home():
            return distance_metres(connection.home_location,
                                   connection.location.global_frame)

        while True:
            command_last = command_num
            command_num = connection.commands.next
            has_started |= command_num != 0

            # if we've reached the next WP, print a summary of the copter state
            if command_num != command_last:
                logger.debug(f"NEXT WP: {command_num}")
                logger.debug(f"HOME: {connection.home_location}")
                logger.debug(f"MODE: {connection.mode.name}")
                logger.debug(f"LOCATION: {connection.location.global_frame}")
                logger.debug("DISTANCE TO HOME: "
                             f"{distance_to_home():.2f} metres")

            # the command pointer rolls back to zero upon mission completion
            if has_started and command_num == 0:
                break

            if timer.duration > timeout_mission:
                logger.debug("timeout occurred during mission execution.")
                raise TimeoutError("mission did not complete before timeout")

            time.sleep(0.05)

        logger.debug('removing STATUSTEXT listener')
        connection.add_message_listener('STATUSTEXT', listener_statustext)
        logger.debug('removed STATUSTEXT listener')
        logger.debug("mission terminated")


@attr.s
class SITL:
    _container: DarjeelingContainer = attr.ib(repr=False)
    ip_address: str = attr.ib()
    model: str = attr.ib()
    parameters_filename: str = attr.ib()
    home: Tuple[float, float, float, float] = \
        attr.ib(default=(-35.363262, 149.165237, 0.000000, 0.00))
    speedup: int = attr.ib(default=1)
    _process: Optional[subprocess.Popen] = attr.ib(default=None, repr=False)
    binary: str = attr.ib(init=False)

    def __attrs_post_init__(self) -> None:
        binary_name = ({
            'copter': 'arducopter',
            'rover': 'ardurover',
            'plane': 'arduplane'
        })[self.model]
        self.binary = f'/opt/ardupilot/build/sitl/bin/{binary_name}'

    @staticmethod
    @contextlib.contextmanager
    def launch_with_mavproxy(container: DarjeelingContainer,
                             ip_address: str,
                             model: str,
                             parameters_filename: str,
                             home: Tuple[float, float, float, float],
                             ports: Tuple[int, ...],
                             *,
                             speedup: int = 1
                             ) -> Iterator[Tuple[str, ...]]:
        logger.debug("launching SITL with MAVProxy...")
        with SITL(container=container,
                  ip_address=ip_address,
                  model=model,
                  parameters_filename=parameters_filename,
                  home=home,
                  speedup=speedup) as sitl:
            logger.debug(f"started SITL: {sitl}")
            with sitl.mavproxy(*ports) as urls:
                yield urls

    @property
    def command(self) -> str:
        """The command that should be used to launch this SITL."""
        arg_home = ",".join(map(str, self.home))
        fn_param = self.parameters_filename
        fn_script = '/opt/ardupilot/Tools/autotest/sim_vehicle.py'
        cmd = (f'{self.binary} '
               f'--speedup {self.speedup} '
               f'--model {self.model} '
               f'--home {arg_home} '
               f'--defaults {fn_param}')
        return cmd

    @contextlib.contextmanager
    def mavproxy(self, *ports: int) -> Iterator[Tuple[str, ...]]:
        url_master = f'tcp:{self.ip_address}:5760'
        url_sitl = f'tcp:{self.ip_address}:5501'
        urls_out = tuple(f'udp:127.0.0.1:{p}' for p in ports)
        cmd_args = [f'{BIN_MAVPROXY} --daemon --master={url_master}']
        cmd_args += [f'--out {url}' for url in urls_out]
        cmd = ' '.join(cmd_args)

        logger.debug(f"launching mavproxy: {cmd}")
        p = None
        try:
            p = subprocess.Popen(cmd,
                                 encoding='utf-8',
                                 stdin=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL,
                                 stdout=subprocess.DEVNULL,
                                 preexec_fn=os.setsid,
                                 shell=True)
            yield urls_out
        finally:
            if p:
                os.killpg(p.pid, signal.SIGTERM)
                p.wait(2)
                logger.debug("mavproxy exited with code %d", p.returncode)

    def open(self) -> 'SITL':
        """Launches this SITL."""
        command = self.command
        logger.debug(f'launching SITL: {command}')
        self._process = self._container.shell.popen(command)
        time.sleep(5)
        return self

    def close(self) -> None:
        """Closes this SITL."""
        # FIXME temporary workaround for problems with .terminate
        # self._process.terminate()
        assert self._process
        cmd_kill = f'killall -15 {self.binary}'
        self._container.shell.execute(cmd_kill, user='root')
        try:
            retcode = self._process.wait(0.5)
        except subprocess.TimeoutExpired:
            logger.debug("force killing SITL process")
            self._process.kill()
            retcode = self._process.wait()
        out = '\n'.join(self._process.stream)  # type: ignore
        logger.debug('SITL output [{retcode}]:\n{out}')

    def __enter__(self) -> 'SITL':
        self.open()
        return self

    def __exit__(self, exc_type, exc, exc_tb):
        self.close()
