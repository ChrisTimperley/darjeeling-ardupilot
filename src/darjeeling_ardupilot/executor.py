# -*- coding: utf-8 -*-
from typing import Tuple, Optional
from contextlib import closing, ExitStack
from multiprocessing import Process, Queue
from queue import Empty
import time

from darjeeling import ProgramContainer as DarjeelingContainer
from darjeeling.core import TestOutcome
from darjeeling.util import Stopwatch
from loguru import logger

import darjeeling
import dronekit

from .ardu import Mission, SITL
from .core import Monitor
from .attack import Attack


def run_with_monitor_in_process(*args, **kwargs) -> TestOutcome:
    outcome: TestOutcome
    timeout: int = kwargs['timeout']

    def runner(q: Queue) -> None:
        outcome = run_with_monitor(*args, **kwargs)
        q.put_nowait(outcome)

    q: Queue = Queue()
    p = Process(target=runner, daemon=True, args=(q,))
    timer = Stopwatch()
    timer.start()
    p.start()
    p.join(timeout)  # increase?
    timer.stop()

    if p.is_alive():
        logger.debug("force terminating execution process")
        p.terminate()
        return TestOutcome(False, timer.duration)
    try:
        return q.get_nowait()
    except Empty:
        return TestOutcome(False, timer.duration)


def run_with_monitor(container: DarjeelingContainer,
                     mission: Mission,
                     model: str,
                     parameters_filename: str,
                     monitor: Monitor,
                     attack: Optional[Attack],
                     speedup: int,
                     timeout: int,
                     ports_mavlink: Tuple[int, int, int]
                     ) -> TestOutcome:
    ip_address = container.ip_address
    logger.debug(f"container: {container}")
    logger.debug(f"container IP address: {ip_address}")
    logger.debug(f"using model: {model}")
    logger.debug(f"using parameters filename: {parameters_filename}")
    logger.debug(f"using timeout: {timeout:d} seconds")
    logger.debug(f"using speedup: {speedup:d}X")

    with ExitStack() as exit_stack:
        shell = container.shell

        # launch SITL
        sitl_args = {'container': container,
                     'model': model,
                     'parameters_filename': parameters_filename,
                     'home': mission.home_location,
                     'speedup': speedup,
                     'ports': ports_mavlink}
        url_dronekit, url_attacker, url_monitor = \
            exit_stack.enter_context(SITL.launch(**sitl_args))
        logger.debug(f"allocated DroneKit URL: {url_dronekit}")
        logger.debug(f"allocated attacker URL: {url_attacker}")
        logger.debug(f"allocated monitor URL: {url_monitor}")

        # connect via DroneKit
        vehicle = exit_stack.enter_context(
            closing(dronekit.connect(url_dronekit, heartbeat_timeout=15)))
        logger.debug(f"connected to vehicle via DroneKit: {url_dronekit}")

        # attach the attacker
        if attack:
            logger.debug(f"launching attack: {attack}")
            exit_stack.enter_context(attack.wait_and_send(url_attacker))

        # attach the monitor
        logger.debug("attaching monitor to vehicle...")
        monitor.attach_to(url_monitor)
        exit_stack.enter_context(monitor)
        logger.debug("attached monitor to vehicle")

        # execute the mission
        timer = Stopwatch()
        timer.start()
        try:
            logger.debug("executing mission...")
            mission.execute(vehicle, timeout_mission=timeout)
        except TimeoutError:
            logger.debug("mission timed out after "
                         f"{timer.duration:.2f} seconds")
            passed = False
        # allow a small amount of time for the message to arrive
        else:
            logger.debug("reached end of mission")
            monitor.notify_mission_end()
            passed = monitor.is_ok()
            time.sleep(0.5)
        timer.stop()
        logger.debug(f"finished mission execution after {timer.duration:.3f}s")

        # determine the outcome
        outcome = TestOutcome(passed, timer.duration)
        logger.debug(f"test outcome: {outcome}")
        return outcome
