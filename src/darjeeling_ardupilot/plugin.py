# -*- coding: utf-8 -*-
"""
This module will provide a plugin for Darjeeling that adds CMT support.
"""
__all__ = ('StartTest',)

from typing import (Tuple, Dict, Any, Sequence, NoReturn, Mapping, Iterator,
                    Optional, Callable)
import os

import attr
import bugzoo
import darjeeling
from darjeeling.config import TestSuiteConfig
from darjeeling.core import Test, TestOutcome
from darjeeling.container import Container as DarjeelingContainer
from darjeeling.test import TestSuite
from darjeeling.util import Stopwatch
from darjeeling.exceptions import BadConfigurationException
from loguru import logger

from .ardu import Mission, SITL
from .core import Monitor
from .attack import Attack
from .simple import SimpleMonitor
from .util import CircleIntBuffer
from .executor import run_with_monitor_in_process


@attr.s(frozen=True, auto_attribs=True)
class StartTest:
    """Describes a START test case.

    Attributes
    ----------
    name: str
        The unique name of the test case.
    parameters_filename: str
        The absolute path to the parameters file inside the container.
    attack: Attack, optional
        A description of the dynamic attack that should take place during the
        test.
    mission: Mission
        The mission that should be executed by the vehicle.
    timeout_secs: int
        The maximum number of seconds, when speedup is applied, before the
        mission is cancelled.
    timeout_secs_without_speedup: int
        The maximum number of seconds, when speedup is not applied, before the
        mission is cancelled.
    speedup: int
        The simulator speedup that should be used during execution.
    """
    name: str
    mission: Mission
    parameters_filename: str = attr.ib()
    attack: Optional[Attack] = attr.ib(default=None)
    timeout_secs: int = attr.ib(default=300)
    speedup: int = attr.ib(default=1)

    @property
    def timeout_secs_without_speedup(self) -> int:
        return self.timeout_secs * self.speedup

    @staticmethod
    def from_dict(d: Dict[str, Any], dir_: str) -> 'StartTest':
        """Loads a test description from a given dictionary.

        Parameters
        ----------
        d: Dict[str, Any]
            A dictionary-based description of the test.
        dir_: str
            The absolute path of the directory that should be used when
            evaluating relative file paths.
        """
        logger.debug(f"loading test from dict [dir={dir_}]: {d}")

        def err(msg: str) -> NoReturn:
            raise BadConfigurationException(msg)

        # determine the test name
        if 'name' not in d:
            err("test definition is missing 'name' property")
        name = d['name']

        # determine the parameters file
        default_fn_parameters = '/opt/ardupilot/copter.parm'
        if 'parameters' not in d:
            logger.warning("test definition has no 'parameters' property: "
                           f"using default file [{default_fn_parameters}]")
        fn_parameters = d.get('parameters', default_fn_parameters)
        if not os.path.isabs(fn_parameters):
            err(f"'parameters' filename must be given as an absolute path")

        # load the mission
        if 'mission' not in d:
            err("test definition is missing 'mission' property")

        fn_mission = d['mission']
        if not os.path.isabs(fn_mission):
            fn_mission = os.path.join(dir_, fn_mission)
        if not os.path.exists(fn_mission):
            err(f"could not find mission file: {fn_mission}")

        mission = Mission.from_file(fn_mission)

        # determine speedup
        speedup = d.get('speedup', 1)

        # determine timeout
        if 'timeout-seconds' not in d:
            err("test definition is missing 'timeout-seconds' property")
        timeout_secs = d['timeout-seconds']

        # fetch attack
        attack: Optional[Attack]
        if 'attack' in d:
            attack = Attack.from_dict(d['attack'])
        else:
            attack = None

        return StartTest(name=name,
                         attack=attack,
                         parameters_filename=fn_parameters,
                         mission=mission,
                         timeout_secs=timeout_secs,
                         speedup=speedup)


@attr.s(frozen=True)
class StartTestSuiteConfig(TestSuiteConfig):
    """Describes a START test suite.

    Attributes
    ----------
    tests: Sequence[StartTest]
        A description of the tests within this test suite.
    model: str
        The vehicle model that should be tested.
    """
    NAME = 'start'
    tests: Sequence[StartTest] = attr.ib()
    model: str = attr.ib()

    @classmethod
    def from_dict(cls,
                  d: Dict[str, Any],
                  dir_: Optional[str] = None
                  ) -> TestSuiteConfig:
        assert dir_

        def err(msg: str) -> NoReturn:
            raise BadConfigurationException(msg)

        default_vehicle = 'copter'
        if 'vehicle' not in d:
            logger.warning("test suite definition is missing 'vehicle' "
                           f"property. using default value: {default_vehicle}")
            vehicle = default_vehicle
        else:
            vehicle = d['vehicle']
            logger.info("using vehicle type: {vehicle}")

        if 'tests' not in d:
            err("test suite definition is missing 'tests' section")
        tests = [StartTest.from_dict(dd, dir_) for dd in d['tests']]

        return StartTestSuiteConfig(tests=tests, model=vehicle)

    def build(self,
              environment: darjeeling.Environment,
              snapshot: bugzoo.Bug
              ) -> 'StartTestSuite':
        tests = {t.name: t for t in self.tests}
        return StartTestSuite(tests=tests,
                              environment=environment,
                              model=self.model)


MonitorFactory = Callable[[StartTest, Mission], Monitor]


@attr.s(str=False, repr=False)
class StartTestSuite(TestSuite):
    _tests: Mapping[str, StartTest] = attr.ib()
    _environment: darjeeling.Environment = attr.ib()
    _model: str = attr.ib()
    _port_pool_mavlink: CircleIntBuffer = \
        attr.ib(default=CircleIntBuffer(13000, 13500))

    def __getitem__(self, name: str) -> StartTest:
        return self._tests[name]

    def __iter__(self) -> Iterator[Test]:
        yield from self._tests.values()

    def __len__(self) -> int:
        return len(self._tests)

    def __str__(self) -> str:
        return repr(self)

    def __repr__(self) -> str:
        description = ', '.join(str(self[t]) for t in self._tests)
        return f"StartTestSuite({description})"

    def _execute_with_monitor(self,
                              container: DarjeelingContainer,
                              test: StartTest,
                              monitor: Monitor,
                              speedup: int,
                              timeout_mission: int
                              ) -> TestOutcome:
        timeout_overall = timeout_mission + 10
        ip_address = container.ip_address
        ports_mavlink = self._port_pool_mavlink.take(3)
        kwargs = {'mission': test.mission,
                  'container': container,
                  'model': self._model,
                  'parameters_filename': test.parameters_filename,
                  'monitor': monitor,
                  'attack': test.attack,
                  'speedup': speedup,
                  'timeout': timeout_mission,
                  'ports_mavlink': ports_mavlink}
        return run_with_monitor_in_process(**kwargs)

    def execute(self,
                container: DarjeelingContainer,
                test: StartTest,
                *,
                coverage: bool = False
                ) -> TestOutcome:
        """Executes a given test case inside a container."""
        monitor = SimpleMonitor(mission=test.mission)
        outcome = self._execute_with_monitor(container,
                                             test,
                                             monitor,
                                             speedup=test.speedup,
                                             timeout_mission=test.timeout_secs)
        logger.debug("CMT not included with public code -- skipping")
        return outcome
