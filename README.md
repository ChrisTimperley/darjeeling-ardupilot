# darjeeling-ardupilot

A test harness plugin for the [Darjeeling](https://github.com/squaresLab/Darjeeling)
program repair framework that allows ArduPilot systems to be tested via
software-in-the-loop simulation.


## Installation

Before installion, make sure that the following prerequisites are installed
on your machine:

* [Docker](https://docs.docker.com/install/). If using Ubuntu, be sure to follow
  the official installation instructions for Docker, as `apt`, by default, provides
  several egregiously outdated versions of Docker under slightly different names
  (e.g., `docker`, `docker.io`).
* Python 3.6.6 or greater. If the version of Python 3 on your machine is older
  than 3.6.6, you may find it convenient to use [pyenv](https://github.com/pyenv/pyen)
  to install the latest version.
* [Pipenv](https://github.com/pypa/pipenv) should also be installed. Pipenv is
  most easily installed to your system's Python libraries via `pip3`.


To install the Docker images for the bug scenarios:

```
$ make scenarios
```

To install `darjeeling-ardupilot` and its dependencies, including a specific
version of Darjeeling:

```
(pipenv) $ pip install -r requirements.txt
(pipenv) $ pip install -e .
```
