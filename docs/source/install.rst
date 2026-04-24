.. toctree::

.. _install:

Installing ROSCO toolset
========================
ROSCO toolsets can be utilized either to run an existing controller or to design and tune a controller from scratch.
We recommend using the instructions provided in the :ref:`full_rosco` to install the full ROSCO toolset.
This allows for full use of the provided functionalities including the controller and toolbox to facilitate controller tuning.
However, if only the ROSCO binary is needed (to run an existing controller, for example), then users should follow the instructions provided in :ref:`rosco_controller`

.. note::

   The ROSCO controller is implemented in **C++** (C++17). A Fortran compiler is **not** required.
   The build system fetches the ``toml++`` library automatically — no manual dependency installation is needed
   beyond a C++17 compiler and CMake.

.. _full_rosco:

Complete ROSCO Installation
---------------------------
Steps for the installation of the complete rosco toolset are:

1. Create a conda environment for ROSCO

.. code-block:: bash

    conda config --add channels conda-forge
    conda create -y --name rosco-env python=3.10
    conda activate rosco-env

    # Windows users: execute commands in an Anaconda terminal, not the standard command prompt
    # If you encounter SSL errors on Windows:
    conda config --set ssl_verify no

    # Install a C/C++17-capable compiler
    conda install -y gcc gxx          # Linux/Mac (via conda-forge)
    conda install -y m2w64-toolchain  # Windows (MinGW-w64 toolchain)

    # Install CMake (if not already available system-wide)
    conda install -y cmake make

    # Optional: ZeroMQ support (enables external controller communication)
    brew install zeromq               # Mac
    sudo apt install libzmq3-dev      # Linux (Debian/Ubuntu)

2. Clone and install the ROSCO toolbox with ROSCO controller

.. code-block:: bash

    git clone https://github.com/NREL/ROSCO.git
    cd ROSCO
    pip install -e . --no-deps

This step compiles the controller shared library (``libdiscon.so`` on Linux, ``libdiscon.dylib`` on Mac, or ``libdiscon.dll`` on Windows) into ``ROSCO/rosco/lib`` and installs the Python toolbox in develop mode. The ``toml++`` library is fetched automatically by CMake during the build.

3. If the pip-based build does not work, you can create the environment from the provided file and retry:

.. code-block:: bash

    conda env update --file environment.yml
    pip install -e . --no-deps

.. _rosco_controller:

Installing only the ROSCO controller
------------------------------------
:numref:`rosco_table` provides an overview of the primary methods available for installing only the ROSCO controller binary.

.. _rosco_table:
.. list-table:: Methods for Installing the ROSCO Controller
   :widths: 30 70
   :header-rows: 1

   * - Method
     - Use Case
   * - :ref:`rosco_direct_download`
     - Best for users who simply want to use a released version of the controller binary without working through the compilation procedures.
   * - :ref:`rosco_anaconda_download`
     - Best for users who just want to use the controller binary but prefer to download using the Anaconda package manager.
   * - :ref:`cmake_compile`
     - Best for users who need to re-compile the source code often, plan to use non-released versions of ROSCO (including modified source code), or who simply want to compile the controller themselves.

Anaconda is a popular package manager used to distribute software packages of various types.
CMake is a build configuration system that creates files as input to a build tool like GNU Make, Visual Studio, or Ninja.
CMake does not compile code or run compilers directly, but rather creates the environment needed for another tool to run compilers and create binaries.
For more information on CMake, please see `understanding CMake <https://openfast.readthedocs.io/en/main/source/install/index.html#understanding-cmake>`_ in the OpenFAST documentation.

.. _rosco_direct_download:

Direct Download
................
The most recent tagged version releases of the controller are `available for download <https://github.com/NREL/ROSCO/tags>`_. One can simply download these compiled binary files for their system and point to them in their simulation tools (e.g. through :code:`DLL_FileName` in the ServoDyn input file of OpenFAST).

.. _rosco_anaconda_download:

Anaconda Download
.................
Using the package manager, Anaconda_, the tagged 64-bit versions of ROSCO are available through the conda-forge channel.
In order to download the most recently compiled version release, from an anaconda powershell (Windows) or terminal (Mac/Linux) window, create a new anaconda virtual environment:

.. code-block:: bash

    conda config --add channels conda-forge
    conda create -y --name rosco-env python=3.10
    conda activate rosco-env

navigate to your desired folder to save the compiled binary using:

.. code-block:: bash

    cd <desired_folder>

and download the controller:

.. code-block:: bash

    conda install -y ROSCO

This will download a compiled ROSCO binary file into the default filepath for any dynamic libraries downloaded via anaconda while in the ROSCO-env.
The ROSCO binary file can be copied to your desired folder using:

.. code-block:: bash

    cp $CONDA_PREFIX/lib/libdiscon.* <desired_folder>

on Linux/Mac, or:

.. code-block:: bash

    copy %CONDA_PREFIX%/lib/libdiscon.* <desired_folder>

on Windows.


.. _cmake_compile:

Compile using CMake
.....................
CMake_ eases the compiling process significantly. We recommend that users use CMake if at all possible.

Prerequisites
^^^^^^^^^^^^^

- CMake ≥ 3.14
- C++17 compiler: ``clang++`` or ``g++`` on Mac/Linux; MSVC 2019+ or MinGW-w64 on Windows
- Internet access on first configure (CMake fetches ``toml++`` automatically via FetchContent)
- *Optional*: ZeroMQ development libraries for external controller communication

The ``toml++`` library is the only required external dependency and is downloaded automatically during the CMake configure step — no manual installation is needed.

Installing compilers
^^^^^^^^^^^^^^^^^^^^^

On **Mac**, the Xcode Command Line Tools include a suitable ``clang++``:

.. code-block:: bash

    xcode-select --install

On **Linux** (Debian/Ubuntu):

.. code-block:: bash

    sudo apt install build-essential cmake

On **Windows (64-bit)**, install the MinGW-w64 toolchain via Anaconda:

.. code-block:: bash

    conda install -y m2w64-toolchain cmake make

Building the shared library
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Once CMake and a C++17 compiler are available, build ROSCO from the repository root:

.. code-block:: bash

    # Clone ROSCO
    git clone https://github.com/NREL/ROSCO.git
    cd ROSCO

    # Configure
    mkdir -p build
    cd build
    cmake ../rosco/controller          # Mac/Linux
    cmake ../rosco/controller -G "MinGW Makefiles"  # Windows (MinGW)

    # Build and install (works on all platforms)
    cmake --build .
    cmake --install .

This generates ``libdiscon.so`` (Linux), ``libdiscon.dylib`` (Mac), or ``libdiscon.dll`` (Windows) and
copies it into the ``rosco/lib/`` directory.

.. note::

   The first ``cmake`` run will download ``toml++`` from GitHub. Subsequent builds use the cached copy.
   If you are working in an offline environment, clone ``toml++`` v3.4.0 manually and point CMake to it
   with ``-DFETCHCONTENT_SOURCE_DIR_TOMLPLUSPLUS=<path>``.

Optional: ZeroMQ support
^^^^^^^^^^^^^^^^^^^^^^^^^

ZeroMQ enables the controller to communicate with external processes at runtime. If ZeroMQ development
libraries are detected by CMake, support is compiled in automatically.

.. code-block:: bash

    # Mac
    brew install zeromq

    # Linux (Debian/Ubuntu)
    sudo apt install libzmq3-dev libzmq5 libczmq-dev libczmq4

Re-run CMake after installing ZeroMQ to pick up the new dependency.

Input file formats
^^^^^^^^^^^^^^^^^^

The C++ controller accepts both the classic ``DISCON.IN`` format and the newer TOML format (``DISCON.toml``).
The format is detected automatically from the file extension. The TOML format offers native array syntax,
comments, section headers, and type validation at parse time:

.. code-block:: toml

    [Filters]
    F_LPFCornerFreq = 1.674
    F_LPFType       = 1
    F_NotchFreqs    = [1.5, 3.0]

    [PitchControl]
    PC_ControlMode = 1
    PC_GS_angles   = [0.057, 0.084, 0.106]

See ``Examples/DISCON_template.toml`` for a fully annotated template with all available parameters and defaults.
Existing ``DISCON.IN`` files continue to work without modification.


.. _Anaconda: https://www.anaconda.com/
.. _CMake: https://cmake.org/
.. _MinGW: https://mingw-w64.org/
