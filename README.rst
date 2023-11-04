Introduction
============

CircuitPython helper library for the QMI8658C 6-DoF Accelerometer and Gyroscope

.. image:: https://img.shields.io/discord/327254708534116352.svg
    :target: https://adafru.it/discord
    :alt: Discord


.. image:: https://github.com/jins-tkomoda/CircuitPython_QMI8658C/workflows/Build%20CI/badge.svg
    :target: https://github.com/jins-tkomoda/CircuitPython_QMI8658C/actions
    :alt: Build Status


.. image:: https://img.shields.io/badge/code%20style-black-000000.svg
    :target: https://github.com/psf/black
    :alt: Code Style: Black


Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_
* `Register <https://github.com/adafruit/Adafruit_CircuitPython_Register>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://circuitpython.org/libraries>`_
or individual libraries can be installed using
`circup <https://github.com/adafruit/circup>`_.

Installing from PyPI
=====================
.. note:: This library is not available on PyPI yet. Install documentation is included
   as a standard element. Stay tuned for PyPI availability!


Installing to a Connected CircuitPython Device with Circup
==========================================================

Make sure that you have ``circup`` installed in your Python environment.
Install it with the following command if necessary:

.. code-block:: shell

    pip3 install circup

With ``circup`` installed and your CircuitPython device connected use the
following command to install:

.. code-block:: shell

    circup install qmi8658c

Or the following command to update an existing version:

.. code-block:: shell

    circup update

Usage Example
=============

.. code-block:: python3

    import time
    import board
    import qmi8658c

    i2c = board.I2C()  # uses board.SCL and board.SDA
    mpu = qmi8658c.QMI8658C(i2c)

    while True:
        ac = sensor.acceleration
        gy = sensor.gyro
        print(f"Acceleration: X:{ac[0]:.2f}, Y:{ac[1]:.2f}, Z:{ac[2]:.2f} m/s^2")
        print(f"Gyro X:{gy[0]:.2f}, Y:{gy[1]:.2f}, Z:{gy[2]:.2f} rad/s")
        print(f"Temperature: {sensor.temperature:.2f} C")
        print("")
        time.sleep(1)


Usage Note
=============

Accelerometer low power(LP) mode must be a gyro disabled.

.. code-block:: python3

    sensor.gyro_enable = 0
    sensor.accelerometer_rate = AccRate.RATE_LP_21_HZ


Documentation
=============
API documentation for this library can be found on `Read the Docs <https://circuitpython-qmi8658c.readthedocs.io/>`_.

For information on building library documentation, please check out
`this guide <https://learn.adafruit.com/creating-and-sharing-a-circuitpython-library/sharing-our-docs-on-readthedocs#sphinx-5-1>`_.


Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/jins-tkomoda/CircuitPython_QMI8658C/blob/HEAD/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.
