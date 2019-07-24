===================
Configuration Files
===================


------------------------------------
User specified Global Configurations
------------------------------------

Example global configuration file

.. literalinclude:: ../data/sample.global.yml
    :language: yaml


-----------------------------------------
Auto-generated Agent Local Configurations
-----------------------------------------

Example local configuration file by assigning agent 1 to device 1.
Note that agent and device indices start from 0, and they are different from
``pid``.

.. literalinclude:: ../data/sample.local.yml
    :language: yaml


.. todo::

    Include usage of `gen_local_config` script to generate a local config from
    the global config

.. todo::

    We could simply the options for configuration files and generate necessary
    objects after parsing.

    + ``plist`` can be as simple as a list of pids of other agents.


