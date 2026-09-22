"""Quest 3 controller -> EE target mapping, ported from rammp-org/kinova-quest-teleop.

Pure numpy/scipy; nothing here imports ROS. ``oculus_reader`` is imported lazily
inside ``OculusPoseSource`` so this package imports on hosts without adb.
"""
