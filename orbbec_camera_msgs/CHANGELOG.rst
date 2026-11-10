^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package orbbec_camera_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

2.10.7 (2026-11-10)
-------------------
* Update package version to 2.10.7.

2.10.6 (2026-09-30)
-------------------
* Add ``StreamStatus`` and replace individual stream fields in ``DeviceStatus`` with a ``streams`` array; downstream message consumers must be updated.
* Extend ``SendActionCommand`` with trigger modes, relative delays, and the encoded timestamp response; rename ``destination_ip`` to ``broadcast_ip``.

2.9.3 (2026-07-17)
------------------
* Add interfaces for device configuration, SDK bag recording, and runtime stream profile control.

2.8.6 (2026-04-30)
------------------
* Add depth filter status message types: ``DepthFilterParam.msg``, ``DepthFilterState.msg``, and ``DepthFiltersStatus.msg``.
* Contributors: ob-yalian

2.7.6 (2026-02-13)
------------------
* Update package version.
* Contributors: ob-yalian

2.7.2 (2026-01-22)
------------------
* Update package version.
* Contributors: ob-yalian

2.6.3 (2025-12-31)
------------------
* Add changelog and update package version.
* Contributors: ob-yalian, obyalian, slz

2.5.4 (2025-10-23)
------------------
* Add device status message, user calibration services, and SDK/hardware version fields in ``DeviceInfo.msg``.
* Contributors: xiexun, obyalian, slz

2.4.7 (2025-09-01)
------------------

2.4.5 (2025-07-04)
------------------
* Add .camke-format.py and format CMakeLists.txt
* Contributors: jj

2.4.3 (2025-05-22)
------------------

2.3.4 (2025-04-21)
------------------
* Add service to set color and depth ROI
* Contributors: jj

2.2.4 (2025-02-28)
------------------

2.2.3 (2025-02-15)
------------------
* Add service switch and parameter setting interface for DecimationFilter, HDRMerge, SequencedFilter, ThresholdFilter, NoiseRemovalFilter, HardwareNoiseRemoval, SpatialAdvancedFilter and TemporalFilter
* Contributors: jj

2.1.1 (2024-12-24)
------------------
* Merge pull request #59 from orbbec/zerocopy
  chore: add gemini_intra_process_demo
* chore: add gemini_intra_process_demo
* chore: add license
* Contributors: Joe Dong

1.5.8 (2024-06-22)
------------------

1.5.7 (2024-06-05)
------------------

1.5.5 (2024-05-22)
------------------

1.5.4 (2024-05-15)
------------------

1.5.2 (2024-05-12)
------------------
* update README && make package
* Remove device info pid && vid
* Adapted G2R
* Contributors: Joe Dong

1.4.6 (2024-02-26)
------------------

1.4.4 (2024-01-15)
------------------

1.4.2 (2023-12-06)
------------------

1.4.1 (2023-11-23)
------------------

1.3.9 (2023-10-25)
------------------

1.3.8 (2023-10-17)
------------------

1.3.7 (2023-10-11)
------------------

1.3.6 (2023-10-07)
------------------

1.3.5 (2023-09-15 16:54)
------------------------

1.3.4 (2023-09-15 15:11)
------------------------

1.3.2 (2023-09-13)
------------------

1.2.9 (2023-08-23)
------------------

1.2.4 (2023-04-28)
------------------
* bump to v1.2.2
* add switch IR camera
* Contributors: Joe Dong

1.2.1 (2023-02-20)
------------------
* fixed typo
* clean .make_deb
* Contributors: Joe Dong

1.0.4 (2022-07-07)
------------------
* add make deb scripts
* Contributors: Joe Dong

1.0.3 (2022-07-04 17:05)
------------------------

1.0.2 (2022-07-04 16:52)
------------------------
* fix typo
* add get bool msg
* Merge branch 'dev' into 'master'
  Fix point cloud direction
  See merge request OrbbecSDK/orbbecros2sdk!1
* Fix point cloud direction
* Contributors: Joe Dong, 默存

1.0.0 (2022-06-10)
------------------
* add get verion api
* add get device info srv
* init commit
* Contributors: Joe Dong
