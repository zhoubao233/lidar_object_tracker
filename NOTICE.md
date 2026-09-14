# Code provenance and licensing

The tracking modules, Livox adapters, configuration, RViz configuration and tests
were copied from the working tree of `/home/zhoubao/APS/M-detector` on 2026-09-11:
- tools/object_tracking_test: tracking_core.py, object_tracker.py, config.yaml, tracking.rviz, test_tracking.py, evaluate_bag.py, VALIDATION.md
- tools/world_cloud_test: livox_to_body.py, livox_real_to_body.py, real_livox_core.py, corresponding launch files and tests

The source working tree includes uncommitted changes. This is a snapshot, not a
claim that all files match the source repository HEAD. Algorithms were retained;
imports, packaging, input namespaces and the simulator point-limit parameter were adapted.
No original dynfilter or M-detector C++ detection implementation is included.

The source repository has a GPL version 2 LICENSE; its text is preserved in
licenses/M-detector-GPL-2.0.txt for the migrated material. The destination already
contained a GPL version 3 LICENSE, which has not been changed. This migration does
not purport to relicense the migrated files under GPLv3. Authorship and any right
to relicense need to be confirmed before choosing a single license for publication.

## C++ port (2026-09-11)

All project Python sources were replaced by C++17 implementations, including live
nodes, deskew, tests and offline bag evaluation. Shell scripts, ROS configuration
and historical documents remain. PCL/Eigen/jsoncpp/yaml-cpp replace NumPy/SciPy/rospy.
This translation preserves licensing notices and asserts no new relicensing rights.
