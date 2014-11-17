^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf_lookup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.3 (2014-11-17)
------------------
* backports to c++03
* Contributors: Enrique Fernandez

0.1.2 (2014-11-17)
------------------
* Backport from C++11 to C++03
* Contributors: Paul Mathieu

0.1.1 (2014-11-14)
------------------
* Fixes catkin_lint warnings
* Contributors: Enrique Fernandez

0.1.0 (2014-08-25)
------------------
* 0.1.0
* Add guards to tf_lookup_client_impl.h
* Add `wait' parameter for blocking part
* Need to wait for server to be available before using send_goal
* adds sanitize_frame, so the frame IDs are '/'-left-stripped
* tf_lookup: fix high cpu usage in python impl
* catches ROSException when send_goal (in order to have a clean shutdown)
* adds key_from_transformation and transformation_from_key
* uses lstrip instead of strip
* refs #7069 : adds key_from_transformation to tf_common; used in tf_stream/lookup_client
* refs #7069 : strips '/' from target and source frames when the key is generated
* tf_lookup: don't erase previous CMAKE_CXX_FLAGS
* refs #7069 : fixes problem with the GoalHandle DestructionGuard w/o using an actionlib overlay
* refs #7069 : tf_lookup does NOT need pal_cmake, but it MUST be compiled with C++11
* tf_lookup: setup python lib correctly with catkin
* tf_lookup: add python clients
* Catkinize tf_lookup
* tf_lookup: add license headers in source files
* tf_lookup: fix tf namespacing problem
* tf_lookup: cleanup topics when not used anymore
* Cleanup packages tf_lookup and sensor_to_cloud (no PAL dependencies)
* Fix super duper crazy corner case of #$@%!!
* tf_lookup: fix stream library (now works)
* tf_lookup: now a header-only lib with TfLookupClient
  implemented into sonar_to_cloud_publisher
* tf_lookup: new actionlib interface to stream individual transforms
  also, some changes were brought to sonar_to_cloud_publisher
  to reflect the changes in tf_lookup
* No thread anymore in tf_lookup
* Refactor tf_lookup
* Added an actionlib interface to tf_lookup
* Contributors: Enrique Fernandez, Paul Mathieu, Siegfried Gevatter
