- [x] Solucionado
- [ ] No solucionado

## Error de "sincronizacion" o que no lee datos
[ WARN] [1721460485.086045186]: /rtabmap_viz: Did not receive data since 5 seconds! Make sure the input topics are published ("$ rostopic hz my_topic") and the timestamps in their header are set. If topics are coming from different computers, make sure the clocks of the computers are synchronized ("ntpdate"). If topics are not published at the same rate, you could increase "sync_queue_size" and/or "topic_queue_size" parameters (current=50 and 50 respectively).
/rtabmap_viz subscribed to:
   /airsim/odom

## Error al iniciar rtab-map
### Este error pude encontrar que
[ INFO] [1721460479.375829476]: rtabmap 0.21.5 started...
terminate called after throwing an instance of 'cv::Exception'
  what():  OpenCV(4.2.0) ../modules/core/src/matrix.cpp:209: error: (-215:Assertion failed) 0 <= _dims && _dims <= CV_MAX_DIM in function 'setSize'

[rtabmap-1] process has died [pid 49137, exit code -6, cmd /home/manuelo247/catkin_ws/devel/lib/rtabmap_slam/rtabmap --delete_db_on_start udebug /rgb/camera_info:=/airsim/image/camera_info /odom_info:=/airsim/odom_info /odom:=/airsim/odom /depth/image:=/airsim/image/depth /rgb/image:=/airsim/image/rgb __name:=rtabmap __log:=/home/manuelo247/.ros/log/98c6ab9e-4656-11ef-9083-5dc43197f92a/rtabmap-1.log].
log file: /home/manuelo247/.ros/log/98c6ab9e-4656-11ef-9083-5dc43197f92a/rtabmap-1*.log