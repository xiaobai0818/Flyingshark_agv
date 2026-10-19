include "flyingshark_2d.lua"

-- 底盘没接上时，只用激光推里程计。正常运行请用 flyingshark_2d.lua。
options.provide_odom_frame = true
options.use_odometry = false
options.published_frame = "base_footprint"
options.tracking_frame = "base_footprint"

return options
