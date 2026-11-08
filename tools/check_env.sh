#!/bin/bash
# check_env.sh —— 巡检前环境自检 + 一键清场
#
# 用法：
#   bash ~/narsu/ros2/tools/check_env.sh          # 只检查
#   bash ~/narsu/ros2/tools/check_env.sh kill     # 先杀掉所有相关进程再检查
#
# 目的：把"上次跑残的进程"清干净，逐项确认后再开始巡检。

source /opt/ros/humble/setup.bash 2>/dev/null
source /home/ros/narsu/ros2/patrol_ws/install/setup.bash 2>/dev/null

CAM_HOST="10.142.138.251"

ok()   { echo "  [OK]   $1"; }
bad()  { echo "  [FAIL] $1"; }
warn() { echo "  [WARN] $1"; }

if [ "$1" = "kill" ]; then
  echo "==== 清理残留进程 ===="
  pkill -f traffic_light_node 2>/dev/null && echo "  已停 traffic_light_node"
  pkill -f aim_marker         2>/dev/null && echo "  已停 aim_marker"
  pkill -f patrol_node        2>/dev/null && echo "  已停 patrol_node"
  pkill -f patrol_gui         2>/dev/null && echo "  已停 patrol_gui"
  pkill -f navigation2        2>/dev/null && echo "  已停 navigation2"
  pkill -f bringup.launch     2>/dev/null && echo "  已停 bringup"
  pkill -f rviz2              2>/dev/null && echo "  已停 rviz2"
  pkill -f "ros2 launch"      2>/dev/null
  sleep 3
  echo
fi

echo "==== 1. 网络 ===="
echo "  虚拟机 IP: $(hostname -I | awk '{print $1}')"
if ping -c1 -W1 "$CAM_HOST" >/dev/null 2>&1; then ok "摄像头 $CAM_HOST 可达"; else bad "摄像头不可达（检查是否通电、是否同一热点）"; fi

echo "==== 2. 摄像头视频流 ===="
CODE=$(curl -s -o /tmp/_cap.jpg -w "%{http_code}" --max-time 5 "http://$CAM_HOST/capture" 2>/dev/null)
if [ "$CODE" = "200" ]; then
  ok "/capture 200（$(stat -c%s /tmp/_cap.jpg 2>/dev/null) 字节）"
else
  bad "/capture 返回 $CODE"
fi

echo "==== 3. micro-ROS agent ===="
docker ps --format '{{.Names}} {{.Status}}' 2>/dev/null | grep -q mrosagent && ok "mrosagent 在跑" || bad "mrosagent 没跑（docker start mrosagent）"
N=$(timeout 15 ros2 node list 2>/dev/null | grep -c esp32)
if [ "$N" = "1" ]; then ok "esp32 节点 = 1"; else bad "esp32 节点 = $N（要 1；按板子 RST 或 docker restart mrosagent）"; fi

echo "==== 4. 雷达 ===="
PC=$(timeout 10 ros2 topic info /scan 2>/dev/null | grep -i "publisher count" | awk '{print $3}')
if [ "$PC" = "1" ]; then ok "/scan 发布者 = 1"; else bad "/scan 发布者 = ${PC:-0}"; fi

echo "==== 5. 相机图像话题 ===="
HZ=$(timeout 6 ros2 topic hz /vision/image_raw 2>/dev/null | head -1 | grep -oE "[0-9]+\.[0-9]+")
if [ -n "$HZ" ]; then ok "/vision/image_raw ${HZ} Hz"; else warn "/vision/image_raw 无数据（巡检界面起来后会补上）"; fi

echo "==== 6. 关键节点 ===="
NODES=$(timeout 15 ros2 node list 2>/dev/null)
for n in patrol_node patrol_gui traffic_light aim_marker controller_server planner_server amcl bt_navigator; do
  if echo "$NODES" | grep -qx "/$n"; then ok "$n"; else bad "$n 缺失"; fi
done

echo "==== 7. 关键话题 ===="
chk_topic() {
  local t=$1; local want=$2; local field=$3
  local v=$(timeout 8 ros2 topic info "$t" 2>/dev/null | grep -i "$field" | grep -oE '[0-9]+$')
  if [ "${v:-0}" = "$want" ]; then ok "$t $field=$v"; else bad "$t $field=${v:-0}（要 $want）"; fi
}
chk_topic /cmd_ptz_angle   1 "subscription count"
chk_topic /patrol_status   1 "publisher count"
chk_topic /patrol_command  1 "subscription count"
chk_topic /vision/light_state 1 "publisher count"
chk_topic /cmd_vel         1 "subscription count"

echo "==== 8. TF ===="
TFOK=$(timeout 6 ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null | head -3 | grep -c Translation)
if [ "${TFOK:-0}" -ge 1 ]; then ok "map -> base_footprint 可用"; else warn "map -> base_footprint 拿不到（导航没起来时正常）"; fi

echo "==== 9. 配置文件 ===="
CFG=/home/ros/narsu/ros2/patrol_ws/src/roscar_patrol/config
if [ -f "$CFG/patrol.yaml" ]; then
  echo "  巡检顺序: $(grep -A6 '^order' "$CFG/patrol.yaml" | grep -oE '^\- [A-Z]' | tr -d '[:space:]-' | tr '\n' ' ')"
  grep -E "inspect:|pan:|tilt:" "$CFG/patrol.yaml" | sed 's/^/    /'
else
  bad "patrol.yaml 不存在"
fi
if [ -f "$CFG/waypoints.yaml" ]; then
  ok "waypoints.yaml 存在（$(grep -c '^  [A-Z]:' "$CFG/waypoints.yaml") 个点位）"
fi

echo "==== 10. 地图 ===="
MAP=/home/ros/narsu/ros2/patrol_ws/src/roscar_navigation/maps
ls "$MAP"/room_*.yaml 2>/dev/null | tail -3 | sed 's/^/    /'

echo
echo "==== 启动顺序 ===="
echo "  终端1: ros2 launch roscar_bringup bringup.launch.py use_rviz:=false"
echo "  终端2: ros2 launch roscar_navigation navigation2.launch.py use_rviz:=false map:=$MAP/room_20261106.yaml"
echo "  终端3: ros2 launch roscar_patrol patrol_gui.launch.py"
