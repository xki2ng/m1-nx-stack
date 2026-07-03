#!/bin/bash
# ============================================================
# M1 Full Stack: 点云处理 → FAST-LIO → nvblox → Nav2
# 一键启动 / 一键停止
# Usage: bash m1_full_stack.sh [start|stop|status|kill-nav2|restart-nav2]
# ============================================================
set -e

LOG_DIR=/home/robot/fastlio/logs
mkdir -p $LOG_DIR
LOG=$LOG_DIR/fullstack_$(date +%Y%m%d_%H%M%S).log
exec > >(tee -a $LOG) 2>&1

# ---- Config ----
FASTLIO_BIN=/home/robot/m1_ws/install/fast_lio/lib/fast_lio/fastlio_mapping
FASTLIO_CFG=/home/robot/fastlio/m1_airy96_rot.yaml
VOXEL_BIN=/home/robot/m1_ws/build/m1_voxel_filter/voxel_filter_node
VOXEL_LIB=/home/robot/m1_ws/build/m1_voxel_filter
MERGER_BIN=/home/robot/m1_ws/build/m1_voxel_filter/nav2_merger
NVBLX_BIN=/home/robot/m1_ws/install/nvblox_fastlio_bridge/lib/nvblox_fastlio_bridge/nvblox_fastlio_node
NVBLX_CFG=/home/robot/m1_ws/install/nvblox_fastlio_bridge/share/nvblox_fastlio_bridge/config/nvblox_airy96.yaml
NAV2_LAUNCH=/home/robot/fastlio/m1_nav2.launch.py
NAV2_LOCK=/tmp/m1_nav2.lock

VOXEL_LEAF=0.05          # 体素降采样尺寸 (m)

# ---- Env ----
source /opt/ros/humble/setup.bash
source /home/robot/m1_ws/install/setup.bash 2>/dev/null || true
source /home/robot/nav2_ws/install/setup.bash 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=66
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$VOXEL_LIB:$LD_LIBRARY_PATH

# ============================================================
# Zenoh 僵尸 session 清理
# ============================================================
cleanup_zenoh_zombies() {
    local closed=0
    ss -tnp 2>/dev/null | grep ":7447" | while read line; do
        local pid=$(echo "$line" | grep -oP 'pid=\K[0-9]+')
        if [ -z "$pid" ]; then continue; fi
        if ! kill -0 "$pid" 2>/dev/null; then
            local src=$(echo "$line" | awk '{print $4}')
            local sport=$(echo "$src" | rev | cut -d: -f1 | rev)
            echo "  Closing zombie Zenoh session: sport=$sport (dead PID $pid)"
            echo 1 | sudo -S ss -K "sport = :$sport" 2>/dev/null || true
            closed=$((closed + 1))
        fi
    done
    if [ $closed -gt 0 ]; then
        echo "  Cleaned $closed zombie Zenoh sessions"
    fi
}

# ============================================================
# Nav2 专用清理（保证残留彻底清除，支持手动 ros2 launch 清理）
# ============================================================
kill_nav2() {
    echo "=== Killing all Nav2 processes ==="

    # 如果有 PID 文件，先杀记录的 launch
    if [ -f "$NAV2_LOCK" ]; then
        local old_pid=$(cat "$NAV2_LOCK" 2>/dev/null)
        if [ -n "$old_pid" ]; then
            echo "  Killing recorded nav2 launch (PID $old_pid)..."
            kill -9 $old_pid 2>/dev/null || true
            pkill -9 -P $old_pid 2>/dev/null || true
        fi
        rm -f "$NAV2_LOCK"
    fi

    # 广度清理所有 nav2 相关 — 不管谁启动的都杀
    for name in controller_server planner_server behavior_server bt_navigator \
                lifecycle_manager velocity_smoother nav2_map_server \
                local_costmap global_costmap waypoint_follower; do
        pkill -9 -f "$name" 2>/dev/null || true
    done

    # 杀所有 body_to_base_link_tf（Nav2 launch 启动的）
    pkill -9 -f "static_transform_publisher.*body.*base_link" 2>/dev/null || true

    # 杀 costmap relay
    pkill -9 -f "costmap_zenoh_relay" 2>/dev/null || true
    pkill -9 -f "m1_vel_sign" 2>/dev/null || true
    pkill -9 -f "costmap_relay.py" 2>/dev/null || true

    # 杀 nav2 launch python 进程
    pkill -9 -f "m1_nav2.launch.py" 2>/dev/null || true
    pkill -9 -f "nav2_bringup" 2>/dev/null || true

    sleep 2

    # 二次确认
    for name in controller_server planner_server behavior_server bt_navigator \
                lifecycle_manager velocity_smoother body_to_base_link_tf \
                local_costmap global_costmap costmap_relay m1_nav2; do
        local leftover=$(pgrep -f "$name" 2>/dev/null || true)
        if [ -n "$leftover" ]; then
            echo "  WARNING: leftover '$name' still running, force killing..."
            pkill -9 -f "$name" 2>/dev/null || true
        fi
    done

    cleanup_zenoh_zombies
    sleep 1

    # 最终验证
    local remaining=$(ps aux | grep -E "(controller_server|planner_server|behavior_server|bt_navigator)" | grep -v grep | wc -l)
    if [ "$remaining" -gt 0 ]; then
        echo "  ERROR: $remaining nav2 processes still alive after cleanup!"
        ps aux | grep -E "(controller_server|planner_server|behavior_server|bt_navigator)" | grep -v grep
    else
        echo "  Nav2 cleanup: complete"
    fi
}

# ============================================================
stop_stack() {
    echo "=== Stopping M1 Full Stack ==="

    # 先杀 Nav2
    kill_nav2
    sleep 1
    echo "Restarting tail radar TF..."
    nohup ros2 run tf2_ros static_transform_publisher -0.404 0 -0.038 -0.707 0 0.707 0 body rslidar_tail > /home/robot/fastlio/logs/tf_rslidar_tail.log 2>&1 &

    # 杀核心进程
    echo "=== Killing core processes ==="
    echo 1 | sudo -S pkill -9 fastlio_mapping 2>/dev/null || true
    pkill -9 -f "voxel_filter_node" 2>/dev/null || true
    pkill -9 -f "m1_voxel_flt" 2>/dev/null || true
    pkill -9 -f "nav2_merger" 2>/dev/null || true
    pkill -9 -f "nvblox_fastlio_node" 2>/dev/null || true
    pkill -9 -f "m1_nvblox_br" 2>/dev/null || true

    sleep 1
    cleanup_zenoh_zombies
    sleep 1
    echo "  All stopped."
}

status_stack() {
    echo "=== M1 Full Stack Status ==="
    echo
    echo "-- Sensors & Odometry --"
    pgrep -a fastlio_mapping   && echo "  FAST-LIO: RUNNING"             || echo "  FAST-LIO: stopped"
    pgrep -a voxel_filter_node | while read p; do echo "  VoxelFilter: $p"; done
    ! pgrep voxel_filter_node >/dev/null 2>&1 && echo "  VoxelFilter: stopped"
    pgrep -a nvblox_fastlio_node && echo "  nvblox: RUNNING"             || echo "  nvblox: stopped"
    echo
    echo "-- Data Processing --"
    pgrep -a nav2_merger       && echo "  Nav2Merger: RUNNING"           || echo "  Nav2Merger: stopped"
    echo
    echo "-- Navigation --"
    local nav2_count=$(pgrep -c controller_server 2>/dev/null || echo 0)
    if [ "$nav2_count" -eq 1 ]; then
        echo "  Nav2: RUNNING (1 instance)"
    elif [ "$nav2_count" -gt 1 ]; then
        echo "  Nav2: RUNNING ($nav2_count instances -- DUPLICATE!)"
    else
        echo "  Nav2: stopped"
    fi
    pgrep -a planner_server | while read p; do echo "  Planner: $p"; done
    pgrep -a bt_navigator     | while read p; do echo "  BT: $p"; done
    pgrep -a costmap_relay    | while read p; do echo "  CostmapRelay: $p"; done

    echo
    echo "-- Lock File --"
    if [ -f "$NAV2_LOCK" ]; then
        local lock_pid=$(cat "$NAV2_LOCK")
        if kill -0 "$lock_pid" 2>/dev/null; then
            echo "  Lock: active (PID $lock_pid)"
        else
            echo "  Lock: stale (PID $lock_pid is dead)"
        fi
    else
        echo "  Lock: none"
    fi

    echo
    echo "-- Zenoh Zombies --"
    local zombies=0
    ss -tnp 2>/dev/null | grep ":7447" | while read line; do
        local pid=$(echo "$line" | grep -oP 'pid=\K[0-9]+')
        if [ -z "$pid" ]; then continue; fi
        if ! kill -0 "$pid" 2>/dev/null; then
            echo "  ZOMBIE: $line"
            zombies=$((zombies + 1))
        fi
    done
    if [ $zombies -eq 0 ] 2>/dev/null; then echo "  No zombie sessions"; fi
}

# ============================================================
restart_nav2() {
    echo "=== Restarting Nav2 only (core processes untouched) ==="
    kill_nav2
    sleep 1
    echo "Restarting tail radar TF..."
    nohup ros2 run tf2_ros static_transform_publisher -0.404 0 -0.038 -0.707 0 0.707 0 body rslidar_tail > /home/robot/fastlio/logs/tf_rslidar_tail.log 2>&1 &
    sleep 2

    echo "Starting Nav2..."
    nohup ros2 launch $NAV2_LAUNCH > $LOG_DIR/nav2.log 2>&1 &
    NAV2_PID=$!
    echo "$NAV2_PID" > "$NAV2_LOCK"
    echo "  Nav2: launched (PID $NAV2_PID)"
    sleep 25

    ros2 daemon stop 2>/dev/null; sleep 1
    nohup python3 /home/robot/fastlio/scripts/costmap_zenoh_relay.py > $LOG_DIR/relay.log 2>&1 &
    RELAY_PID=$!
    sleep 2
    if kill -0 $RELAY_PID 2>/dev/null; then
        echo "  Costmap relay: started (PID $RELAY_PID)"
    else
        echo "  Costmap relay: FAILED"
    fi
    nohup python3 /home/robot/fastlio/scripts/m1_vel_sign.py > $LOG_DIR/vel_sign.log 2>&1 &
    VEL_PID=$!
    sleep 1
    if kill -0 $VEL_PID 2>/dev/null; then
        echo "  Vel sign corrector: started (PID $VEL_PID)"
    else
        echo "  Vel sign corrector: FAILED"
    fi
    echo "  Nav2 restart complete."
}

# ============================================================
start_stack() {
    echo "========================================="
    echo "  M1 Full Stack Launch"
    echo "  $(date)"
    echo "========================================="

    # 0. Pre-flight check: 强制清理旧 Nav2
    echo "[0/5] Pre-flight cleanup..."
    kill_nav2
    sleep 1
    echo "Restarting tail radar TF..."
    nohup ros2 run tf2_ros static_transform_publisher -0.404 0 -0.038 -0.707 0 0.707 0 body rslidar_tail > /home/robot/fastlio/logs/tf_rslidar_tail.log 2>&1 &

    # 0.1 Kill old core processes
    echo "=== Stopping old core processes ==="
    echo 1 | sudo -S pkill -9 fastlio_mapping 2>/dev/null || true
    pkill -9 -f "voxel_filter_node" 2>/dev/null || true
    pkill -9 -f "m1_voxel_flt" 2>/dev/null || true
    pkill -9 -f "nav2_merger" 2>/dev/null || true
    pkill -9 -f "nvblox_fastlio_node" 2>/dev/null || true
    pkill -9 -f "m1_nvblox_br" 2>/dev/null || true
    cleanup_zenoh_zombies
    sleep 2

    # 1. Voxel Filters (front + rear)
    echo "[1/5] Starting voxel filters (leaf=${VOXEL_LEAF}m)..."
    nohup $VOXEL_BIN --ros-args \
        -p leaf_size:=$VOXEL_LEAF \
        -p input_topic:=/front_lidar \
        -p output_topic:=/front_lidar/filtered \
        > $LOG_DIR/voxel_front.log 2>&1 &
    PID_VF=$!

    nohup $VOXEL_BIN --ros-args -r __node:=voxel_filter_rear \
        -p leaf_size:=$VOXEL_LEAF \
        -p input_topic:=/rear_lidar \
        -p output_topic:=/rear_lidar/filtered \
        > $LOG_DIR/voxel_rear.log 2>&1 &
    PID_VR=$!

    sleep 3
    if kill -0 $PID_VF 2>/dev/null && kill -0 $PID_VR 2>/dev/null; then
        echo "  Voxel filters: OK (PIDs $PID_VF $PID_VR)"
        nohup ros2 run tf2_ros static_transform_publisher -0.404 0 -0.038 -0.707 0 0.707 0 body rslidar_tail \
            > $LOG_DIR/tf_rslidar_tail.log 2>&1 &
    else
        echo "  Voxel filters: FAILED!"
        return 1
    fi

    # 2. FAST-LIO
    echo "[2/5] Starting FAST-LIO..."
    nohup $FASTLIO_BIN --ros-args --params-file $FASTLIO_CFG \
        -p common.lid_topic:=/front_lidar/filtered \
        > $LOG_DIR/fastlio.log 2>&1 &
    PID_FL=$!
    sleep 5

    if kill -0 $PID_FL 2>/dev/null; then
        echo "  FAST-LIO: OK (PID $PID_FL)"
    else
        echo "  FAST-LIO: FAILED! Check $LOG_DIR/fastlio.log"
        return 1
    fi

    # 3. Nav2 Merger
    echo "[3/5] Starting Nav2 Merger..."
    nohup $MERGER_BIN > $LOG_DIR/merger.log 2>&1 &
    PID_MG=$!
    sleep 2

    if kill -0 $PID_MG 2>/dev/null; then
        echo "  Nav2Merger: OK (PID $PID_MG)"
    else
        echo "  Nav2Merger: FAILED!"
        return 1
    fi

    # 4. nvblox
    echo "[4/5] Starting nvblox..."
    nohup $NVBLX_BIN --ros-args --params-file $NVBLX_CFG \
        > $LOG_DIR/nvblox.log 2>&1 &
    PID_NV=$!
    sleep 2

    if kill -0 $PID_NV 2>/dev/null; then
        echo "  nvblox: OK (PID $PID_NV)"
    else
        echo "  nvblox: FAILED (non-critical, continuing)"
    fi

    # 5. Nav2 (单实例，PID 文件锁防重复)
    echo "[5/5] Starting Nav2..."
    kill_nav2   # 最后确认
    sleep 1

    nohup ros2 launch $NAV2_LAUNCH > $LOG_DIR/nav2.log 2>&1 &
    NAV2_PID=$!
    echo "$NAV2_PID" > "$NAV2_LOCK"
    echo "  Nav2: launched (PID $NAV2_PID, lock=$NAV2_LOCK)"
    sleep 25

    ros2 daemon stop 2>/dev/null; sleep 1
    nohup python3 /home/robot/fastlio/scripts/costmap_zenoh_relay.py > $LOG_DIR/relay.log 2>&1 &
    RELAY_PID=$!
    sleep 2
    if kill -0 $RELAY_PID 2>/dev/null; then
        echo "  Costmap relay: started (PID $RELAY_PID)"
    else
        echo "  Costmap relay: FAILED"
    fi
    nohup python3 /home/robot/fastlio/scripts/m1_vel_sign.py > $LOG_DIR/vel_sign.log 2>&1 &
    VEL_PID=$!
    sleep 1
    if kill -0 $VEL_PID 2>/dev/null; then
        echo "  Vel sign corrector: started (PID $VEL_PID)"
    else
        echo "  Vel sign corrector: FAILED"
    fi

    echo ""
    echo "========================================="
    echo "  Full Stack Running!"
    echo "  FAST-LIO:  /Odometry"
    echo "  Merged:    /merged_cloud (Nav2)"
    echo "  nvblox:    /nvblox/mesh, /nvblox/occupancy_grid"
    echo "  Nav2:      /local_costmap, /global_costmap"
    echo "  Nav2 PID:  $NAV2_PID (lock: $NAV2_LOCK)"
    echo "  Logs:      $LOG_DIR/"
    echo "========================================="
}

# ============================================================
case "${1:-start}" in
    stop)          stop_stack ;;
    status)        status_stack ;;
    kill-nav2)     kill_nav2 ;;
    restart-nav2)  restart_nav2 ;;
    start)         start_stack ;;
    *)             echo "Usage: $0 [start|stop|status|kill-nav2|restart-nav2]"; exit 1 ;;
esac
