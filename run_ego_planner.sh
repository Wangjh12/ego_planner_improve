# #!/bin/bash
# LOG_FILE="/home/wjh/work/ego-planner_origin/flight_time_log.txt"  # 定义日志文件路径
# echo "Flight Time Log - 所有运行记录" > "$LOG_FILE"  # 初始化日志文件

# for i in {1..3}
# do
#   echo "第 $i 次执行 roslaunch ego_planner simple_run.launch"
#   gnome-terminal -- bash -c "\
#     cd /home/wjh/work/ego-planner_origin; \
#     source /opt/ros/noetic/setup.bash; \
#     source devel/setup.bash; \
#     mkfifo /tmp/ros_output_$$; \
#     roslaunch ego_planner simple_run.launch 2>&1 | tee /tmp/ros_output_$$ & \
#     LAUNCH_PID=\$!; \
#     while read line; do \
#       echo \"\$line\"; \
#       if [[ \"\$line\" == *\"flight time\"* ]]; then \
#         echo '检测到 flight time，等待 10 秒后终止...'; \
#         echo \"Flight Time Log - 第 $i 次运行\" >> \"$LOG_FILE\"; \
#         echo \"\$line\" >> \"$LOG_FILE\"; \
#         sleep 10; \
#         kill -INT \$LAUNCH_PID; \
#         sleep 1; \
#         rm -f /tmp/ros_output_$$; \
#         exit 0; \
#       fi; \
#     done < /tmp/ros_output_$$; \
#     rm -f /tmp/ros_output_$$; \
#     exec bash"
#   sleep 50
# done


# #!/bin/bash
# LOG_FILE="/home/wjh/work/ego_planner/flight_time_log.txt"  # 定义日志文件路径
# echo "Flight Time Log - 所有运行记录" >> "$LOG_FILE"  # 初始化日志文件

# for i in {1..3}
# do
#   echo "第 $i 次执行 roslaunch ego_planner simple_run.launch"
#   gnome-terminal -- bash -c "\
#     sleep 5;\
#     cd /home/wjh/work/ego_planner; \
#     source /opt/ros/noetic/setup.bash; \
#     source devel/setup.bash; \
#     mkfifo /tmp/ros_output_$$; \
#     roslaunch ego_planner simple_run.launch 2>&1 | tee /tmp/ros_output_$$ & \
#     LAUNCH_PID=\$!; \
#     \
#     # 启动70秒超时计时器，超时后强制终止roslaunch并关闭终端
#     ( sleep 70; \
#       echo '超时 70 秒，强制终止 roslaunch...'; \
#       kill -INT \$LAUNCH_PID; \
#       sleep 1; \
#       rm -f /tmp/ros_output_$$; \
#       exit 0; \
#     ) & \
#     TIMER_PID=\$!; \
#     \
#     while read line; do \
#       echo \"\$line\"; \
#       if [[ \"\$line\" == *\"flight time\"* ]]; then \
#         echo '检测到 flight time，等待 10 秒后终止...'; \
#         # 先去除 ANSI 转义码，再提取干净的 flight time 信息
#         clean_line=\$(echo \"\$line\" | sed -r 's/\x1B\[[0-9;]*m//g' | sed -n 's/.*\(flight time:.*\)/\1/p'); \
#         echo \"Flight Time Log - 第 $i 次运行\" >> \"$LOG_FILE\"; \
#         echo \"\$clean_line\" >> \"$LOG_FILE\"; \
#         sleep 10; \
#         kill -INT \$LAUNCH_PID; \
#         kill -INT \$TIMER_PID; \
#         sleep 1; \
#         rm -f /tmp/ros_output_$$; \
#         exit 0; \
#       fi; \
#     done < /tmp/ros_output_$$; \
#     rm -f /tmp/ros_output_$$; \
#     exit 0"
#   sleep 80
# done


#!/bin/bash
LOG_FILE="/home/wjh/work/ego_planner/flight_time_log_50.txt"  # 定义日志文件路径
echo "Flight Time Log - 所有运行记录" >> "$LOG_FILE"  # 追加写入日志文件，不清空上次内容

for i in {115..300}
do
  echo "第 $i 次执行 roslaunch ego_planner simple_run.launch"
  gnome-terminal -- bash -c "\
    sleep 5; \
    cd /home/wjh/work/ego_planner; \
    source /opt/ros/noetic/setup.bash; \
    source devel/setup.bash; \
    mkfifo /tmp/ros_output_$$; \
    roslaunch ego_planner simple_run.launch 2>&1 | tee /tmp/ros_output_$$ & \
    LAUNCH_PID=\$!; \
    \
    # 启动70秒超时计时器，超时后连续两次发送 Ctrl+C 强制终止 roslaunch，同时关闭 rviz 并关闭终端
    ( sleep 70; \
      echo '超时 70 秒，强制终止 roslaunch...'; \
      kill -INT \$LAUNCH_PID; \
      sleep 0.5; \
      kill -INT \$LAUNCH_PID; \
      killall -INT rviz; \
      sleep 0.5; \
      killall -INT rviz; \
      sleep 1; \
      rm -f /tmp/ros_output_$$; \
      exit 0; \
    ) & \
    TIMER_PID=\$!; \
    \
    while read line; do \
      echo \"\$line\"; \
      if [[ \"\$line\" == *\"flight time\"* ]]; then \
        echo '检测到 flight time，等待 10 秒后终止...'; \
        # 先去除 ANSI 转义码，再提取干净的 flight time 信息
        clean_line=\$(echo \"\$line\" | sed -r 's/\x1B\[[0-9;]*m//g' | sed -n 's/.*\(flight time:.*\)/\1/p'); \
        echo \"Flight Time Log - 第 $i 次运行\" >> \"$LOG_FILE\"; \
        echo \"\$clean_line\" >> \"$LOG_FILE\"; \
        sleep 10; \
        kill -INT \$LAUNCH_PID; \
        sleep 0.5; \
        kill -INT \$LAUNCH_PID; \
        killall -INT rviz; \
        sleep 0.5; \
        killall -INT rviz; \
        kill -INT \$TIMER_PID; \
        sleep 1; \
        rm -f /tmp/ros_output_$$; \
        exit 0; \
      fi; \
    done < /tmp/ros_output_$$; \
    rm -f /tmp/ros_output_$$; \
    exit 0"
  sleep 85
done
