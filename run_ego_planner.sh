#!/bin/bash
LOG_FILE="/home/wjh/work/ego_planner/new_time_200.txt"  # 定义日志文件路径
echo "Flight Time Log - 所有运行记录" >> "$LOG_FILE"  # 追加写入日志文件，不清空上次内容

# 外层循环：10 次
for j in {1..10}
do
  # 内层循环：20 次
  for i in {1..58}
  do
    echo "外层循环第 $j 次 | 内层循环第 $i 次 | 总计第 $(( (j-1)*20 + i )) 次运行"
    gnome-terminal -- bash -c "\
      sleep 3; \
      cd /home/wjh/work/ego_planner; \
      source /opt/ros/noetic/setup.bash; \
      source devel/setup.bash; \
      mkfifo /tmp/ros_output_$$; \
      roslaunch ego_planner simple_run.launch obs_seek:=$j 2>&1 | tee /tmp/ros_output_$$ & \
      LAUNCH_PID=\$!; \
      \
      # 启动超时计时器（原 70 秒，）
      ( sleep 55; \
        echo '超时 70 秒，强制终止...'; \
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
          echo '检测到 flight time，终止进程...'; \
          clean_line=\$(echo \"\$line\" | sed -r 's/\x1B\[[0-9;]*m//g' | sed -n 's/.*\(flight time:.*\)/\1/p'); \
          echo \"[Run $j-$i] Flight Time Log\" >> \"$LOG_FILE\"; \
          echo \"\$clean_line\" >> \"$LOG_FILE\"; \
          sleep 2; \
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
    sleep 60  # 等待本次运行结束
  done
done