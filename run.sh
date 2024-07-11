#!/bin/bash

# Function to run the first series of commands in a new sub-shell
run_first_series() {
    echo "Starting camera connection and yolo..."
    (
        # 进入第一个子 Shell
        source ~/anaconda3/etc/profile.d/conda.sh
        source ~/.bashrc
        conda activate camera
        cd ~/camera
        python3 sender.py
    )
}

# Function to run the second series of commands in a new sub-shell
run_second_series() {
    echo "Starting ros2 nodes..."
    (
        # 进入第二个子 Shell
        # 在这里激活你的虚拟环境
        source ~/.bashrc
        source ~/roscar_ws/install/setup.bash
        
        ros2 launch starter start.py
        
    )
}

# Run both series of commands in parallel
run_first_series &
run_second_series &

# Wait for all background processes to finish
wait

echo "All processes have completed."
