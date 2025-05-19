순서만 대충 쓸게요... (지침이슈)
1. 브랜치 파기
2. git chekcout {브랜치이름}
3. 본격 작업 전 아래 코드 ~/.bashrc에 입력하기
   
    ```
    echo 'export GAZEBO_PLUGIN_PATH=$HOME/DigitalTwinFSD/build/turtlebot:$GAZEBO_PLUGIN_PATH' >> ~/.bashrc
    echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~$HOME/DigitalTwinFSD/src/turtlebot/models' >> ~/.bashrc
    echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/DigitalTwinFSD/src/turtlebot/models' >> ~/.bashrc
    echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
    echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
    ```
4. 빌드하시고 소싱하세요
5. 가제보 가상환경 돌리기 : `ros2 launch turtlebot auto_drive.launch.py`
6. 열심히 작업합시다!
7. 파이팅.