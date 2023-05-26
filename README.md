# TurtleSim ChatGPT
This demo demonstrates how `ChatGPT` can be used to call into `ROS` services, specifically services in [turtlesim](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html).

`ROS` is interfaced via WebSockets through [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite). `ChatGPT`:
- Calls into WebSockets
- Does **not** execute any code on your machine
- Is given knowledge about the API via [turtlesim_api.json](turtlesim_api.json)


Prompt:

```shell
Move the turtle left by 2, then rotate 180 degrees, and move back to (5, 5).
Finally, spawn a turtle named turtle2 at (10, 10) and remove turtle1.
```

Result:

<p align="center">
    <img src="img/turtlesim.gif" width="300" height="300" />
</p>

## Installation
- Have a running ROS distribution, e.g. [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- Clone this repo `git clone git@github.com:mhubii/chatgpt_turtlesim.git`
- Install Python dependencies: `pip install -r requirements.txt`
- Register an account at [OpenAI](https://openai.com/) and get a key [Where do I find my Secret API Key?](https://help.openai.com/en/articles/4936850-where-do-i-find-my-secret-api-key)

## Run the Demo
1. Run `turtlesim`
```shell
source /opt/ros/humble/setup.bash # source your ROS distribution
ros2 run turtlesim turtlesim_node
```

2. Run `rosbridge_server`
```shell
source /opt/ros/humble/setup.bash # source your ROS distribution
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

3. Prompt [OpenAI](https://openai.com/)'s GPT models
```shell
python main.py --key your_key
```

You will we asked to enter a prompt. Try out
```shell
Move turtle1 left by 2, then rotate 180 degrees, and move back to (5, 5). Finally, spawn a turtle named turtle2 at (10, 10) and remove turtle1.
```

You will be presented with the generated API calls and will have to accept each one prior to execution.

Why not try others:

```shell
Add 3 more turtles with names inspired by artists at random positions
```

```shell
Move the turtles with artist's names forward by 2
```

Maybe you can find new ones! You might find that the format is not always adhered to!

## Notes

This repository is just a proof of concept and it is rather slow. It, however, demonstrates how such a system could work more broadly across ROS.

<details>
<summary>What are limitations?</summary>
<br>
ChatGPT does not always generate answers in the expected JSON format. This is something that requires improvements.
</details>

<details>
<summary>Why roslibpy and rosbridge_suite? Why not just execute Python code?</summary>
<br>
Operating ROS services through WebSockets limits GPT's access to your system. It further relieves users from ROS dependencies. In the future, someone might find executing Python code more powerful!
</details>

<details>
<summary>Where to go from here?</summary>
<br>
Play around and have ChatGPT call into any desired services.
</details>

<details>
<summary>Why not action clients instead of services?</summary>
<br>
By the time of this writing, action clients are not supported via rosbridge_suite yet, refer to https://github.com/RobotWebTools/rosbridge_suite/issues/697.
</details>
