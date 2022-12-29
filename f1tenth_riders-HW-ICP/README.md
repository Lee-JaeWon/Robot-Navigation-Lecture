# The F1TENTH - Riders

This is a sample project for [F1Tenth](https://f1tenth.org) challenges, the most recent being [F1Tenth ICRA 2022](https://riders.ai/challenge/67/f1-tenth-icra-2022/aboutCompetition). 

## Requirements

* Python 3.8
* Pip 22.0.3

There can be issues with installation when using older pip versions. 

## Installation

Clone this repository and install required packages:

```bash
git clone https://gitlab.com/acrome-colab/riders-poc/f1tenth-riders-quickstart --config core.autocrlf=input
cd f1tenth-riders-quickstart
pip install --user -e gym
```

Finally, check if the repo is working properly:

```bash
cd pkg/src
python -m pkg.main
```

## Entering the Competition

* Sign up to the competition platform [Riders.ai](http://riders.ai/), if you haven't already.
* Go to [My Account > Purchases](https://riders.ai/account/purchases).
* Enter the Redeem Code you have been provided (or ask an organizer for one).
* Once the (free) purchase is successful, go to your [Riders.ai Dashboard](https://riders.ai/dashboard/index). Now, you should be able to see F1Tenth ICRA 2022 challenge there, under Recent Challenges.
* Follow [that link](https://riders.ai/challenge/67/f1-tenth-icra-2022/aboutCompetition) and click Join Competition button (Note: this creates a Challenge Group, and makes you the admin of that group. If you want other people to compete with you on the same Team, you should invite them from [Challenge > My Team](https://riders.ai/challenge/67/f1-tenth-icra-2022/team) page).

## Developing your Driver

To develop your driver you can work in the folder [pkg/src/pkg](./pkg/src/pkg).

### Structure of a Driver

Let's take a look at the most basic Driver, which is in the file [drivers.py](./pkg/src/pkg/drivers.py)

```python
class SimpleDriver:    

    def process_observation(self, ranges=None, ego_odom=None):
        speed = 5.0
        steering_angle = 0.0
        return speed, steering_angle
```

A Driver is just a class that has a ```process_observation``` function which takes in  and odometry data and returns a speed to drive at along with a steering angle.

```ranges```: an array of 1080 distances (ranges) detected by the LiDAR scanner. As the LiDAR scanner takes readings for the full 360&deg;, the angle between each range is 2&pi;/1080 (in radians).

``ego_odom``: A dict with following indices:

```
{
  'pose_x': float,
  'pose_y': float,
  'pose_theta': float,
  'linear_vel_x': float,
  'linear_vel_y': float,
  'angular_vel_z': float,
}
```

```steering_angle```: an angle in the range [-&pi;/2, &pi;/2], i.e. [-90&deg;, 90&deg;] in radians, with 0&deg; meaning straight ahead.

### Choosing a Driver

Let's look at the [main.py](./pkg/src/pkg/main.py) file. The section shown below is all we need to worry about.

```python
...
# import your drivers here
from pkg.drivers import DisparityExtender

# choose your drivers here (1-4)
drivers = [DisparityExtender()]

# choose your racetrack here (Oschersleben, SOCHI, SOCHI_OBS)
RACETRACK = 'Oschersleben'
...
```

As shown in the comments above, we can import Drivers and then choose which ones we want to use. Let's import our SimpleDriver and choose it

```python
...
# import your drivers here
from pkg.drivers import DisparityExtender, SimpleDriver

# choose your drivers here (1-4)
drivers = [SimpleDriver()]
...
```

Now if you run the main.py file again, it uses our SimpleDriver

```bash
$ python main.py
```

To see some more complex processing, take a look at the GapFollower Driver which implements the [Follow The Gap Method](https://www.youtube.com/watch?v=7VLYP-z9hTw&ab_channel=Real-TimemLABUPenn)! Notice that it still has a ```process_lidar``` function which takes in LiDAR data and returns a speed and steering angle. That's all we'll ever need.

### Multi-Agent Racing

To practice racing multiple Drivers against each other, simply choose multiple Drivers! You may choose up to 4 drivers, but in practice the simulator will usually run very slowly if you choose more than 2. You may race the same Driver against itself by choosing it twice. If you try racing GapFollower against itself, you will find that it is not good at multi-agent racing! 

Here's how we would race GapFollower against SimpleDriver:

```python
# import your drivers here
from pkg.drivers import GapFollower, SimpleDriver

# choose your drivers here (1-4)
drivers = [GapFollower(), SimpleDriver()]

# choose your racetrack here (Oschersleben, SOCHI, SOCHI_OBS)
RACETRACK = 'Oschersleben'
```

### Changing Map

You may choose between the ordinary Sochi map or the Sochi Obstacles map. These are the sample maps that you can use in your tests. Competition will take place in Oschersleben map. The one with obstacles is hidden from participants. To switch maps, simply change the name of the selected `RACETRACK`

```python
...
# choose your racetrack here (Oschersleben, SOCHI, SOCHI_OBS)
RACETRACK = 'Oschersleben'
...
```

### Baseline Solution

The baseline solution for this competition is the DisparityExtender, which is included in the [drivers.py](./pkg/src/pkg/drivers.py) file. This Driver is an implementation of the [Disparity Extender Algorithm](https://www.nathanotterness.com/2019/04/the-disparity-extender-algorithm-and.html) which has proved successful in previous competitions.

This baseline should already pass the obstacle avoidance track as-is but it's not very fast! Speeding it up will introduce new challenges which can be handled with some thinking. Each function in this baseline also has tips on ways it can be improved.

You don't need to use the baseline solution but if you're not sure where to start this is a good place!

## Preparing a Submission

Prerequisites installs for submitting are: [Docker](https://www.docker.com/products/docker-desktop), [jq](https://stedolan.github.io/jq/)

Use the file [pkg/nodes/f1tenth_ros_agent.py](pkg/nodes/f1tenth_ros_agent.py) to choose the driver you are submitting, as shown below, where we choose the DisparityExtender driver:

```python
...
from pkg.drivers import DisparityExtender as Driver
...
```

If you're using additional dependencies, make sure they are provided in the `pkg/requirements.txt` file (or update your Docker image accordingly, if you know Dockerfile format).

Create an `.env` file at the root of the project with following contents:

```bash
RACE_MAP_PATH=/catkin_ws/src/f1tenth_gym_ros/maps/Oschersleben.yaml
RACE_MAP_IMG_EXT=.png
F1TENTH_AGENT_NAME=a1
F1TENTH_AGENT_IMAGE=a1
RIDERS_CHALLENGE_ID=67
RIDERS_API_HOST=https://api.riders.ai
```

**NOTE:** If you're on Linux, change `ROS_MASTER_URI=http://host.docker.internal:11311` lines in `docker-compose.yml` with `ROS_MASTER_URI=http://172.17.0.1:11311`. Otherwise your separate Docker instances won't be able to find each other.

Then, from the root of the project, build your submission:

```bash
docker-compose build agent
```

## Testing a Submission

The submission platform uses ROS to run the cars. Your car should race almost exactly the same in ROS as it did in the environment used in `Developing your Driver`, but it is a good idea to double-check sometimes by using ROS locally. This section will show you how to test your submission (if you want to) before you upload it.

Note: choose between `SOCHI.yaml` and `SOCHI_OBS.yaml` (or `Oschersleben.yaml`) in the `.env` file shown above to choose which map to test on (this will not have an effect on what map is used when you submit)

Start ROSCore & F1Tenth ROS Bridge:

```bash
docker-compose up --force-recreate roscore-dev bridge-dev
```

Go to http://localhost:6080 , if everything worked properly until now, you should see simulator window. 

Finally, from another terminal, launch the Driver agent:   

```bash
docker-compose up --force-recreate agent-dev
``` 

You should see your agent start driving along the track.

## Submitting

Add all your code and other files to be submitted to git. Submission script submits only the files in the repository.

Move into the [scripts](./scripts) directory and run the submission file:

```bash
cd scripts
./submit.sh
```

Follow the instructions displayed by the script. Your submission is the `challengeproject.tar.gz` file located at the project root. Extract it to make sure that it is not empty.

Once it's finished, check the competition page to see how you did! (it may take up to 15 minutes to process your new submission)

### Submission in Windows (With Docker)

If you already have Docker, just run `submit-with-docker.sh` in project root:

```bash
bash scripts/submit-with-docker.sh
```

If you don't have Docker or get any error, follow this instructions.

- Visit https://www.docker.com/products/docker-desktop and install Docker Desktop.
- To be sure, start Docker Desktop from the Windows Start menu. Then, from the Docker menu, select **Settings > General**. If you have installed Docker Desktop successfully, **Use WSL 2 based engine** check box will be checked by default. If not, check and click **Apply & Restart**.
- After that, visit here https://docs.microsoft.com/en-us/windows/wsl/install-win10#step-4---download-the-linux-kernel-update-package
- Perform steps 4-5-6 at the above address
- Then, to check the WSL mode, run :

```bash
wsl.exe -l -v
```
- Here, you can see the linux distro you installed in step 6. You need to set your chosen distro as default, run:

```bash
wsl --set-default <distro name>
```

For example, to set Ubuntu as your default WSL distro, run:

 ```bash
 wsl --set-default ubuntu
```

- Installation is done! Finally, run `submit-with-docker.sh` in project root:

```bash
bash scripts/submit-with-docker.sh
```

### Submission in Windows (with Python)

If previous options don't work, you can try to upload with Python script:

```bash
pip install docker six
python scripts/submit-with-docker.py
``` 
Enter your credentials and this script should start a docker container in backend that completes submission. After submission make sure that you visit Submissions page to validate newly created submission.
In 15 minutes, visit Results page to view results of your agent.

## Known issues (from original repo)

- If you run the `pip install...` command above and then later change your file structure in some way, you may get errors with `gym` such as `module 'gym' has no attribute 'make'`. The solution to this is to re-run the command `pip install --user -e gym/`.

- On MacOS Big Sur and above, when rendering is turned on, you might encounter the error:
```
ImportError: Can't find framework /System/Library/Frameworks/OpenGL.framework.
```
You can fix the error by installing a newer version of pyglet:
```bash
$ pip3 install pyglet==1.5.11
```
And you might see an error similar to
```
gym 0.17.3 requires pyglet<=1.5.0,>=1.4.0, but you'll have pyglet 1.5.11 which is incompatible.
```
which could be ignored. The environment should still work without error.

## FAQ

- How can I view the state of my submission?

Go to [Submissions page](https://riders.ai/challenge/67/f1-tenth-icra-2022/submissions) in Riders.ai, and then click on **View Status** for the related submission. 

If your agent has any issues (such as a syntax error), you will only see a single file "F1Tenth Bridge & Agent Log". By looking at this file you can understand why your agent haven't started (generally it's either a typo or an import issue).

If your agent starts successfully, you'll see two other logs (one for timed trial and one for obstacle avoidance). 

- How can I replay my submission?

Download log for Timed Trial or Obstacle Avoidance results, these logs should be in .jsonl format. 

Clone [F1Tenth Log Player](https://gitlab.com/acrome-colab/riders-poc/f1tenth-log-player) repo, update path of the log file in the main.py and run the Python file as described in the repo README.

- How can I add requirements to my agent?

If these are Python requirements, you can add these to the `pkg/requirements.txt` file, they will be automatically installed by the Agent docker image. If you want to add an arbitrary dependency, you'll need to update `compose/agent/Dockerfile`. Since these images are based on Ubuntu 20:04, you can use any dependency that's available with apt using `apt-get install -y {package-name}`.

## Citing
If you find this Gym environment useful, please consider citing:

```
@inproceedings{okelly2020f1tenth,
  title={F1TENTH: An Open-source Evaluation Environment for Continuous Control and Reinforcement Learning},
  author={O�Kelly, Matthew and Zheng, Hongrui and Karthik, Dhruv and Mangharam, Rahul},
  booktitle={NeurIPS 2019 Competition and Demonstration Track},
  pages={77--89},
  year={2020},
  organization={PMLR}
}
```
