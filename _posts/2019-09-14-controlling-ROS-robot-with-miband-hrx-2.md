---
layout: post
title:  "Controlling ROS Turtlebot3 with Miband - Part II"
date:   2019-09-14 00:00:00-0400
description: "Coding a Xiaomi Miband Controller for ROS Gazebo Turtlebot3 simulation."
comments: true
image: "/images/controlling-ROS-robot-with-miband-hrx-2-img01.png"
image1: "/images/controlling-ROS-robot-with-miband-hrx-1-img02.png"
image2: "/images/controlling-ROS-robot-with-miband-hrx-1-img03.gif"
gh_file1: "https://github.com/4lhc/ROS/blob/master/learning_ws/src/x1_miband_control/src/generic_robot.py"
gh_file2: "https://github.com/4lhc/ROS/blob/e8adca517d6d338cf86a63c005fb4a2c9cdcdf44/learning_ws/src/x1_miband_control/bin/miband_controller.py"
jupyter_NB: ""
tags: ros ble
categories: ros
---

After successfully establishing communication [(read Part I)]({% post_url 2019-09-05-controlling-ROS-robot-with-miband-hrx-1 %}) with the MiBand HRX, we can start writing the controller node.

We will start by setting up project ros package.

### 1. Setup
Create a package named ``x1_miband_control`` with a dependency on rospy.
{% highlight bash %}
catkin_create_pkg x1_miband_control rospy
cd x1_miband_control/src
{% endhighlight %}

We have three other runtime dependencies to make the MiBand controller work. [PyCrypto](https://github.com/dlitz/pycrypto), [bluepy](https://github.com/IanHarvey/bluepy) & the [MiBand HRX](https://github.com/4lhc/MiBand_HRX) library that we created in Part I.
Once inside the ``x1_miband_control/src`` directory we can clone the dependencies. Make sure that you checkout the right versions

{% highlight bash %}
git clone -n https://github.com/4lhc/MiBand_HRX
cd MiBand_HRX
git checkout 1711a218ab66bfba25aa7de717452574301dcba5
{% endhighlight %}

Repeat the same for the other dependencies. From ``xi_miband_control/src``
{% highlight bash %}
git clone -n https://github.com/dlitz/pycrypto
cd pycrypto
git checkout 7fd528d03b5eae58eef6fd219af5d9ac9c83fa50
cd ..
git clone -n https://github.com/IanHarvey/bluepy
cd bluepy
git checkout dc33285f31a873fab92c22e8839c44899f82b041
{% endhighlight %}

Bluepy and pycrypto has to be built and installed using setup.py inside their respective directories. I am yet to find a way to automate the build of these dependencies using CmakeLists.txt. Inside both bluepy & pycrypto directories run,

{% highlight bash %}
python setup.py build && python setup.py install
{% endhighlight %}

Furthermore, I had to move ``xi_miband_control/src/bluepy/bluepy`` to ``xi_miband_control/src/bluepy`` inorder for bluepy to work.


Next, we will create a setup.py in out package root with the following content and uncomment the ``catkin_python_setup()`` macro in ``CmakeLists.txt`` to make sure that our dependencies get installed.

{% highlight python linenos %}
# For catkin_make
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['MiBand_HRX', 'bluepy', 'pycrypto'],
    package_dir={'': 'src'},
)

setup(**setup_args)
{% endhighlight %}


Finally, run ``catkin_make`` from the catkin workspace root to build everything.


### 2. MiBand Controller

We will start by importing the necessary modules. Multithreading is used to prevent the ``band.start_raw_data_realtime()`` method from blocking.
[``generic_robot``]({{page.gh_file1}}) contains a simple ``Robot()`` class definition.


[``vel_control()``]({{page.gh_file2}}#L42) method sets the robot's linear x velocity to a value proportional to the pitch of the MiBand and the angular z velocity to a value proportional to MiBand's roll. It also applies a low pass filter to the roll & pitch.

[``start_control()``]({{page.gh_file2}}#L27) will run as long as data is available from the MiBand. If the robot crashes into a wall, the method sends a vibration alert to the MiBand.


{% highlight python linenos %}
#!/usr/bin/env python
# part of https://github.com/4lhc/ROS/tree/master/learning_ws/src/x1_miband_control
# Gist: https://gist.github.com/4lhc/d42bba11b870be4c5860b00256fbec27
import rospy
from MiBand_HRX.base import MiBand2
from MiBand_HRX.constants import ALERT_TYPES
from threading import Thread
from generic_robot import Robot


class MiBandController():
    def __init__(self, MAC):
        self.robot = Robot()
        self.band = MiBand2(MAC, debug=True)
        self.band.setSecurityLevel(level="medium")
        if not self.band.authenticate():
            self.band.initialize()
            self.band.authenticate()

        self.t1 = Thread(target=self.start_data_realtime)
        self.prev_signal = (0, 0, 0)

    def accel_call_back(self, x):
        pass

    def start_data_realtime(self):
        self.band.start_raw_data_realtime(self.accel_call_back)

    def start_control(self):
        rospy.loginfo("Starting Control")
        self.t1.start()
        while not self.band.is_realtime_stopped():
            self.vel_control()
            if self.robot.get_scenario() == 7:
                self.band.stop_realtime()
                self.band.send_alert(ALERT_TYPES.MESSAGE)
        self.robot.stop()

    def stop_control(self):
        self.band.stop_realtime()



    def vel_control(self):
        #we will drive lin_vel.x proportional to pitch & ang_vel.x proportional
        #to roll. We don't have yaw information.
        r1, p1, y1 = self.band.get_euler()
        rospy.loginfo("{} {} {}".format(r1, p1, y1))
        r0, p0, y0 = self.prev_signal
        a = 0.5
        #simple low pass filter
        r2, p2, y2 = (a*r1 + (1-a)*r0, a*p1 + (1-a)*p0, 0)

        self.prev_signal = (r2, p2, y2)
        self.robot.set_velocity(lin_vel=p2, ang_vel=-r2)

if __name__ == "__main__":
    rospy.init_node("miband_controller")
    micontrol = MiBandController(MAC="C5:64:8F:FC:47:F2")
    try:
        micontrol.start_control()
    except KeyboardInterrupt:
        micontrol.stop_control()
{% endhighlight %}

### 3 Test Run

{% include youtube.html id="Zg9SUYd6MYA" width="200%" %}

### 4 Next
This was a fun project. It still requires a lot of improvements. If time permits, I would like to build a game of tag, with multiple MiBand controlled robots in gazebo -- would be cool!
