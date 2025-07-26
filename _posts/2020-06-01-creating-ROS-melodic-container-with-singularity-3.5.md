---
layout: post
title:  "Create a ROS melodic container with singularity"
date:   2020-06-01
description: "Creating a ROS melodic container with singularity 3.5"
comments: true
singularity_install: "https://sylabs.io/guides/3.5/user-guide/quick_start.html"
docker_link: "https://hub.docker.com/_/ros"
ref1: "https://stackoverflow.com/questions/55645332/how-to-use-change-directory-cd-and-source-commands-in-a-singularity-recipe"
ref2: "https://sylabs.io/guides/3.1/user-guide/definition_files.html"
categories: ros
tags: singularity containers ros docker
---


Singularity can create portable and reproducible ROS environments. Moreover, we can run ROS environments that are unsupported for our distros without having to mess with virtual machines. It is quite easy to set up a singularity container for ROS melodic. We will be using the osrf docker image from [Docker Hub]({{page.docker_link}}). Before we begin, follow the instructions [here]({{singularity_install}}) to install singularity if you haven't.



### Creating a sandbox
It is recommended to run the command below with root permissions.

```sh
sudo singularity build --sandbox melodic/ docker://osrf/ros:melodic-desktop-full
```
This will create a directory named `melodic` inside the current working directory. Once the sandbox is built, we can test it by running `roscore`.

```sh
singularity exec -p melodic/  bash -c '/ros_entrypoint.sh roscore'
```

Where `ros_entrypoint.sh` is a bash script inside the `melodic/` sandbox that sources `setup.bash` for us.


### Update and modification

Let's now proceed by updating the packages (you can also install packages that you need). For this we will use the `shell` command.
Note, that the `--writable` option is necessary to mount the container as read-write.

```sh
sudo singularity shell --writable melodic/
```

Once we are inside the container shell, we can run `apt update` and install our packages. We can also add the update the `/root/.bashrc` to our taste. Or, ditch bash altogether and install `zsh`.

```sh
> apt update
> mkdir -p /root/Projects #if you want to create extra directories
```


### Binding filesystems

By default, singularity will mount the current working directory inside the container, if the directory exists inside. You can use the `--bind` option to mount host filesystems to directories inside the container. For example, the following command will start the shell and mount my host system's `/dev/sda5` to  `/root/Projects`.

```sh
sudo singularity shell --bind /dev/sda5:/root/Projects --bind /run --writable melodic/
```

In order to run gui applications, use `--bind /run`.



### Convert container format

Finally, we can create `img` file from the `sandbox` by running the following command.

```sh
sudo singularity build melodic.img melodic/
```


### Building from recipe files

The steps mentioned above can be build easily with a recipe file too.

{% highlight docker linenos %}
#melodic-husky.def
Bootstrap: docker
From: osrf/ros:melodic-desktop-full-bionic

%help
    Singularity container with ROS melodic with husky

%labels
    Author sj@email.org
    

%post
    apt -y update
    apt -y install ros-melodic-husky-navigation \
                   ros-melodic-husky-gazebo \
                   ros-melodic-husky-viz
    bash -c "source /opt/ros/melodic/setup.bash"
{% endhighlight %}

Build the container with the following command

```sh
sudo singularity build melodic-husky.sif melodic-husky.def
```

### Reference
[1] [{{page.ref1}}]({{page.ref1}})

[2] [{{page.ref2}}]({{page.ref2}})
