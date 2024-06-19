<!-- <p align="center">
  <a href="" rel="noopener">
 <img width=200px height=200px src="https://i.imgur.com/6wj0hh6.jpg" alt="Project logo"></a>
</p> -->

<h1 align="center">reference_signal_generator</h1>

<div align="center">

  [![GitHub issues](https://img.shields.io/github/issues/leggedrobotics-usp/reference_signal_generator)](https://github.com/leggedrobotics-usp/reference_signal_generator/issues)
  ![GitHub pull requests](https://img.shields.io/github/issues-pr/leggedrobotics-usp/reference_signal_generator)
  [![GitHub forks](https://img.shields.io/github/forks/leggedrobotics-usp/reference_signal_generator)](https://github.com/leggedrobotics-usp/reference_signal_generator/network)
  [![GitHub stars](https://img.shields.io/github/stars/leggedrobotics-usp/reference_signal_generator)](https://github.com/leggedrobotics-usp/reference_signal_generator/stargazers)
  [![GitHub license](https://img.shields.io/github/license/leggedrobotics-usp/reference_signal_generator)](https://github.com/leggedrobotics-usp/reference_signal_generator/blob/main/LICENSE)

</div>

---

<p align="center"> A simple ROS node for generating reference signals for control systems.
    <br>
</p>

## 📝 Table of Contents
- [About](#about)
- [Getting Started](#getting_started)
- [Usage](#usage)
- [Feature requests](#feature_requests)
- [Contributing](#contributing)
- [Author](#author)

## 🧐 About <a name = "about"></a>
Wanting to quickly send some reference signals for your own control system in ROS2? Then this repo's for you. **reference_signal_generator** allows you to quickly bind a ROS2 node to a user-specified topic, and publish a reference signal with any number of inputs and a series of different signal types. The parameters are customized through ROS2 service calls.

You may call the services in many different ways, but this package comes with an extra: a custom [Foxglove](https://foxglove.dev/) panel made specifically for it! Check out [foxglove-reference-signal](https://github.com/leggedrobotics-usp/foxglove-reference-signal) now for more information!

## 🏁 Getting Started <a name = "getting_started"></a>
This repo is a standard ROS2 package (ament_python) that depends on custom service definitions.

### 🛠 Prerequisites

- A ROS2 workspace (colcon)
    - See [this tutorial](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) to learn how to create your own workspace.

- [reference_signal_srvs](https://github.com/leggedrobotics-usp/reference_signal_srvs)
    - This [other repo](https://github.com/leggedrobotics-usp/reference_signal_srvs) contains the service definitions for the reference signals advertised by **reference_signal_generator**. Definitions are kept in a dedicated package following the ROS2 [best practice](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html).

### 💻 Installing

As mentioned above, this repo is a standard ROS2 package. Thus, you simply have to clone it inside your workspace's **src** directory.

```bash
cd <path_to_your_ros2_ws>/src
git clone https://github.com/leggedrobotics-usp/reference_signal_generator.git
```

Don't forget to also clone the required [reference_signal_srvs](https://github.com/leggedrobotics-usp/reference_signal_srvs) package!

```bash
git clone https://github.com/leggedrobotics-usp/reference_signal_srvs.git
```

Then, use **colcon** to build.

```bash
cd <path_to_your_ros2_ws>
colcon build --symlink-install
```

## 🎈 Usage <a name="usage"></a>

Each instance of **reference_signal_generator** binds to a user-specified topic and creates multiple services for starting and stopping several different types of reference signals. Default topic name is **/reference_signal** and user can specify through command-line as follows:

```bash
ros2 run reference_signal_generator reference_signal_generator --ros-args -p topic_name:=<your_topic_name>
```

After starting the generator node, two services will be created to start and stop the reference signals. Use ``ros2 topic list`` to check them out. You must see:

- ``/<your_topic_name>/start`` Used to parameterize and start the reference signal
- ``/<your_topic_name>/stop`` Used to stop a signal created by a ``start`` before its *total_time*

See [reference_signal_srvs](https://github.com/leggedrobotics-usp/reference_signal_srvs) for more details on each type of reference signal. Currently supported types are Step, Ramp, Sine, Square, Triangle, Sawtooth and Chirp.

## 🔋 Feature requests <a name="feature_requests"></a>

Want another type of reference signal? Open an *Enhancement* issue describing it.

## 🤝 Contributing <a name="contributing"></a>

- Fork the repo
  - <https://github.com/leggedrobotics-usp/reference_signal_generator/fork>
- Check out a new branch based and name it to what you intend to do:
  - ````bash
    git checkout -b BRANCH_NAME
    ````
- Commit your changes
  - Please provide a git message that explains what you've done;
  - Commit to the forked repository.
    ````bash
    git commit -m "A short and relevant message"
    ````
- Push to the branch
  - ````bash
    git push origin BRANCH_NAME
    ````
- Make a pull request!

## ✍️ Author <a name = "author"></a>

<a href="https://github.com/Vtn21">
 <img style="border-radius: 50%;" src="https://avatars.githubusercontent.com/u/13922299?s=460&u=2e2554bb02cc92028e5cba651b04459afd3c84fd&v=4" width="100px;" alt=""/>
 <br />
 <sub><b>Victor T. N. 🤖</b></sub></a>

Made with ❤️ by [@Vtn21](https://github.com/Vtn21)

<!-- [![Gmail Badge](https://img.shields.io/badge/-victor.noppeney@usp.br-c14438?style=flat-square&logo=Gmail&logoColor=white&link=mailto:victor.noppeney@usp.br)](mailto:victor.noppeney@usp.br) -->

<!-- -  - Idea & Initial work -->

<!-- See also the list of [contributors](https://github.com/kylelobo/The-Documentation-Compendium/contributors) who participated in this project. -->
