<>
<h1 align="center">Flight Arena of the Laboratorio de Control Avanzado de Sistemas Ciberfísicos</h1>
<p>The flight arena of Lab-CASCi is a space that allows to test algorithms designed in MATLAB/Simulink for air and mobile systems, tracked by OptiTrack cameras, a precision motion capture system. This repository gathers the tools of ROS environment that links MATLAB/Simulink with OptiTrack data and commands the unmanned vehicules.

<details>
  <summary>Table of contents</summary>
  <ol>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#requirements">Requirements</a></li>
      </ul>
    </li>
  </ol>
</details>


<h3 id="#geting-started">Getting started</h3>
<p>This repository is created with <a href="http://wiki.ros.org/noetic">Ros Noetic </a href="https://ubuntu.com/download"> in <a> Ubunu 20.04.</a> Before using this repository, you must complete its installation.</p>

<b id="#requirements">requirements</b>
## Requirements
* vrpn-ros
  ```sh
  sudo apt-get install ros-noetic-vrpn-client-ros
  ```
* rosserial
  ```sh
  sudo apt-get install ros-noetic-rosserial
  sudo apt-get install ros-noetic-rosserial-arduino
  ```

<h3 id="#content">Content of this repository</h3>
This repository contains the following workspaces:
  <ol>
    <li>
      <a href="https://github.com/LuisJongInd/Lab_CASCi/tree/master/omni_robots">Omnidirectional robots</a>
    </li>
  </ol>

  

