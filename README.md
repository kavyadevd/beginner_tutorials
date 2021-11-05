[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
# ROS Beginner Tutorials

A beginner ROS package that implements simple publisher-subscriber nodes in ROS


## Installations

To install ROS refer [this link](http://wiki.ros.org/ROS/Installation).

## Requirements / Assumptions
Project requires and evironment with Ubuntu 18.04, ROS Melodic


### Step 1. Clone repository and build the package
```bash
git clone --recursive https://github.com/kavyadevd/beginner_tutorials.git
cp <repository_path> <catkin_workspace_path/src/>
cd <catkin_workspace>
source ./devel/setup.bash
catkin_make
```

### Step 2. Make sure that a roscore is up and running
```bash
roscore
```
### Step 3. In new terminal run publisher node 
```bash
rosrun beginner_tutorials talker
```
### Step 4.  In new terminal run subscriber node 
```bash
rosrun beginner_tutorials listener
```

#### To run launch file

```bash
roslaunch beginner_tutorials LaunchTalkerListener.launch frequency:=<desired-frequency>
```
This command will launch published and subscriber.

#### To run service
```bash
rosservice call /ServiceFile "<desired-custom-message>"
```

#### To invoke rqt logger and console on separate terminals run the following two commands
```bash
rosrun rqt_console rqt_console
rosrun rqt_logger_level rqt_logger_level
```


## Plugins


- CppCheckEclipse

    To install and run cppcheck in Eclipse

    1. In Eclipse, go to Window -> Preferences -> C/C++ -> cppcheclipse.
    Set cppcheck binary path to "/usr/bin/cppcheck".

    2. To run CPPCheck on a project, right-click on the project name in the Project Explorer 
    and choose cppcheck -> Run cppcheck.
    
    3. To run on terminal
    ```bash
    cppcheck --enable=all --std=c++11 -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp -or -name *.h | grep -vE -e "^./build/" -e "^./vendor/") >     Results/cppcheckoutput.xml
    ```
    Results are present at Results/cppcheckoutput.xml
    
- Cpplint
   1. To run cpplint on terminal
   ```bash
   cpplint $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/") $( find . -name *.hpp | grep -vE -e "^./build/" -e "^./vendor/") >                    Results/cpplintoutput.txt
   ```
   Results are present at Results/cpplintoutput.xml

- Google C++ Style

    To include and use Google C++ Style formatter in Eclipse

    1. In Eclipse, go to Window -> Preferences -> C/C++ -> Code Style -> Formatter. 
    Import [eclipse-cpp-google-style][reference-id-for-eclipse-cpp-google-style] and apply.

    2. To use Google C++ style formatter, right-click on the source code or folder in 
    Project Explorer and choose Source -> Format

[reference-id-for-eclipse-cpp-google-style]: https://raw.githubusercontent.com/google/styleguide/gh-pages/eclipse-cpp-google-style.xml

- Doxygen

    The HTML page for project outlines can be generated using the following commands
    ```bash
    doxygen -g
    doxygen Doxyfile
    ```


## Licensing
The project is licensed under [MIT License](https://opensource.org/licenses/MIT). Click [here](https://github.com/kavyadevd/beginner_tutorials/blob/main/LICENSE) to know more
