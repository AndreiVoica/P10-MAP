# PMI_Python_Lib

Python API for interfacing with the Planar Motor Controller

## Using the library
Once the library is built, it can be installed using `pip install /path/to/.whl/file/`

Then, it can be used like any other python library, i.e:
```
from pmclib import system_commands as sys
from pmclib import xbot_commands as bot

sys.auto_connect_to_pmc()
bot.activate_xbots()
bot.linear_motion_si(4, 0.12, 0.12, 0.5, 10)
```

## Installing the library on Ubuntu
Installing the library on linux is a little bit trickier, since dotnet and some required libraries needs to be installed before installing pythonnet. If these libraries aren't installed beforehand, the installation of pythonnet will fail. Run the following commands:

Replace VER with the Ubuntu version. (i.e. 18.04 or 20.04)

```
sudo apt-get update

# install mono (.NET implementation)
sudo apt install gnupg ca-certificates
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 3FA7E0328081BFF6A14DA29AA6A19B38D3D831EF
echo "deb https://download.mono-project.com/repo/ubuntu stable-bionic main" | sudo tee /etc/apt/sources.list.d/mono-official-stable.list
sudo apt update
sudo apt install mono-devel

# install pythonnet
pip install wheel
pip install pythonnet
```

if building the wheel for pythonnet fails, try installing the following libaries

```
# install libraries needed to build pythonnet
sudo apt-get install clang
sudo apt-get install libglib2.0-dev
```

## VS Code Security settings

```
"terminal.integrated.profiles.windows": {
     "PowerShell": {
      "source": "PowerShell",
      "icon": "terminal-powershell",
      "args": ["-ExecutionPolicy", "Bypass"]
     }
    }
```
