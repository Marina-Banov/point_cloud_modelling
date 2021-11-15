# plane_detection
## PCL Installation
### Ubuntu 20.04
```
pip install cython numpy
```

```
sudo apt-get update
sudo apt-get install libpcl-dev
sudo apt-get install python3-pcl pcl-tools
```

### Windows 10
- download and install [python 3.6.0](https://www.python.org/downloads/release/python-360/) (Windows x86 executable installer)
- download and install [pcl 1.8.1](https://github.com/PointCloudLibrary/pcl/releases/) (pcl-1.8.1/PCL-1.8.1-AllInOne-msvc2017-win32.exe)
- add `%OPENNI2_REDIST%`, `%OPENNI2_ROOT%\Tools` and `%PCL_ROOT%\3rdParty\VTK\bin` to `PATH` environment variable
- add `%PCL_ROOT%\lib\pkgconfig` to `PKG_CONFIG_PATH` environment variable
- download [gtk](http://www.tarnyko.net/dl/gtk.htm) (gtk+-bundle_3.6.4-20130513_win32.zip)
- download and install [mingw](https://sourceforge.net/projects/mingw/) (mingw-developer-tools, mingw32-base, mingw-gcc-g++, msys-base)
- install [Microsoft Visual C++ build tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/)
  ![Microsoft Visual C++ build tools](https://docs.microsoft.com/en-us/answers/storage/attachments/34873-10262.png)


- **(RECOMMENDED)** initialize virtual environment
  ```
  virtualenv --python=%PYTHON36_ROOT%\python.exe env
  env\Scripts\activate
  ```
- ```
  pip install cython numpy
  pip install --upgrade setuptools
  ```
- download [python-pcl](https://github.com/strawlab/python-pcl/releases), unzip into `env\Lib\site-packages` and position yourself inside that folder
- unzip `gtk\bin` into `pkg-config`
- update `pcl\pcl_defs.pxd` (line 512):
  ```
  FROM: cdef extern from "pcl/pcl_base.h" namespace "pcl":
    TO: cdef extern from "pcl/pcl_base.h" namespace "pcl" nogil: 
  ```
- ```
  pip install .
  ```
