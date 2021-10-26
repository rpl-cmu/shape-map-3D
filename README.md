# Shape map 3-D: Efficient shape mapping through dense touch and vision

[![License: X11](https://img.shields.io/badge/license-X11-yellowgreen)](https://github.com/rpl-cmu/shape-map-3D/blob/master/LICENSE) &nbsp; <img height="20" src="media/rpl.png" alt="RPL-logo" /> &nbsp;&nbsp; <img height="20" src="media/robotouch.png" alt="Robotouch-logo" />

![cover](/media/cover.jpg) &nbsp;&nbsp;&nbsp; ![shape-map](/media/shape-map.gif)

Base library for visuo-tactile shape mapping with gelsight and depth camera. For more information, consider our work [Efficient shape mapping through dense touch and vision](https://www.cs.cmu.edu/~sudhars1/shape-map/). 

## Roadmap
The library is still being actively updated, please check back soon for a stable version!
- [x] Initial code upload
- [ ] Data upload 
- [ ] Compilation fixes
- [ ] Enhanced documentation

## Folder structure
- `matlab/`: Executable and utility scripts for mapping
  - `shape-map/`: 3-D and 2-D mapping scripts for both simulated and real-world data
  - `utils/`: helpers scripts 
- `gpfactor/`: cpp header wrapped with the gtsam + MATLAB library

## Requirements
- [gtsam](https://github.com/borglab/gtsam)
- [MATLAB](https://www.mathworks.com/products/matlab.html)

## Other dependencies
- [spatialmath-matlab](https://github.com/petercorke/spatialmath-matlab)

## Third-party scripts
- [splitFV](https://www.mathworks.com/matlabcentral/fileexchange/27667-splitfv-split-a-mesh)
- [stlwrite](https://www.mathworks.com/matlabcentral/fileexchange/20922-stlwrite-write-ascii-or-binary-stl-files)

## Compilation
- **GTSAM**: Checkout gtsam 4.0.2 `master` branch (not `develop`):
    - Using the default CMakeLists without modification
        ```
        cmake -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_WITH_TBB=OFF -DGTSAM_INSTALL_MATLAB_TOOLBOX=ON ..
        sudo make install -j
        ```
    - MATLAB toolbox check
        - Open MATLAB
        - add `addpath /usr/local/gtsam_toolbox` to the `startup.m`
        - Test `gtsamExamples`
- C++ header compilation:
    ```
    mkdir build 
    cd build
    cmake -DGPFACTOR_BUILD_TOOLBOX:OPTION=ON -DCMAKE_BUILD_TYPE=Release -DGTSAM_TOOLBOX_INSTALL_PATH:PATH=/usr/local/gtsam_toolbox ..
    sudo make install 
    ```
    - `sudo ldconfig` in terminal
    - open MATLAB here
    - Run `.m` files

## Citation 
Feel free to use the library as you please. If you find it helpful, please consider referencing: 

```BibTeX
@misc{suresh2021efficient,
      title={Efficient shape mapping through dense touch and vision}, 
      author={Sudharshan Suresh and Zilin Si and Joshua G. Mangelson and Wenzhen Yuan and Michael Kaess},
      year={2021},
      eprint={2109.09884},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
      }
```
