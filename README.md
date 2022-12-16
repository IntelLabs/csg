# Critical Scenario Generation Toolkit

## Introduction 
  This repo offers the source code for Critical Scnenario Generation (CSG) toolkit. The toolkit aims to extract dynamic (trajectories) and static (road) elements from a given surveillance or dash camera video, and fromulated as OpenDrive and OpenScenario files as outputs.  
## How to build

1. Install system dependencies: 
```Shell
sudo apt-get install python3-tk
``` 
2. Install python3 package: 
```Shell
pip3 install Cython numpy
pip3 install -r requirements.txt
```
3. compile cython package
```Shell
cd src/track/kcf && make & cd ../../../
```

## How to run:
1. Edit variables in config.py (Optional)
2. Download machine learning models from [Google Drive](https://drive.google.com/file/d/1QAyzSvLjLBETgSk5w7hOp-1WGwD9aH9I/view?usp=sharing ) and unzip to "data" folder. 
3. Run 

- For survelliance video: 
```Shell
python3 CSG.py 
```

- For dash camera video: 

It uses [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2 "ORB-SLAM2") to get extrinsic parameters of moving camera.  Please install ORB-SLAM2 first and put the executable file under the "/src/SLAM" folder. 

Then run:
```Shell
python3 CSG_ego.py 
```

## Reqruiments

At least 1 GPU is needed. By default, the models are deployed on gpu:0. You can change your settings in "config" file. 
## Folder structure
```bash
+-- critical-scenario-generation
¦   +-- data
¦   ¦   +-- yolo
¦   ¦   +-- vehicle_reid
¦   ¦   +-- mask_rcnn_ego
¦   ¦   +-- mask_rcnn
¦   ¦   +-- lane
¦   ¦   +-- depth
¦   +-- src
¦   ¦   +-- SLAM
¦   ¦   ¦   +-- mono_kitti
¦   ¦   +-- ...
¦   +-- Lib_OpenSCENARIO
¦   +-- icon
¦   +-- CSG.py
¦   +-- CSG_ego.py
¦   +-- ScaleSolver.py
¦   +-- config.py
¦   +-- requirements.txt
```

## Licence
check [LICENSE](LICENSE)

## Citation
If you use our source code, please consider citing the following:
```bibtex
@InProceedings{csg2020,
  title={CSG: critical scenario generation from real traffic accidents},
  author={Zhang, Xinxin and Li, Fei and Wu, Xiangbin},
  booktitle = {IV},
  year={2020}
}
```
