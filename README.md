# Critical Scenario Generation Toolkit

## Introduction 
  This repo offers the source code for Critical Scnenario Generation (CSG) toolkit. The toolkit can extract dynamic(trajectories) and static (road) elements from a given surveillance or dash camera video. Then the elements will be organized and fromulated as OpenDrive and OpenScenario files.  
## How to build

1. Install system dependencies: 
```Shell
sudo apt-get install python3-tk
``` 
2. Install python3 package: 
```Shell
pip3 install -r requirements.txt
```
3. compile cython package
```Shell
cd src/track/kcf && make & cd ../../../
```

## How to run:
1. Edit variables in config.py (Optional)
2. Download machine learning models from: (TODO) 
3. Run 

For survelliance video: 
```Shell
python3 CSG.py 

```


## Reqruiments

At least 1 GPU is needed. By default, the models is deployed on gpu:0. You can change your settings in "config" file. 

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
