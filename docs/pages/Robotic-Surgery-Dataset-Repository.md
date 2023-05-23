# Introduction

This page lists some currently available datasets for robotics surgery, sorted by the information they include. It also lists data that we recommend to be collected for groups running user studies going forward. Please contact us if you would like your dataset to be included on this page. 

# Dataset requirements

## Data description 
In order to easily share data from studies between groups, we recommend the following data to be reported for every dataset. 
* Clinical Application (Suturing? Knot tying? Cutting? Etc.)
* Data Types (video, position, velocity, torque, current, multiple...)
* Configuration Data:  camera intrinsic (and extrinsic) parameters, if available, base frames (e.g., for PSMs, ECM)
* Kinematics parameters - include JSON dVRK config files
* Environment (phantom, animal, patient...)
* Users: experts, non-experts, mix (be careful to avoid personal data)
* System (dVRK, da Vinci (with permission from ISI), simulated, Raven, etc.)
* Trial size:  number of participants and number of trials per participant
* Data Size: megabytes
* Background Information: e.g., part of clinical trial
* Citation: credit if data is used
* Version of software/firmware the data was collected on

## Data/privacy protection
Please consider your institution's and country's rules regarding data privacy and company policies before uploading any data. It is up to each individual group to obtain permission and user consent to share the data publicly. Time stamps should include an offset to further anonymize studies. Consider asking for the following in all consent forms before conducting user studies to maximize usability of the data:
* Capture and use data for any future research (as opposed to a single study)
* Permission to share anonymized data with other sites
* Permission to store data indefinitely

# Datasets

|Dataset| Clinical application | Data type | Environment | Users | System | 
|-------|----------------------|-----------|-------------|-------|--------|
|[JIGSAWS](https://cirl.lcsr.jhu.edu/research/hmm/datasets/jigsaws_release/)| Suturing, Knot-Tying, Needle-Passing | Video, Kinematics, Phase Annotation | Phantom | Mixed (8)| da Vinci Research API |
|[CUHK-JHU MCPeT](https://github.com/YonghaoLong/CUHK-JHU-MCPeT) | Peg Transfer |  Video, Kinematics, Phase Annotation | Phantom | Novice (2) | dVRKs, 2 systems |
|[Stanford Single PSM Manip](https://github.com/enhanced-telerobotics/single_psm_manipulation_dataset) | Single tool retraction/palpation |  Video, Kinematics, Force data | Phantom | N/A | dVRK 1.7 |

# Citations

If you use the datasets, please cite the below references.

## JIGSAWS
```bibtex
@inproceedings{gao2014jhu,
  title={Jhu-isi gesture and skill assessment working set (jigsaws): A surgical activity dataset for human motion modeling},
  author={Gao, Yixin and Vedula, S Swaroop and Reiley, Carol E and Ahmidi, Narges and Varadarajan, Balakrishnan and Lin, Henry C and Tao, Lingling and Zappella, Luca and B{\'e}jar, Benjam{\i}n and Yuh, David D and others},
  booktitle={MICCAI workshop: M2cai},
  volume={3},
  pages={3},
  year={2014}
}
```
## CUHK-JHU MCPeT
```bibtex
@inproceedings{long2021relational,
  title={Relational graph learning on visual and kinematics embeddings for accurate gesture recognition in robotic surgery},
  author={Long, Yonghao and Wu, Jie Ying and Lu, Bo and Jin, Yueming and Unberath, Mathias and Liu, Yun-Hui and Heng, Pheng Ann and Dou, Qi},
  booktitle={2021 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={13346--13353},
  year={2021},
  organization={IEEE}
}
```
## Stanford Single PSM Manip.
```bibtex
@inproceedings{chuaForceEstimationRobotAssisted2021,
 title = {Toward Force Estimation in Robot-Assisted Surgery Using Deep Learning with Vision and Robot State},
 booktitle = {2021 IEEE International Conference on Robotics and Automation (ICRA)},
 author = {Chua, Zonghe and Jarc, Anthony M. and Okamura, Allison M.},
 year = {2021},
 month = may,
 pages = {12335--12341},
 doi = {10.1109/ICRA48506.2021.9560945}
}
