# 2021 SUMO Conference - Speed Adviosry on Transit Signal Priority (TSP)

* Author: Husan-Chih Wang
* Latest Update: 2022/07/31 
* Website: https://wangtechlab.com/
* Feel free to contact the author if you have any question!

![image](https://user-images.githubusercontent.com/53686476/182598526-be3df643-68a6-4fb2-a085-92bb9d9bdcfa.png)

## Introduction
This is the source code of the papaer "A Study of Applying Eco-Driving Speed Advisory on Transit Signal Priority" presented in 2021 SUMO Conference. 
You can download the [paper](https://www.tib-op.org/ojs/index.php/scp/article/view/92) from SUMO Conference Proceedings. 

<br>The project consists two parts: 
1. SUMO traffic simulation. 
2. Output data and parsers.  

## PART 1. SUMO Simulation
### Main program: bus_operation.py
The main program of SUMO simulation is **bus_operation.py**. This python file import several modules located in "moudles" folder. 
* **modules**
  * **OBUObject.py**
    * To simulate On-board Unit (OBU) in real-world cases. 
    * Each bus must be equipped with OBU to send TSP requests and enable speed advisory serivce. 
  * **RSUObject.py**
    * To simulate Road-side Unit (RSU) in real-world cases. 
    * Each signalized intersection must be installed with RSU to receive TSP requests and adjust signal timing settings. 
  * **BusOBU.py**
    * An inheritence class from "OBUObject", which overrides some parent functions to support the requirements of this project.    
  * **BusRSU.py**
    * An inheritence class from "RSUObject", which overrides some parent functions to support the requirements of this project.    
  * **SignalPlanObject.py**
    * A class for signal plans management. 
   
## PART 2. Output data parsers
The raw output data generated from SUMO would be in the "output" folder (defualt). You may use two parsers **performance_calculation.py** and **pbattery_calculation.py**, which are located in **"tools"** folder, to analyze output data. 

