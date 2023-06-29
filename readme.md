# Co-Simulation testbed and controllers for compressor-based loads providing grid balancing services

## Description

This code has been developed for the ARPA-E project "Overcoming the Technical Challenges of Coordinating Distributed Load Resources at Scale". 
Its purpose is to simulate a feeder with residential A/Cs that are being controlled for the provision of load balancing services.
The simulation environment can run stand-alone, where voltages are set artificially, or it can run linked with GridLAB-D, where GridLAB-D simulates 
the distribution network down to the house nodes, and where the MATLAB code simulates the Thermostatically Controlled Loads (TCLs) that are connected to the distribution feeder. 

## How to run

**NOTE:** If cloning the repository locally in a Windows machine fails due to the length of some file names, you can run Git Bash as an administrator and execute the command: git config --global core.longpaths true

You will first need to download the repo from https://github.com/gridlab-d/Taxonomy_Feeders and unzip it into a folder called "Taxonomy_Feeders-master" located in the main directory of this codebase (the directory that includes the main_runSim script).

The following three scripts need to be executed sequentially. 
1. pre1_constructFeeders: constructs a user-defined set of PNNL feeders from feeder files located in Taxonomy_Feeders.
2. pre2_tclSetup: constructs the house models.
3. main_runSim: runs a simulation with the specified controller.
	* To simulate the distribution feeder, useGLD should be set to 1. In that case, the script will generate a batch file that can be executed from the command line and will run a simulation using Matlab and GridLAB-D.
	* Otherwise, useGLD should be set to 0. In that case, the simulation can be run directly within Matlab as no link with GridLAB-D will be used.

## Main Options

* **Simulation timestep**: Determines how often the temperature dynamics and on/off modes are updated. Can be specified in pre1_constructFeeders through the timeStepInSec variable.
* **Population size**: Determines the size of the controlled population. Can be specified implicitely in pre1_constructFeeders through the feederOptions.percentTclOptionSet variable.
* **Mixture of 2-zone houses**: Determines what percentage of the controlled population corresponds to houses with 2 zones and 1 comporessor. Can be specified in pre1_constructFeeders through the perc_2zone variable.
* **Feeder head voltage**: Determines the pu voltage value at the feeder head. Can be specified pre1_constructFeeders though the feederOptions.voltageRegulatorOptionSet variable.
* **Feeder file**: Determines what feeder to use. Can be specified pre1_constructFeeders though the feederOptions.setOfTaxFiles variable. Should match the corresponding choices in pre2_tclSetup and main_runSim.
* **Scenarios for controlled houses**: Determines the ambient conditions, building parameter distribution and heterogeneity, lockout duration, setpoint ranges. All these can be specified in pre2_tclSetup and should match the choices in main_runSim.
* **Distribution network option**: Option to run with or without a feeder model. If network voltages are required, GridLAB-D will be used. This option can be specified in main_runSim through the simulationOptions.useGLD variable.
* **Simulation duration**: Duration in hours for the simulation. Can be specified in main_runSim through the simulationOptions.simLength variable.
* **Communication network**: Option to consider delays and packet dropouts in the communication network. Can be controlled by changing the simulationOptionSet.commScenario variable in main_runSim.
* **Controller timestep**: Determines at what resolution the controllers will be acting. Can be specified in main_runSim through the simulationOptions.controlTimeStep variable.
* **Controller type**: There are three main controllers that can be currently used, a Markov-based controller, a Packetized Energy Management (PEM)-based controller and a baseline PID controller. Can be specified in main_runSim through the simulationOptions.controllerType variable.
* **Controller parameters**: The specific variables associated with each controller can be set in controllerSetup.
* **Tracking signal options**: The are 4 types of regulation signals that can be used to assess controller performance, including PJM RegD signals. The type and amplitude of the tracking signals can be set in main_runSim.

## Authors
* G. S. Ledva, University of Michigan (now at Virtual Peaker, Inc.)
* I. M. Granitsas, University of Michigan
* O. Oyefeso, University of Michigan
* S. A. Nugroho, University of Michigan (now at Cummins, Inc.)
* P. Phanivong, University of California at Berkeley
* I. A. Hiskens, University of Michigan
* J. L. Mathieu , University of Michigan

## License
This project is licensed under a GNU General Public License. Details are provided in license.txt.
