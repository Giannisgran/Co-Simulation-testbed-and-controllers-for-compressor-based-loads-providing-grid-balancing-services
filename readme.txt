This code has been developed for the ARPA-E
project "Overcoming the Technical Challenges of Coordinating Distributed Load 
Resources at Scale." The simulation environment can run stand-alone where 
voltages are set artificially. The simulation environment can also run linked
with GridLAB-D, which is the main purpose of the simulation environment, where
GridLAB-D simulates the distribution network down to the house nodes, and
where the MATLAB code simulates the houses and TCLs that are connected to a
distribution feeder. The documentation lists the files and directories at the
time this was written and explains how to setup and run a simulation with and
without GridLAB-D.

List of contents:
- readme: this document. See each script and function for additional details.
- main_constructFeeders: constructs a user-defined set of PNNL feeders from 
	feeder files located in Taxonomy_Feeders. It takes the feeder,
	populates house and TCL models onto the feeder based on a defined
	average house size and static, aggregated planning loads. The end
	result for each feeder constructed is a directory indicating the 
	feeder, a glm file that contains the feeder information for GridLAB-D,
	and matlab files that allow the constructed feeder and house/tcl models
	to run over time.
- main_simulateWithoutGLD: simulates a previously constructed feeder file 
	without linking with gridlab-d. Voltages are set artificially, but
	otherwise the script uses the same code as is used to run a simulation
	with gridlab-d.
- Taxonomy_Feeders: These are the PNNL files associated with the feeder
	taxonomy models (see 
	http://gridlab-d.shoutwiki.com/wiki/Feeder_Taxonomy). The contents
	are taken from 
	https://sourceforge.net/p/gridlab-d/code/HEAD/tree/Taxonomy_Feeders/
	and are left unmodified where possible. Some feeder files have been
	modified to fix errors in the file (e.g., nodes and meters being
	given different nominal voltage values). 
- plotsAndPlottingFunctions: These are miscellaneous plots and plotting scripts
	that are put together to generate plots for reports and such. The
	individual contents will not be detailed. 
- DocumentationFigures: This directory contains figures that may be used in the
	GitHub wiki to document the code. The contents of this directory with
	not be detailed.
- dataFiles: this directory contains data files that are used to construct 
	the feeders, the house models, the tcl models, and any other files
	needed to simulate the feeder over time. The contents of this folder
	are the following:
	- inrushP, inrushQ: directory containing the inrush statistics for the
	  47 houses, which we use to generate inrush probability distributions
	  for each of the houses that are connected to the feeder
	- noiseP, noiseQ: currently aren't used in the actual simulation, but
	  these directories contain the noise distributions for additive noise
	  that isn't captured in the modelling process. The noise distribution
	  corresponds to the histogram of residuals resulting from multiple
	  linear regression for the active and reactive power models. 
	- processedFilesP, processedFilesQ: files that contain the active and 
	  reactive power for the 47 houses over the one day included in the 
	  Milestone 2 report. Used in a script within the
	  plotsAndPlottingFunctions directory. 
	- processedFilesStates, processedFilesTemp, processedFilesV: on/off 
	  modes, outdoor temperature, and voltage for the 47 houses where the
	  data corresponds to the same times as the processed power data above. 
	- coefficients_reactive_multiple_regression, 
	  coefficients_real_multiple_regression: contain the reactive and real
	 coefficients generated from multiple linear regression on the PSI data
	 on one-second timescales over the summer, where inputs of the 
	  regression are voltage and outdoor temperature and outputs are 
	  the active and reactive power of the air conditioner
	- matlabLink_baseGSL: creates the portion of the base link file used
	  to run gridlabd with matlab. The base file is used when constructing
	  the feeders to generate the feeder-specific link file
	- square_footage_probability: distribution of the square footages
	  of the houses within the PSI one-second data set and the probability
	  of each of those square footages (occurances over total houses)
	- square_footage_reactive, square_footage_real: average real and 
	  reactive power values over days in the summer. Was going to be used
          to determine background load values based on house square footage,
	  but code currently uses a formula for gridlabd based on the house 
	  square footage	  
	- weather_PSI_doNotShare: PSI data that contains the weather data
	  that corresponds to the one-second power data. Weather data is
 	  sampled on one hour time intervals.
	
- functions: contains a list of functions called by and within the scripts
    in the main body of the code. We detail each of the functions below:
    - calculateInitialConditions
        - Called from main_constructFeeders and simulates the TCLs 
            populated on the feeder being constructed using the first 
            outdoor temperature of the simulation. Is used to find an
            initial condition for the TCLs so that there are no transiet
            dnyamics when starting the actual simulation. 
    - check_onOff_mode
        - Called within TCL_pop_sim_3state when updating the TCLs for a new
            time-step. Advances the internal temperatures, on/off mode, 
            power draw, and heat injection (negative for air conditioners)
            for the new time-step
    - fastinterpcol
        - A function from MATLAB's codebase that interpolates along the 
            columns of a matrix. Used in performing random draws from
            discrete probability distributions. 
    - generateControlSignal
        - Called within calculateInitialConditions and onSync_base. Is the
            function that executes the control algorithm at each time-step
    - generateFeeder
        - Called within main_constructFeeders to construct the feeder,
            populate houses and TCLs onto the feeder, and generate the
            files needed to simulate the feeder. 
    - GenerateRandomHouseParameters
        - Called within generateFeeder to construct the house models that
            where the houses have been populated onto the feeder using the
            nodal power value in the feeder model, the average power value
            of houses given the regionalization parameters. Determines the
            baseload (non-AC demand) of the house based on its square
            footage. Determines whether the house has an air conditioner
            based on a random draw and the portion of houses with air
            conditioners in the region. 
    - GenerateRandomTclParameters
        - Called by GenerateRandomHouseParameters to construct the set of 
            TCLs connected to the houses. 
    - getOutdoorTemperature
        - Called within main_constructFeeders to get the outdoor
            temperature data from the PSI weather data for the simulation
            period. Assumes 1-hour measurement intervals, and finds
            temperatures that correspond to each hour within the simulation
            then linearly interpolates those down to the time-step interval
            of the simulation.
    - onInit_base
        - Used in main_constructFeeders where the file is copied into the 
            feeder directory being constructed. Called by GridLAB-D when
            initializing the MATLAB portion of the simulation. 
    - onSync_base
        - Used in main_constructFeeders where the file is copied into the 
            feeder directory being constructed. Called by GridLAB-D at each
            pass within a time-step of the simulation. It should be called 
            at least twice within a time-step to check for convergence of
            the load models. 
    - onTerm_base
        - Used in main_constructFeeders where the file is copied into the 
            feeder directory being constructed. Called by GridLAB-D after
            the simulation is complete.
    - regionalizationGSL
        - Called within main_constructFeeders to set some regional 
            parameters like portion of houses with ACs for the feeder that
            are used to populate houses and TCLs to the feeder.
    - TaxFeederDataGSL
        - Called within main_constructFeeders to set some values for the 
            feeder based on PNNL taxonomy feeder information. These include
            nominal voltages, substation power rating, and average house
            size on the feeder. 
    - TCL_pop_sim_3state
        - called within calculateInitialConditions and onSync_base to
            update TCL parameters for the current time-step and advance
            the internal states of the TCLs based on the parameters and 
            exogenous variables (e.g., outdoor temperature)
    - updatePowerDraw
        - Called within TCL_pop_sim_3state and within onSync_base to update
            the real and reactive power draw of each TCL based on the
            outdoor temperature and the voltage that the TCL sees. It also
            incorporates in-rush power draw if the unit has recently
            switched on. The steady-state power draw is non-zero even if 
            the device is not drawing power, but we multiply by the on/off
            mode within the simulation to remove power for off units. 
    - updateQh
        - called within TCL_pop_sim_3state to update the heat transfer 
            between the air conditioner and the inside of the house based 
            on the outdoor temperature. 
    - createBinModel
	- creates the binModel struct by simulating the TCL population for a day
	    in order to capture all transition probabilities. Gets called from
  	    main_simulateWithoutGLD.
    - update_binModel_states
	- updates the field x of binModel stuct with the percentages of the
	    population within each bin. Gets call at each iteration during onSync
    - generatePlots
	- produces basic plots that give an idea of the system's response


- How to run a simulation:
    - Before running a simulation, run main_constructFeeders to create the
      feeders from PNNL's taxonomy list and the house and TCL models
      developed for this project. This will create folders with the feeder
      name and an arpaE suffix (e.g., "R5_1247_1_arpaE") where the suffix
      is used to indicate the technology build used to populate the feeder.
      The PNNL feeder populator provides a suffix when running its code,
      and we have followed their method while providing our own suffix.
    - To run a simulation without a GridLAB-D link (and so without
      receiving real-time voltages from a grid model), run 
      main_simulateWithoutGLD with the variable tax_file set to the feeder
      you want to simulate. The feeder must have been previously 
      constructed using the prior step, and the simulation will run with 
      the options used in feeder construction script. This script is
      intended to provide the same information (i.e., set the same
      variables) that GridLAB-D does such as the time within the simulation
      and the relevant nodal voltages. 
    - To run a simulation with GridLAB-D, there must be a GridLAB-D build 
      that enables a GridLAB-D simulation to link with with MATLAB and run
      MATLAB code while passing variables back and forth. If this exists, 
      open a command prompt (in Windows), set the current directory to the
      feeder that should be simulated. For example, execute "cd 
      pathToCodebase\arpae_codebase\feederOfInterest" in the commant prompt
      where pathToCodebase is the path to the directory containing the code
      base, and where feederOfInterest is the feeder to be simulated. Then,
      execute the simulation in the command prompt using "gridlabd 
      feederOfInterest.glm", again where feederOfInterest is replaced by
      the feeder whose directory we have moded the current directory to. 