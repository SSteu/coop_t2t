### --- INFORMATION REGARDING THE coop_t2t ROS PACKAGE --- ###

# General information:
	This package contains all relevant files for the T2T association
	and fusion algorithms.
	It also includes three executable files, that show how the data
	sets can be loaded and analyzed and how the simulation setup
	can be used.
	The entire implementation is in python.

# Executable files:
	historic_assoc.py:
		Parameter 1: OPTIONAL : filename of the rosbag in the data/ folder that should be played
		Parameter 2: OPTIONAL : start time in seconds
		Parameter 3: OPTIONAL : Play rate of ros bags (ex: 0.5 to halve playing speed)
	manual_RA_match.py:
		Parameter 1: OPTIONAL : Start time in seconds
        Parameter 2: OPTIONAL : Play rate of ros bags (ex: 0.5 to halve playing speed)
	sim_RMSE.py:
		Parameter 1: OPTIONAL : base standard deviation used for the measurements
		Parameter 2: OPTIONAL : value that is added to the standard deviation for every subsequent sensor

# Workspace setup:
	Source files are in the ./src folder. Main files are in ./src, other 
	files that provide helper functions are split in two subfolders: 
	./src/general and ./src/simulation.

	More information regarding these is provided below.
	The ./data folder DOES NOT contain the real data sets.
	These should be in the workspace/data folder, so that the workspace setup is similar to this:
		src/
			[... other packages]
			coop_t2t
				src/
					[... source files]
				data/
					[pickle files, output, etc]
		data/
			[real data sets]
		[... more folders (devel, ...]
	Please also refer to the ./data/README_data.txt file.

# RVIZ Setup:
	To visualize the results in rviz some configuration is necessary:
		Set the global fixed frame to "ibeo_front_center"

	The following topics need to be subscribed to:
		/spherical_grid : PointCloud2
			Size(m) should be 0.1 to 0.2
		/assoc_markers : MarkerArray
			queue size = 100
	The assoc_markers topic includes ALL markers that are relevant for the respective step, i.e. also
	the tracking visualization.	

# Short description of every file:
		- historic_assoc.py
			Main executable file for the first two data sets.
		- manual_RA_match.py
			Main executable file for the third data set. Uses both
			bag files and plays them in sync to simulate CPMs.				
		- sim_RMSE.py
			Executable file that plays a simulation and prints 
			RMSE values for the measurements and the fusion results.
		- tracking_visuals.py
			Contains functions used for visualizing program output.
		- t2t_history.py				
			Contains functions that implement the tracking history
			used to compare tracks w.r.t. distance.
		- t2ta_algorithms.py			
			Contains the core algorithms used for association of 
			objects with each other.
		- t2tf_algorithms.py		
			Contains the core algorithms used for fusion of
			association results.	
		- similarity.py
			Contains simple distance comparison functions that are 
			based on the history of the tracks.
		- sim_coordinator.py			
			Contains the main class that is used to manage vehciles
			in a simulation scenario.
		- sim_classes.py
			Contains the class used to simulate a single vehicle.
		- visual_publishing.py
			Includes functions for publishing MarkerArrays to rviz
