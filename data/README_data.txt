### --- INFORMATION REGARDING FILES IN THE DATA FOLDER --- ###

This folder does not contain actual data sets.

Only output and secondary input is stored here.

For example:
		tf_static_dump.pkl
		tf_static_dump_fascare.pkl
		tf_static_dump_viewcar2.pkl
	These three files contain tf-message dumps. The messages stored are
	static tf messages that are only sent once at the start of the bags.
	So, if the bag should be started at a later time, the necessary tf
	information is instead imported from these files.
