idf_component_register(SRCS "gnss.c" "minmea.c"
	INCLUDE_DIRS "include" "../car/include" 
	)
set_source_files_properties(minmea.c PROPERTIES COMPILE_OPTIONS "-Dtimegm=mktime")
