
if(WIN32)
    option(BUILD_all_in_one_installer "Build an all-in-one NSIS installer" OFF)
endif(WIN32)

if(BUILD_all_in_one_installer)
    get_filename_component(BOOST_ROOT "${Boost_INCLUDE_DIR}" PATH)
    get_filename_component(EIGEN_ROOT "${EIGEN_INCLUDE_DIRS}" PATH)
    get_filename_component(QHULL_ROOT "${QHULL_INCLUDE_DIRS}" PATH)
    get_filename_component(FLANN_ROOT "${FLANN_INCLUDE_DIRS}" PATH)
    get_filename_component(VTK_ROOT "${VTK_DIR}" PATH)
    get_filename_component(VTK_ROOT "${VTK_ROOT}" PATH)
    set(PCL_3RDPARTY_COMPONENTS)
    foreach(dep Eigen Boost Qhull FLANN VTK)
        string(TOUPPER ${dep} DEP)
        install(
            DIRECTORY "${${DEP}_ROOT}/"
            DESTINATION 3rdParty/${dep}
            COMPONENT ${dep}
            PATTERN "*/Uninstall.exe" EXCLUDE
        )
        list(APPEND PCL_3RDPARTY_COMPONENTS ${dep})
    endforeach(dep)

	if($BUILD_OPENNI)
		if(CMAKE_CL_64)
			set(OPENNI_PACKAGE "OpenNI-Win64-1.5.4-Dev.msi")
			set(OPENNI_URL "http://www.pointclouds.org/assets/files/dependencies/${OPENNI_PACKAGE}")
			set(OPENNI_MD5 c8f9cbe8447a16d32572a4e2c2d00af0)
			set(OPENNI_SENSOR_PACKAGE "Sensor-Win-OpenSource64-5.1.0.msi")
			set(OPENNI_SENSOR_URL "http://www.pointclouds.org/assets/files/dependencies/${OPENNI_SENSOR_PACKAGE}")
			set(OPENNI_SENSOR_MD5 badb880116436870943b1b7c447dfa22)
		else(CMAKE_CL_64)
			set(OPENNI_PACKAGE "OpenNI-Win32-1.5.4-Dev.msi")
			set(OPENNI_URL "http://www.pointclouds.org/assets/files/dependencies/${OPENNI_PACKAGE}")
			set(OPENNI_MD5 0b7118a0581abef411b58530d4039cf0)
			set(OPENNI_SENSOR_PACKAGE "Sensor-Win-OpenSource32-5.1.0.msi")
			set(OPENNI_SENSOR_URL "http://www.pointclouds.org/assets/files/dependencies/${OPENNI_SENSOR_PACKAGE}")
			set(OPENNI_SENSOR_MD5 8bf14b2e813859f868fc316acb2d08fa)	
		endif(CMAKE_CL_64)
	else()	# Need OpenNI 2.x
		if(CMAKE_CL_64)
			set(OPENNI_PACKAGE "OpenNI-Windows-x64-2.2.msi")
			set(OPENNI_URL "$ENV{OPENNI2_INSTALLERS}/${OPENNI_PACKAGE}")
			set(OPENNI_MD5 d41d8cd98f00b204e9800998ecf8427e)
		else(CMAKE_CL_64)
			set(OPENNI_PACKAGE "OpenNI-Windows-x86-2.2.msi")
			set(OPENNI_URL "$ENV{OPENNI2_INSTALLERS}/${OPENNI_PACKAGE}")
			set(OPENNI_MD5 e1949f040cfb52ed7dfccf9920e5d0dd)
		endif(CMAKE_CL_64)
	endif()

    set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS "  IntCmp $OpenNI_selected 0 noinstall_openni_packages\n")

    file(DOWNLOAD ${OPENNI_URL} "${CMAKE_CURRENT_BINARY_DIR}/${OPENNI_PACKAGE}" 
        STATUS _openni_download_status LOG _openni_download_log
        EXPECTED_MD5 ${OPENNI_MD5}
       )
    list(GET _openni_download_status 0 _error_code)
    list(GET _openni_download_status 1 _error_message)
    if(_error_code EQUAL 0)
        install(
            FILES "${CMAKE_CURRENT_BINARY_DIR}/${OPENNI_PACKAGE}" 
            DESTINATION 3rdParty/OpenNI
            COMPONENT OpenNI
        )
        list(APPEND PCL_3RDPARTY_COMPONENTS OpenNI)
        set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS 
            "${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}\n    ExecWait 'msiexec /i \\\"$INSTDIR\\\\3rdParty\\\\OpenNI\\\\${OPENNI_PACKAGE}\\\" '")
    else(_error_code EQUAL 0)
        message("WARNING : Could not download ${OPENNI_URL}, error code : ${_error_code}, error message : ${_error_message}")
    endif(_error_code EQUAL 0)

	if($BUILD_OPENNI) # Only need sensor package for OpenNI 1.x, not OpenNI 2.x
		file(DOWNLOAD ${OPENNI_SENSOR_URL} "${CMAKE_CURRENT_BINARY_DIR}/${OPENNI_SENSOR_PACKAGE}" 
			STATUS _openni_download_status LOG _openni_download_log
			EXPECTED_MD5 ${OPENNI_SENSOR_MD5}
		   )
		list(GET _openni_download_status 0 _error_code)
		list(GET _openni_download_status 1 _error_message)
		if(_error_code EQUAL 0)
			install(
				FILES "${CMAKE_CURRENT_BINARY_DIR}/${OPENNI_SENSOR_PACKAGE}"
				DESTINATION 3rdParty/OpenNI
				COMPONENT OpenNI
			)
			list(APPEND PCL_3RDPARTY_COMPONENTS OpenNI)
			set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS 
				"${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}\n    ExecWait 'msiexec /i \\\"$INSTDIR\\\\3rdParty\\\\OpenNI\\\\${OPENNI_SENSOR_PACKAGE}\\\" '")
		else(_error_code EQUAL 0)
			message("WARNING : Could not download ${OPENNI_SENSOR_URL}, error code : ${_error_code}, error message : ${_error_message}")
		endif(_error_code EQUAL 0)
	endif($BUILD_OPENNI)
	
	
    list(REMOVE_DUPLICATES PCL_3RDPARTY_COMPONENTS)
    set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS "${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}\n  noinstall_openni_packages:\n")
endif(BUILD_all_in_one_installer)
