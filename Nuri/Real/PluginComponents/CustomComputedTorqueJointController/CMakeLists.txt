set(CTJCTRLCOMP_NAME CustomComputedTorqueJointControllerCreator)

#############################################################
# Properties                                                #
#############################################################

#############################################################
# Sources                                                   #
#############################################################
file(GLOB_RECURSE CTJCTRLCOMP_SRC "*.cpp")

#############################################################
# Shared Library (Plugin)                                   #
#############################################################
add_library(${CTJCTRLCOMP_NAME} MODULE
			${CTJCTRLCOMP_SRC}
)

#############################################################
# Libraries linking                                         #
#############################################################
target_link_libraries(${CTJCTRLCOMP_NAME} ${LIBS})

set_target_properties(${CTJCTRLCOMP_NAME} PROPERTIES SUFFIX ".comp")
set_target_properties(${CTJCTRLCOMP_NAME} PROPERTIES PREFIX "")

install (TARGETS ${CTJCTRLCOMP_NAME} DESTINATION ./PluginComponents)