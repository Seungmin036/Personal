set(ADMITTANCEJCTRLCOMP_NAME CustomAdmittanceJointControllerCreator)

#############################################################
# Properties                                                #
#############################################################

#############################################################
# Sources                                                   #
#############################################################
file(GLOB_RECURSE ADMITTANCEJCTRLCOMP_SRC "*.cpp")

#############################################################
# Shared Library (Plugin)                                   #
#############################################################
add_library(${ADMITTANCEJCTRLCOMP_NAME} MODULE
			${ADMITTANCEJCTRLCOMP_SRC}
)

#############################################################
# Libraries linking                                         #
#############################################################
target_link_libraries(${ADMITTANCEJCTRLCOMP_NAME} ${LIBS})

set_target_properties(${ADMITTANCEJCTRLCOMP_NAME} PROPERTIES SUFFIX ".comp")
set_target_properties(${ADMITTANCEJCTRLCOMP_NAME} PROPERTIES PREFIX "")

install (TARGETS ${ADMITTANCEJCTRLCOMP_NAME} DESTINATION ./PluginComponents)