set(ADMITTANCETCTRLCOMP_NAME CustomAdmittanceTaskControllerCreator)

#############################################################
# Properties                                                #
#############################################################

#############################################################
# Sources                                                   #
#############################################################
file(GLOB_RECURSE ADMITTANCETCTRLCOMP_SRC "*.cpp")

#############################################################
# Shared Library (Plugin)                                   #
#############################################################
add_library(${ADMITTANCETCTRLCOMP_NAME} MODULE
			${ADMITTANCETCTRLCOMP_SRC}
)

#############################################################
# Libraries linking                                         #
#############################################################
target_link_libraries(${ADMITTANCETCTRLCOMP_NAME} ${LIBS})

set_target_properties(${ADMITTANCETCTRLCOMP_NAME} PROPERTIES SUFFIX ".comp")
set_target_properties(${ADMITTANCETCTRLCOMP_NAME} PROPERTIES PREFIX "")

install (TARGETS ${ADMITTANCETCTRLCOMP_NAME} DESTINATION ./PluginComponents)