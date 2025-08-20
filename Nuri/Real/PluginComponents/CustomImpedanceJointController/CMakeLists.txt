set(IMPEDANCEJCTRLCOMP_NAME CustomImpedanceJointControllerCreator)

#############################################################
# Properties                                                #
#############################################################

#############################################################
# Sources                                                   #
#############################################################
file(GLOB_RECURSE IMPEDANCEJCTRLCOMP_SRC "*.cpp")

#############################################################
# Shared Library (Plugin)                                   #
#############################################################
add_library(${IMPEDANCEJCTRLCOMP_NAME} MODULE
			${IMPEDANCEJCTRLCOMP_SRC}
)

#############################################################
# Libraries linking                                         #
#############################################################
target_link_libraries(${IMPEDANCEJCTRLCOMP_NAME} ${LIBS})

set_target_properties(${IMPEDANCEJCTRLCOMP_NAME} PROPERTIES SUFFIX ".comp")
set_target_properties(${IMPEDANCEJCTRLCOMP_NAME} PROPERTIES PREFIX "")

install (TARGETS ${IMPEDANCEJCTRLCOMP_NAME} DESTINATION ./PluginComponents)