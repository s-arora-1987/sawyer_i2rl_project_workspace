cmake_minimum_required(VERSION 2.8.3)

project(irld C D)

SET( GLOBAL_GDC_DEFS "-frelease -march=core2 -m64" )
#SET( GLOBAL_GDC_DEFS "-O3 -fno-bounds-check -frelease -fomit-frame-pointer -march=core2 -mfpmath=sse -msse2 -msse3 -mssse3" )
SET( GLOBAL_GCC_DEFS "-O3 -fomit-frame-pointer -march=core2 -mfpmath=sse -msse2 -msse3 -mssse3" )
SET( GLOBAL_GDC_DEBUG_DEFS "-g -fdebug -fdebug=lbfgs" )
#SET( GLOBAL_GDC_DEBUG_DEFS "-g -fdebug" )
#SET( GLOBAL_GDC_DEBUG_DEFS "-g -fdebug=printWeights" )
#SET( GLOBAL_GDC_DEBUG_DEFS "-g -fdebug=WeightsLME_EM" )
#SET( GLOBAL_GDC_DEBUG_DEFS "-g -fdebug=WeightsmIRLstar" )

SET( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${GLOBAL_GCC_DEFS}" )
#SET( CMAKE_D_FLAGS "${CMAKE_D_FLAGS} ${GLOBAL_GDC_DEFS}" )
#SET( CMAKE_D_FLAGS "${CMAKE_D_FLAGS} ${GLOBAL_GDC_DEFS} ${GLOBAL_GDC_DEBUG_QWEIGHTSLMEIRL_DEFS}" )
SET( CMAKE_D_FLAGS "${CMAKE_D_FLAGS} ${GLOBAL_GDC_DEFS} ${GLOBAL_GDC_DEBUG_DEFS}" )

find_package(catkin REQUIRED)

include(ExternalProject)
ExternalProject_Add(liblbfgs
	URL ${PROJECT_SOURCE_DIR}/liblbfgs-master.zip
#	GIT_REPOSITORY https://github.com/chokkan/liblbfgs.git
#	GIT_TAG v1.10
	PREFIX ${CMAKE_BINARY_DIR}/liblbfgs
#	CONFIGURE_COMMAND ./autogen.sh && ./configure --enable-sse2 --prefix=${CATKIN_DEVEL_PREFIX}
	CONFIGURE_COMMAND ./autogen.sh && ./configure  --prefix=${CATKIN_DEVEL_PREFIX}
#	CONFIGURE_COMMAND echo "None"
	BUILD_COMMAND make
	INSTALL_COMMAND make install 
	BUILD_IN_SOURCE 1
)

include_directories(src)

add_library(asa047
   src/asa047.c
)

link_directories(${CATKIN_DEVEL_PREFIX}/lib)

#add_executable(irljointmdp src/mdp.d src/boydmdp.d src/jointboydmdp.d src/irl.d src/Boyd2FullMdp.d)
#target_link_libraries(irljointmdp
#   asa047 lbfgs
#)
#add_dependencies(irljointmdp liblbfgs)

add_executable(boydpatroller src/mdp.d src/boydmdp.d src/BoydSolvePMdp.d)
add_executable(boydattacker src/mdp.d src/boydmdp.d src/attacker.d src/BoydSolveAMdp.d)

add_executable(solveboydpatroller src/mdp.d src/boydmdp.d src/solvePatrolMdp.d)


add_executable(boydirl src/mdp.d src/boydmdp.d src/irl.d src/BoydIRL.d src/irlgraveyard.d src/sortingMDP.d)
target_link_libraries(boydirl
   asa047 lbfgs
)
add_dependencies(boydirl liblbfgs)

add_executable(meirl src/mdp.d src/boydmdp.d src/irlWithoutLBFGS.d src/singleTaskIRL.d src/sortingMDP.d)
#target_link_libraries(meirl
#   asa047 lbfgs
#)
#add_dependencies(meirl liblbfgs)

add_executable(noisyObsRobustSamplingMeirl src/mdp.d src/boydmdp.d src/irlWithoutLBFGS.d src/singleTaskIRLNoisyObs.d src/sortingMDP.d)

#add_executable(testdeb src/testdebug.d)

#add_executable(boydemirl src/mdp.d src/boydmdp.d src/irl.d src/BoydEMIRL.d src/irlgraveyard.d)
#target_link_libraries(boydemirl
#   asa047 lbfgs
#)
#add_dependencies(boydemirl liblbfgs)

#add_executable(toytest src/mdp.d src/toymdpmirl.d src/toytest.d src/irl.d src/irlgraveyard.d src/uai16.d)
#target_link_libraries(toytest
#   asa047 lbfgs
#)
#add_dependencies(toytest liblbfgs)

add_executable(boydile src/ILE.d src/mdp.d src/boydmdp.d)

#add_executable(boyd_t src/mdp.d src/BoydSolveT.d src/boydmdp.d src/irl.d)
#target_link_libraries(boyd_t
#   asa047 lbfgs
#)
#add_dependencies(boyd_t liblbfgs)

#add_executable(waterloorun src/mdp.d src/waterloomdp.d src/waterloorun.d src/irl.d)
#target_link_libraries(waterloorun
#   asa047 lbfgs
#)
#add_dependencies(waterloorun liblbfgs)

#add_executable(boydlmeirl src/mdp.d src/boydmdp.d src/irl.d src/BoydLMEIRL.d)
#target_link_libraries(boydlmeirl
#   asa047 lbfgs
#)
#add_dependencies(boydlmeirl liblbfgs)

add_executable(boydsimple_t src/mdp.d src/boydmdp.d src/BoydSimpleT.d) 

#add_executable(boydt src/mdp.d src/boydmdp.d src/BoydT.d) 

#add_executable(gridworld src/mdp.d src/rsmdp.d src/gridworld.d) 

#add_executable(boyd_em_t src/mdp.d src/BoydSolveTEM.d src/boydmdp.d) 

add_executable(solveSortingMDP src/sortingMDP.d src/mdp.d src/solveSortingMDP.d)

add_executable(dpmMEIRL src/multitaskIRL.d src/sortingMDP.d src/mdp.d src/irl.d src/boydmdp.d)
#target_link_libraries(dpmMEIRL
#   asa047 lbfgs
#)
#add_dependencies(dpmMEIRL liblbfgs)

add_executable(sortingMDPILE src/ileSortingMDP.d src/mdp.d src/sortingMDP.d) 

add_executable(code_test src/code_test.d) 

# set_target_properties(boydpatroller boydattacker boydirl boyd_t boydsimple_t gridworld boyd_em_t boydt toytest boydemirl waterloorun boydlmeirl sortingMDP PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/bin) 
# set_target_properties(boydpatroller boydattacker boydirl boydsimple_t boydile PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/bin) 
# set_target_properties(boydpatroller boydattacker boydsimple_t boydile sortingMDP PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/bin) 
#set_target_properties(boydirl boydpatroller boydattacker dpmMEIRL boydsimple_t boydile solveSortingMDP sortingMDPILE solveboydpatroller PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/bin) 
set_target_properties(boydirl meirl noisyObsRobustSamplingMeirl boydpatroller boydattacker dpmMEIRL boydsimple_t boydile solveSortingMDP sortingMDPILE solveboydpatroller code_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/bin) 
