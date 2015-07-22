file(
	GLOB
	CLP_SRC_HEADER
	*.hpp
	*.h
)
file(
	GLOB
	CLP_SRC_CPP
	*.cpp
)


file(
     GLOB
     REMOVE_CPP
     ClpCholeskyMumps.cpp
     ClpCholeskyUfl.cpp
     ClpCholeskyWssmp.cpp
     ClpCholeskyWssmpKKT.cpp
     *Abc*.cpp
)   

file(
     GLOB
    REMOVE_HEADER
     ClpCholeskyMumps.hpp
     ClpCholeskyUfl.hpp
     ClpCholeskyWssmp.hpp
     ClpCholeskyWssmpKKT.hpp 
     *Abc*.hpp
     *Abc*.h
)   

list(REMOVE_ITEM CLP_SRC_HEADER ${REMOVE_HEADER})
list(REMOVE_ITEM CLP_SRC_CPP ${REMOVE_CPP})

ADD_LIBRARY(lib_clp ${CLP_SRC_HEADER} ${CLP_SRC_CPP})
TARGET_LINK_LIBRARIES(lib_clp lib_CoinUtils )

add_subdirectory(OsiClp)

if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(lib_clp PROPERTIES FOLDER "3rd party")
else(ENABLE_SOLUTION_FOLDERS)
	SET_TARGET_PROPERTIES(lib_clp  PROPERTIES PROJECT_LABEL "(TEST) lib_clp")
endif(ENABLE_SOLUTION_FOLDERS)

