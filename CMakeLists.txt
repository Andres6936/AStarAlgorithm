CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

PROJECT(AStarCPP)

# Executable Bucharest
SET(MIN_PATH_BUCHAREST cpp/min_path_to_Bucharest.cpp)
ADD_EXECUTABLE(Bucharest ${MIN_PATH_BUCHAREST})

# Executable Puzzle
SET(PUZZLE_SOURCE cpp/8puzzle.cpp)
ADD_EXECUTABLE(Puzzle ${PUZZLE_SOURCE})

# Exucutable FindPath
SET(FIND_PATH_SOURCE cpp/findpath.cpp)
ADD_EXECUTABLE(FindPath ${FIND_PATH_SOURCE})
