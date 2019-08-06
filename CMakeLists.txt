CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

PROJECT(AStarCPP)

# Add the directory of includes
INCLUDE_DIRECTORIES(Include)

# Executable Bucharest
SET(MIN_PATH_BUCHAREST Source/MinPathBucharest.cpp)
ADD_EXECUTABLE(Bucharest ${MIN_PATH_BUCHAREST})

# Executable Puzzle
SET(PUZZLE_SOURCE Source/PuzzleEight.cpp)
ADD_EXECUTABLE(Puzzle ${PUZZLE_SOURCE})

# Exucutable FindPath
SET(FIND_PATH_SOURCE Source/FindPath.cpp)
ADD_EXECUTABLE(FindPath ${FIND_PATH_SOURCE})
