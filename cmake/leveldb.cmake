include(FetchContent)
FetchContent_Declare(
    leveldb
    GIT_REPOSITORY 	https://github.com/helix-re/leveldb.git
	GIT_TAG 		helix-master
)

FetchContent_GetProperties(leveldb)
if(NOT leveldb_POPULATED)
    FetchContent_Populate(leveldb)
    add_subdirectory(${leveldb_SOURCE_DIR} ${leveldb_BINARY_DIR})
    set(leveldb_INCLUDE "${leveldb_SOURCE_DIR}/include")
    set_property(TARGET leveldb PROPERTY FOLDER "Third/leveldb")
endif()