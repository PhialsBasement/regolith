cmake_minimum_required(VERSION 3.20)
project(regolith LANGUAGES CXX)

option(REGOLITH_BUILD_GDEXTENSION "Build the Godot extension" ON)
option(REGOLITH_BUILD_TESTS "Build the core unit tests" ON)
set(REGOLITH_ADDON_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../addons/regolith"
	CACHE PATH "Where the extension and its .gdextension land")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Same bits on every platform: no FMA contraction, no fast-math.
if(MSVC)
	set(REGOLITH_FP_FLAGS /fp:precise)
else()
	set(REGOLITH_FP_FLAGS -ffp-contract=off -fno-fast-math)
endif()

# zstd: the system's, or for a cross build (no target zstd on the
# host) compiled in from a zstd source tree (its lib/ directory)
set(REGOLITH_ZSTD_SOURCE "" CACHE PATH "zstd lib sources to compile in")
if(REGOLITH_ZSTD_SOURCE)
	enable_language(C)
	file(GLOB REGOLITH_ZSTD_SRC
		"${REGOLITH_ZSTD_SOURCE}/common/*.c"
		"${REGOLITH_ZSTD_SOURCE}/compress/*.c"
		"${REGOLITH_ZSTD_SOURCE}/decompress/*.c")
	add_library(regolith_zstd STATIC ${REGOLITH_ZSTD_SRC})
	target_include_directories(regolith_zstd PUBLIC "${REGOLITH_ZSTD_SOURCE}")
	target_compile_definitions(regolith_zstd PRIVATE ZSTD_DISABLE_ASM)
	set_target_properties(regolith_zstd PROPERTIES POSITION_INDEPENDENT_CODE ON)
	set(REGOLITH_ZSTD regolith_zstd)
else()
	find_package(PkgConfig REQUIRED)
	pkg_check_modules(ZSTD REQUIRED IMPORTED_TARGET libzstd)
	set(REGOLITH_ZSTD PkgConfig::ZSTD)
endif()

add_library(regolith_core STATIC
	core/src/codec.cpp
	core/src/cube.cpp
	core/src/climate.cpp
	core/src/air.cpp
	core/src/body_presets.cpp
	core/src/ephemeris.cpp
	core/src/height_tile.cpp
	core/src/horizon.cpp
	core/src/kepler.cpp
	core/src/orbit.cpp
	core/src/pack.cpp
	core/src/patch.cpp
	core/src/select.cpp
	core/src/synth.cpp
	core/src/tile_loader.cpp
)
target_include_directories(regolith_core PRIVATE thirdparty/fastnoiselite)
find_package(Threads REQUIRED)
target_link_libraries(regolith_core PUBLIC Threads::Threads)
target_include_directories(regolith_core PUBLIC core/include)
target_link_libraries(regolith_core PUBLIC ${REGOLITH_ZSTD})
target_compile_options(regolith_core PRIVATE ${REGOLITH_FP_FLAGS})

option(REGOLITH_BUILD_BAKER "Build the regolith_bake tool" ON)
if(REGOLITH_BUILD_BAKER)
	add_executable(regolith_bake
		bake/areoid.cpp
		bake/catalog.cpp
		bake/main.cpp
		bake/raster_dem.cpp
		bake/phobos_icq.cpp
		bake/samplers.cpp
		bake/tile_bake.cpp
	)
	target_link_libraries(regolith_bake PRIVATE regolith_core)
	target_compile_options(regolith_bake PRIVATE ${REGOLITH_FP_FLAGS})
endif()

if(REGOLITH_BUILD_TESTS)
	enable_testing()
	add_executable(regolith_core_tests
		tests/test_main.cpp
		tests/test_codec.cpp
		tests/test_cube.cpp
		tests/test_air.cpp
		tests/test_ephemeris.cpp
		tests/test_fold.cpp
		tests/test_horizon.cpp
		tests/test_kepler.cpp
		tests/test_patch.cpp
		tests/test_select.cpp
		tests/test_synth.cpp
		tests/test_tiles.cpp
	)
	target_link_libraries(regolith_core_tests PRIVATE regolith_core)
	target_compile_options(regolith_core_tests PRIVATE ${REGOLITH_FP_FLAGS})
	add_test(NAME core COMMAND regolith_core_tests)
endif()

if(REGOLITH_BUILD_GDEXTENSION)
	set(GODOTCPP_DEFAULT_API_VERSION "4.7")
	# A double build needs the API dumped from a double Godot.
	if(GODOTCPP_PRECISION STREQUAL "double")
		set(GODOTCPP_CUSTOM_API_FILE "${CMAKE_CURRENT_SOURCE_DIR}/api/extension_api-4-7-double.json"
			CACHE FILEPATH "" FORCE)
	endif()
	add_subdirectory(thirdparty/godot-cpp)
	add_library(regolith SHARED
		gdext/register_types.cpp
		gdext/regolith_atmosphere.cpp
		gdext/regolith_body.cpp
		gdext/regolith_collider.cpp
		gdext/regolith_system.cpp
		gdext/regolith_vessel.cpp
		gdext/regolith_weather.cpp
		gdext/rd_util.cpp
		gdext/regolith_clouds.cpp
		gdext/regolith_editor.cpp
		gdext/regolith_engine.cpp
	)
	target_link_libraries(regolith PRIVATE godot::cpp regolith_core)
	target_compile_options(regolith PRIVATE ${REGOLITH_FP_FLAGS})
	get_target_property(GODOTCPP_SUFFIX godot-cpp GODOTCPP_SUFFIX)
	set_target_properties(regolith PROPERTIES
		CXX_VISIBILITY_PRESET hidden
		PREFIX "lib"
		OUTPUT_NAME "regolith${GODOTCPP_SUFFIX}"
		LIBRARY_OUTPUT_DIRECTORY "$<1:${REGOLITH_ADDON_DIR}/bin>"
		RUNTIME_OUTPUT_DIRECTORY "$<1:${REGOLITH_ADDON_DIR}/bin>"
	)
	configure_file(addon/regolith.gdextension
		"${REGOLITH_ADDON_DIR}/regolith.gdextension" COPYONLY)
	configure_file(addon/regolith_terrain.gdshader
		"${REGOLITH_ADDON_DIR}/regolith_terrain.gdshader" COPYONLY)
	foreach(f regolith_clipmap.gdshader regolith_cloud_common.glsli
			regolith_cloud_noise.glsl regolith_cloud_light.glsl regolith_cloud_march.glsl regolith_cloud_composite.glsl
			regolith_cloud_mip.glsl regolith_cloud_resolve.glsl
			regolith_air_common.glsli regolith_air_transmittance.glsl regolith_air_multiscatter.glsl
			regolith_air_skyview.glsl regolith_air_aerial.glsl regolith_air_composite.glsl
			regolith_air_ambient.glsl regolith_air_irradiance.glsl regolith_sky_objects.glsli regolith_sky_bodies.glsli
			regolith_stars.res regolith_milkyway.res)
		configure_file(addon/${f} "${REGOLITH_ADDON_DIR}/${f}" COPYONLY)
	endforeach()
	# Stock Godot variants of the terrain shaders, without the
	# patched engine's features: slower, but they compile.
	foreach(f regolith_clipmap regolith_terrain)
		set(src "${CMAKE_CURRENT_SOURCE_DIR}/addon/${f}.gdshader")
		set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${src}")
		file(READ "${src}" code)
		string(REGEX REPLACE ",[ ]*motion_vectors_rigid" "" code "${code}")
		string(REGEX REPLACE "profile_clock\\([0-9]+\\)" "0u" code "${code}")
		string(REGEX REPLACE "profile_add\\([0-9]+, [a-z_]+\\);" ";" code "${code}")
		string(REPLACE "IN_DEPTH_PREPASS" "false" code "${code}")
		file(WRITE "${REGOLITH_ADDON_DIR}/${f}_stock.gdshader" "${code}")
	endforeach()
endif()
