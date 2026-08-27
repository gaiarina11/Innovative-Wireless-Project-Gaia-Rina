#
# Copyright (c) 2024 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(channel_sounding_ras_initiator)

set(ANCHOR_ID "0" CACHE STRING "Physical anchor identifier (0, 1 or 2)")
# Resolve a command-line value from both regular Zephyr builds and sysbuild.
zephyr_get(ANCHOR_ID)
if(NOT ANCHOR_ID MATCHES "^[012]$")
	message(FATAL_ERROR "ANCHOR_ID must be 0, 1 or 2")
endif()

FILE(GLOB app_sources src/*.c)
# NORDIC SDK APP START
target_sources(app PRIVATE
	${app_sources}
)
target_compile_definitions(app PRIVATE ANCHOR_ID=${ANCHOR_ID})
# NORDIC SDK APP END
