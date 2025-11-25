# SPDX-License-Identifier: Apache-2.0


if (CONFIG_SOF AND CONFIG_BOARD_IMX8MP_EVK_MIMX8ML8_M7_DDR)
  add_custom_target(zephyr.ri ALL
    DEPENDS ${CMAKE_BINARY_DIR}/zephyr/zephyr.ri
  )

  add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/zephyr/zephyr.ri
    COMMAND west sign --if-tool-available --tool rimage --build-dir ${CMAKE_BINARY_DIR} ${WEST_SIGN_OPTS}
    DEPENDS ${CMAKE_BINARY_DIR}/zephyr/${KERNEL_ELF_NAME}
  )
endif()

