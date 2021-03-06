SET(${PROJECT_NAME}_BENCHMARK
  boxqp
  unicycle-optctrl
  lqr-optctrl
  arm-manipulation-optctrl
  arm-manipulation-timings
  quadrupedal-gaits-optctrl
  bipedal-timings
  )


IF(BUILD_WITH_CODEGEN_SUPPORT)
  SET(${PROJECT_NAME}_CODEGEN_BENCHMARK
    arm-manipulation-codegen
    bipedal-with-contact-codegen
    )
  LIST(APPEND ${PROJECT_NAME}_BENCHMARK ${${PROJECT_NAME}_CODEGEN_BENCHMARK})
ENDIF()


FOREACH(BENCHMARK_NAME ${${PROJECT_NAME}_BENCHMARK})
  ADD_EXECUTABLE(${BENCHMARK_NAME} ${BENCHMARK_NAME}.cpp)
  TARGET_LINK_LIBRARIES(${BENCHMARK_NAME} ${PROJECT_NAME} example-robot-data::example-robot-data)
  ADD_CUSTOM_TARGET("benchmarks-cpp-${BENCHMARK_NAME}"
    ${BENCHMARK_NAME} \${INPUT})
ENDFOREACH(BENCHMARK_NAME ${${PROJECT_NAME}_BENCHMARK})

IF(BUILD_WITH_CODEGEN_SUPPORT)
  FOREACH(BENCHMARK_NAME ${${PROJECT_NAME}_CODEGEN_BENCHMARK})
    TARGET_LINK_LIBRARIES(${BENCHMARK_NAME} ${CMAKE_DL_LIBS})
  ENDFOREACH(BENCHMARK_NAME ${${PROJECT_NAME}_CODEGEN_BENCHMARK})
ENDIF()


IF(BUILD_PYTHON_INTERFACE)
  SET(${PROJECT_NAME}_BENCHMARK_PYTHON
    unicycle_optctrl
    lqr_optctrl
    arm_manipulation_optctrl
    bipedal_walk_optctrl
    quadrupedal_gaits_optctrl
    )
  
  FOREACH(BENCHMARK_NAME ${${PROJECT_NAME}_BENCHMARK_PYTHON})
    PYTHON_BUILD(. "${BENCHMARK_NAME}.py")
    ADD_CUSTOM_TARGET("benchmarks-py-${BENCHMARK_NAME}"
      ${CMAKE_COMMAND} -E env PYTHONPATH=${PROJECT_BINARY_DIR}/bindings/python:$ENV{PYTHONPATH}
      ${PYTHON_EXECUTABLE} -c "from ${BENCHMARK_NAME} import *" \${INPUT})
  ENDFOREACH(BENCHMARK_NAME ${${PROJECT_NAME}_BENCHMARK_PYTHON})
ENDIF(BUILD_PYTHON_INTERFACE)
