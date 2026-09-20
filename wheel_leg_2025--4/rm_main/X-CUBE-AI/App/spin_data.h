/**
  ******************************************************************************
  * @file    spin_data.h
  * @author  AST Embedded Analytics Research Platform
  * @date    2026-09-20T09:51:34+0800
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#ifndef SPIN_DATA_H
#define SPIN_DATA_H

#include "spin_config.h"
#include "spin_data_params.h"

AI_DEPRECATED
#define AI_SPIN_DATA_ACTIVATIONS(ptr_)  \
  ai_spin_data_activations_buffer_get(AI_HANDLE_PTR(ptr_))

AI_DEPRECATED
#define AI_SPIN_DATA_WEIGHTS(ptr_)  \
  ai_spin_data_weights_buffer_get(AI_HANDLE_PTR(ptr_))


AI_API_DECLARE_BEGIN


extern const ai_u64 s_spin_weights_array_u64[19413];



/*!
 * @brief Get network activations buffer initialized struct.
 * @ingroup spin_data
 * @param[in] ptr a pointer to the activations array storage area
 * @return an ai_buffer initialized struct
 */
AI_DEPRECATED
AI_API_ENTRY
ai_buffer ai_spin_data_activations_buffer_get(const ai_handle ptr);

/*!
 * @brief Get network weights buffer initialized struct.
 * @ingroup spin_data
 * @param[in] ptr a pointer to the weights array storage area
 * @return an ai_buffer initialized struct
 */
AI_DEPRECATED
AI_API_ENTRY
ai_buffer ai_spin_data_weights_buffer_get(const ai_handle ptr);

/*!
 * @brief Get network weights array pointer as a handle ptr.
 * @ingroup spin_data
 * @return a ai_handle pointer to the weights array
 */
AI_DEPRECATED
AI_API_ENTRY
ai_handle ai_spin_data_weights_get(void);


/*!
 * @brief Get network params configuration data structure.
 * @ingroup spin_data
 * @return true if a valid configuration is present, false otherwise
 */
AI_API_ENTRY
ai_bool ai_spin_data_params_get(ai_network_params* params);


AI_API_DECLARE_END

#endif /* SPIN_DATA_H */

