/*
 * Copyright (c) 2019, Inria.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \addtogroup msf
 * @{
 */
/**
 * \file
 *         MSF NumCells* counters APIs
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 */

#ifndef _MSF_NUM_CELLS_H_
#define _MSF_NUM_CELLS_H_

#include "msf-negotiated-cell.h"

/**
 * \brief Reset NumCells* counters
 * \param clear_num_cells_required Specify whether you want to reset
 * NumCellsRequired or not
 */
void msf_num_cells_reset(bool clear_num_cells_required);

/**
 * \brief Update NumCells* counters
 * \details This function is expected to be called with an interval
 * of MSF_HOUSEKEEPING_COLLISION_PERIOD_MIN
 */
void msf_num_cells_update(void);

/**
 * \brief Update NumUsed for the negotiated TX cells
 * \param count A value to add to NumUsed for negotiated TX cells
 */
void msf_num_cells_update_tx_used(uint16_t count);

/**
 * \brief Increment NumUsed for negotiated/autonomous RX cells
 */
void msf_num_cells_increment_rx_used(void);

/**
 * \brief Trigger a 6P transaction if necessary, based on NumCells*
 * counters
 */
void msf_num_cells_trigger_6p_transaction(void);

/**
 * \brief Show NumCells* counters in the shell
 * \param output A pointer to shell_output_func
 */
void msf_num_cells_show(shell_output_func output);

#endif /* !_MSF_NUM_CELLS_H_ */
/** @} */
