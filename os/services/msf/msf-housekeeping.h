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
 *         MSF Housekeeping
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 */

#ifndef _MSF_HOUSEKEEPING_H_
#define _MSF_HOUSEKEEPING_H_

#include "net/linkaddr.h"
#include "services/shell/shell.h"

/**
 * \brief Start the housekeeping process
 */
void msf_housekeeping_start(void);

/**
 * \brief Stop the housekeeping process
 */
void msf_housekeeping_stop(void);

/**
 * \brief Set the parent (time-source) address
 * \param new_parent The MAC address of the new parent
 */
void msf_housekeeping_set_parent_addr(const linkaddr_t *new_parent);

/**
 * \brief Return the parent address
 * \return The MAC address of the parent if available, otherwise NULL
 */
const linkaddr_t * msf_housekeeping_get_parent_addr(void);

/**
 * \brief Delete a cell to be relocated
 */
void msf_housekeeping_delete_cell_to_relocate(void);

/**
 * \brief Resolve schedule inconsistency
 * \param peer_addr The MAC address of the peer to which we detecte
 * inconsitency in the schedule
 * \details This call triggers a CLEAR transaction
 */
void msf_housekeeping_resolve_inconsistency(const linkaddr_t *peer_addr);

/**
 * \brief Schedule to delete a cell
 * \param cell A pointer to a negotiated cell to delete
 */
void msf_housekeeping_delete_cell_later(tsch_link_t *cell);

/**
 * \brief Show scheduled negotiated cells in the shell
 * \param output A pointer to shell_output_func
 */
void msf_housekeeping_show_negotiated_cells(shell_output_func output);

/**
 * \brief Show scheduled automated cells in the shell
 * \param output A pointer to shell_output_func
 */
void msf_housekeeping_show_autonomous_cells(shell_output_func output);

/**
 * \brief Show MSF counters in the shell
 * \param output A pointer to shell_output_func
 */
void msf_housekeeping_show_counters(shell_output_func output);
#endif /* !_MSF_HOUSEKEEPING_H_ */
/** @} */
