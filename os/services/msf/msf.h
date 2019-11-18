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
 * \file
 *         MSF external APIs and configuration
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 */

#ifndef MSF_H
#define MSF_H

#include <stdint.h>

#include "net/linkaddr.h"
#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/sixtop/sixtop.h"
#include "services/shell/shell.h"

/* Constants */
#define MSF_SFID 1
#define MSF_SLOTFRAME_HANDLE_AUTONOMOUS_CELLS 1
#define MSF_SLOTFRAME_HANDLE_NEGOTIATED_CELLS 2
#define MSF_SLOTFRAME_LENGTH TSCH_SCHEDULE_DEFAULT_LENGTH
/*
 * XXX: cannot use
 * TSCH_DEFAULT_TIMESLOT_TIMING[tsch_ts_timeslot_length] for a
 * constant
 */
#define MSF_SLOT_LENGTH_MS 10

#define MSF_6P_CELL_LIST_MAX_LEN 5

/* MSF parameters */
#if MSF_CONF_MAX_NUM_NEGOTIATED_CELLS
#define MSF_MAX_NUM_NEGOTIATED_TX_CELLS MSF_CONF_MAX_NUM_NEGOTIATED_TX_CELLS
#else
#define MSF_MAX_NUM_NEGOTIATED_TX_CELLS 8
#endif /* MSF_CONF_MAX_NUM_OF_NEGOTIATED_TX_CELLS */

#define MSF_MAX_NUM_CELLS 100
#define MSF_LIM_NUM_CELLS_USED_HIGH 75
#define MSF_LIM_NUM_CELLS_USED_LOW 25
#define MSF_HOUSEKEEPING_COLLISION_PERIOD_MIN 1
#define MSF_MIN_NUM_TX_FOR_RELOCATION 128
#define MSF_RELOCATE_PDR_THRESHOLD 50
#define MSF_WAIT_DURATION_MIN_SECONDS 30
#define MSF_WAIT_DURATION_MAX_SECONDS 60

/* SAX parameters */
#define MSF_SAX_H0 0
#define MSF_SAX_L_BIT 0
#define MSF_SAX_R_BIT 1

/* Variables */
extern const sixtop_sf_t msf;

/* Functions */
bool msf_is_activated(void);
bool msf_is_ready(void);
void msf_activate(void);
void msf_deactivate(void);
void msf_shell_sub_cmd(shell_output_func output, char *args);

#endif /* !MSF_H */
